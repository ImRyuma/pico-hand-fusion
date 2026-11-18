// OpenXR implicit API layer that fuses PICO Motion Trackers worn on the wrists
// with camera hand tracking, inside the PICO Connect streaming process.
//
// Chain: PICO Connect -> loader -> this layer -> PICO OpenXR runtime.
// In any other process the layer forwards every call untouched.

#include <pthread.h>
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <openxr/openxr.h>
#include <openxr/openxr_loader_negotiation.h>

#include "config.h"
#include "fusion.h"
#include "log.h"
#include "plthook.h"
#include "recorder.h"
#include "xr_math.h"

#define HF_EXPORT extern "C" __attribute__((visibility("default")))

namespace hf {

int gLogLevel = 1;

namespace {

constexpr const char* kLayerName = "XR_APILAYER_RYUMA_hand_fusion";
constexpr const char* kHostProcesses[] = {"com.picovr.picostreamassistant"};
constexpr int64_t kNoTrackerLogNs = 10000000000LL;
constexpr int64_t kOffsetSaveIntervalNs = 5000000000LL;
constexpr int64_t kStatusIntervalNs = 1000000000LL;

int64_t nowNs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

std::string processName() {
    char buf[256] = {0};
    FILE* f = fopen("/proc/self/cmdline", "r");
    if (!f) return "";
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = 0;
    return std::string(buf);
}

struct State {
    std::mutex m;
    bool active = false;  // true only inside a host process
    std::string dataDir;

    XrInstance instance = XR_NULL_HANDLE;
    XrSession session = XR_NULL_HANDLE;
    bool motionExtEnabled = false;

    PFN_xrGetInstanceProcAddr nextGipa = nullptr;
    PFN_xrDestroyInstance nextDestroyInstance = nullptr;
    PFN_xrCreateSession nextCreateSession = nullptr;
    PFN_xrDestroySession nextDestroySession = nullptr;
    PFN_xrPollEvent nextPollEvent = nullptr;
    PFN_xrCreateHandTrackerEXT nextCreateHandTracker = nullptr;
    PFN_xrDestroyHandTrackerEXT nextDestroyHandTracker = nullptr;
    PFN_xrLocateHandJointsEXT nextLocateHandJoints = nullptr;
    PFN_xrLocateMotionTrackerPICO locateMotionTracker = nullptr;
    PFN_xrRequestMotionTrackerDevicePICO requestMotionTracker = nullptr;
    PFN_xrCreateReferenceSpace createReferenceSpace = nullptr;
    PFN_xrLocateSpace locateSpace = nullptr;
    PFN_xrDestroySpace destroySpace = nullptr;

    std::map<XrHandTrackerEXT, int> handOf;

    // Tracker poses are located by a worker thread, never on PICO Connect's threads: a call into
    // the runtime can take several ms when a tracker is hidden, and blocking there starves the
    // video decoder (frames dropped, key frames re-requested: visible stream lag).
    std::mutex trk;                 // guards everything in this block
    std::mutex workerBusy;          // held by the worker while it calls the runtime
    std::set<uint64_t> trackerIds;
    XrSession workerSession = XR_NULL_HANDLE;
    XrSpace workerSpace = XR_NULL_HANDLE;
    XrTime lastHandTime = 0, prevHandTime = 0;
    TrackerSample workerOut[Fusion::kMaxTrackers];
    int workerCount = 0;
    XrSpace viewSpace = XR_NULL_HANDLE;  // created and used by the worker only
    XrPosef workerView{};                // headset pose in the hand space, for the field of view check
    bool workerViewValid = false;
    bool workerStarted = false;
    bool workerLocateDone = false;
    // Timing (reset at each status write).
    int64_t hookNsSum = 0, hookNsMax = 0, hookCount = 0;
    int64_t trkNsSum = 0, trkNsMax = 0, trkCount = 0;
    int64_t lastProbeNs = 0;
    bool requestedDevices = false;
    bool locateGuardDone = false;
    bool locateGuardLogged = false;
    bool loggedChain = false;

    // Snapshot of the worker's latest tracker poses, taken by the hand tracking hook.
    TrackerSample cache[Fusion::kMaxTrackers];
    int cacheCount = 0;
    XrPosef cacheView{};
    bool cacheViewValid = false;

    ConfigStore config;
    Recorder recorder;
    int64_t epoch = 0;
    Fusion fusion;
    int64_t lastStatusNs = 0;
    uint64_t statusWrites = 0;
    XrPosef savedOffset[2]{};
    bool haveSaved[2] = {false, false};
    float savedLagMs = -1.f;  // measured camera latency last written to the offsets file
    int64_t lastOffsetSaveNs = 0;
    uint64_t locateCalls = 0;
};

State& S() {
    static State s;
    return s;
}

// ---------------------------------------------------------------- crash guard
// The first call of each kind into the runtime's motion tracker API is bracketed by a
// marker file. If the process dies inside it, the marker survives and that call is
// skipped from then on, so PICO Connect cannot crash-loop because of this module.
// Delete the marker (or press "Recalibrate" in the app) to try again.

std::string guardPath(const State& s, const char* what) { return s.dataDir + "/hand_fusion_guard_" + what; }

bool guardTripped(const State& s, const char* what) {
    struct stat st;
    return !s.dataDir.empty() && stat(guardPath(s, what).c_str(), &st) == 0;
}

void guardBegin(const State& s, const char* what) {
    FILE* f = fopen(guardPath(s, what).c_str(), "w");
    if (f) {
        fputs("pending\n", f);
        fflush(f);
        fsync(fileno(f));
        fclose(f);
    }
}

void guardEnd(const State& s, const char* what) { unlink(guardPath(s, what).c_str()); }

// ---------------------------------------------------------------- trackers

bool locateTracker(State& s, uint64_t id, XrSpace space, XrTime time, XrPosef* out, uint64_t* flags) {
    XrMotionTrackerLocationInfoPICO info{XR_TYPE_MOTION_TRACKER_LOCATION_INFO_PICO, nullptr, XR_NULL_HANDLE, 0};
    info.baseSpace = space;
    info.time = time;
    XrMotionTrackerSpaceLocationPICO loc{XR_TYPE_MOTION_TRACKER_SPACE_LOCATION_PICO, nullptr, 0, {}};
    XrResult r = s.locateMotionTracker(s.session, (XrMotionTrackerIdPICO)id, &info, &loc);
    if (XR_FAILED(r)) return false;
    const XrSpaceLocationFlags need = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    if (flags) *flags = loc.locationFlags;
    if ((loc.locationFlags & need) != need) return false;
    *out = loc.pose;
    return true;
}

// Tracker ids are opaque runtime handles: never guess them (an invalid id crashes the
// PICO runtime). They come only from XR_PICO_motion_tracking events, so ask the runtime
// once per session; its "request complete" event lists the connected trackers.
void requestTrackers(State& s, const Config& cfg, int64_t now) {
    {
        std::lock_guard<std::mutex> lk(s.trk);
        if (!s.trackerIds.empty()) return;
    }
    if (!s.requestedDevices && s.requestMotionTracker && cfg.requestDevices > 0) {
        s.requestedDevices = true;
        if (guardTripped(s, "request")) {
            HF_LOGE("trackers: skipping the device request, it crashed last time (guard file present)");
        } else {
            guardBegin(s, "request");
            XrResult r = s.requestMotionTracker(s.session, (uint32_t)cfg.requestDevices);
            guardEnd(s, "request");
            HF_LOGI("trackers: requested %d device(s), result %d", cfg.requestDevices, (int)r);
        }
    }
    if (now - s.lastProbeNs >= kNoTrackerLogNs) {
        s.lastProbeNs = now;
        HF_LOGI("trackers: none reported yet (are they on, and in Object tracking mode?)");
    }
}

// Worker: locates every tracker for the next display time, as soon as a new frame time is known.
void* trackerWorker(void*) {
    State& s = S();
    XrTime lastTarget = 0;
    std::vector<uint64_t> ids;
    for (;;) {
        XrSession session;
        XrSpace space;
        XrTime target;
        {
            std::lock_guard<std::mutex> lk(s.trk);
            session = s.workerSession;
            space = s.workerSpace;
            ids.assign(s.trackerIds.begin(), s.trackerIds.end());
            XrTime period = s.lastHandTime - s.prevHandTime;
            if (period < 0 || period > 20000000LL) period = 0;
            target = s.lastHandTime + period;  // predict for the frame being prepared next
        }
        if (session == XR_NULL_HANDLE || space == XR_NULL_HANDLE || ids.empty() || target == 0 ||
            target == lastTarget || !s.locateMotionTracker) {
            usleep(1000);
            continue;
        }
        lastTarget = target;

        TrackerSample out[Fusion::kMaxTrackers];
        int count = 0;
        {
            std::lock_guard<std::mutex> busy(s.workerBusy);
            {
                std::lock_guard<std::mutex> lk(s.trk);
                if (s.workerSession != session) continue;  // session going away
            }
            if (!s.workerLocateDone) {
                if (guardTripped(s, "locate")) {
                    if (!s.locateGuardLogged) {
                        s.locateGuardLogged = true;
                        HF_LOGE("trackers: locating trackers crashed last time, tracker fusion disabled (guard file present)");
                    }
                    usleep(100000);
                    continue;
                }
                guardBegin(s, "locate");
            }
            const int64_t t0 = nowNs();
            for (uint64_t id : ids) {
                if (count >= Fusion::kMaxTrackers) break;
                TrackerSample& t = out[count++];
                t.id = id;
                uint64_t flags = 0;
                t.valid = locateTracker(s, id, space, target, &t.pose, &flags);
                t.flags = flags;
                // Without the TRACKED bit the runtime only dead-reckons the position (tracker hidden).
                t.positionTracked = (flags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT) != 0;
            }
            // Headset pose for the same time, in the same space (for the field of view check).
            if (s.viewSpace == XR_NULL_HANDLE && s.createReferenceSpace) {
                XrReferenceSpaceCreateInfo ci{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
                ci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
                ci.poseInReferenceSpace = pidentity();
                if (XR_FAILED(s.createReferenceSpace(session, &ci, &s.viewSpace))) s.viewSpace = XR_NULL_HANDLE;
            }
            XrPosef view{};
            bool viewValid = false;
            if (s.viewSpace != XR_NULL_HANDLE && s.locateSpace) {
                XrSpaceLocation vl{XR_TYPE_SPACE_LOCATION};
                const XrSpaceLocationFlags need =
                    XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
                if (XR_SUCCEEDED(s.locateSpace(s.viewSpace, space, target, &vl)) && (vl.locationFlags & need) == need) {
                    view = vl.pose;
                    viewValid = true;
                }
            }
            const int64_t dt = nowNs() - t0;
            if (!s.workerLocateDone) {
                guardEnd(s, "locate");
                s.workerLocateDone = true;
                HF_LOGI("trackers: first locate ok (%d tracker(s), %lld us)", count, (long long)(dt / 1000));
            }
            std::lock_guard<std::mutex> lk(s.trk);
            for (int i = 0; i < count; i++) s.workerOut[i] = out[i];
            s.workerCount = count;
            s.workerView = view;
            s.workerViewValid = viewValid;
            s.trkNsSum += dt;
            s.trkCount++;
            if (dt > s.trkNsMax) s.trkNsMax = dt;
        }
    }
    return nullptr;
}

void startTrackerWorker(State& s) {
    std::lock_guard<std::mutex> lk(s.trk);
    if (s.workerStarted) return;
    pthread_t t;
    if (pthread_create(&t, nullptr, trackerWorker, nullptr) == 0) {
        pthread_detach(t);
        s.workerStarted = true;
    }
}

// Stops the worker from using `session` and waits for a call in flight to finish.
void detachWorker(State& s) {
    std::lock_guard<std::mutex> busy(s.workerBusy);
    std::lock_guard<std::mutex> lk(s.trk);
    if (s.viewSpace != XR_NULL_HANDLE && s.destroySpace) s.destroySpace(s.viewSpace);
    s.viewSpace = XR_NULL_HANDLE;
    s.workerViewValid = false;
    s.workerSession = XR_NULL_HANDLE;
    s.workerSpace = XR_NULL_HANDLE;
    s.workerCount = 0;
}

// Hand tracking hook side: publishes the frame time, takes the latest tracker poses.
void refreshTrackers(State& s, XrSpace space, XrTime time) {
    std::lock_guard<std::mutex> lk(s.trk);
    s.workerSession = s.session;
    s.workerSpace = space;
    if (time != s.lastHandTime) {
        s.prevHandTime = s.lastHandTime;
        s.lastHandTime = time;
    }
    s.cacheCount = s.workerCount;
    for (int i = 0; i < s.workerCount; i++) s.cache[i] = s.workerOut[i];
    s.cacheView = s.workerView;
    s.cacheViewValid = s.workerViewValid;
}

// ---------------------------------------------------------------- offset persistence
// The palm pose in each tracker frame is saved so the next session is calibrated from the start.

std::string offsetsPath(const State& s) { return s.dataDir + "/hand_fusion_offsets.txt"; }

void loadOffsets(State& s) {
    FILE* f = fopen(offsetsPath(s).c_str(), "re");
    if (!f) return;
    char side[16];
    XrPosef p;
    int loaded = 0;
    while (fscanf(f, "%15s %f %f %f %f %f %f %f", side, &p.position.x, &p.position.y, &p.position.z,
                  &p.orientation.x, &p.orientation.y, &p.orientation.z, &p.orientation.w) == 8) {
        if (!strcmp(side, "lag")) {  // measured camera latency (ms), padded to the same 8 fields
            s.fusion.seedCameraLag(p.position.x);
            s.savedLagMs = p.position.x;
            continue;
        }
        const int hand = !strcmp(side, "left") ? 0 : !strcmp(side, "right") ? 1 : -1;
        if (hand >= 0) {
            s.fusion.seedOffset(hand, p);
            s.savedOffset[hand] = p;
            s.haveSaved[hand] = true;
            loaded++;
        }
    }
    fclose(f);
    if (loaded) HF_LOGI("offsets: %d remembered offset(s) loaded", loaded);
}

void saveOffsets(State& s, int64_t now, bool force) {
    if (s.dataDir.empty() || (!force && now - s.lastOffsetSaveNs < kOffsetSaveIntervalNs)) return;
    s.lastOffsetSaveNs = now;
    bool changed = false;
    for (int hand = 0; hand < 2; hand++) {
        XrPosef p;
        if (!s.fusion.learnedOffset(hand, &p)) continue;
        if (!s.haveSaved[hand] || memcmp(&p, &s.savedOffset[hand], sizeof(p)) != 0) changed = true;
        s.savedOffset[hand] = p;
        s.haveSaved[hand] = true;
    }
    const float lag = s.fusion.measuredLagMs();
    if (lag > 0.f && std::fabs(lag - s.savedLagMs) >= 1.f) {
        s.savedLagMs = lag;
        changed = true;
    }
    if (!changed) return;
    const std::string path = offsetsPath(s), tmp = path + ".tmp";
    FILE* f = fopen(tmp.c_str(), "w");
    if (!f) return;
    for (int hand = 0; hand < 2; hand++) {
        if (!s.haveSaved[hand]) continue;
        const XrPosef& p = s.savedOffset[hand];
        fprintf(f, "%s %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", hand ? "right" : "left", p.position.x, p.position.y,
                p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    }
    if (s.savedLagMs > 0.f) fprintf(f, "lag %.1f 0 0 0 0 0 0\n", s.savedLagMs);
    fclose(f);
    rename(tmp.c_str(), path.c_str());
}

void forgetOffsets(State& s) {
    unlink(offsetsPath(s).c_str());
    s.haveSaved[0] = s.haveSaved[1] = false;
    s.savedLagMs = -1.f;
    HF_LOGI("offsets: forgotten (recalibration requested)");
}

void writeStatus(State& s, int64_t now) {
    if (now - s.lastStatusNs < kStatusIntervalNs || s.dataDir.empty()) return;
    s.lastStatusNs = now;
    std::string out;
    char buf[128];
    snprintf(buf, sizeof(buf), "layer_version=3\nenabled=%d\nmotion_ext=%d\nlocate_calls=%llu\nusing_trackers=%d\n",
             s.config.get().enabled ? 1 : 0, s.motionExtEnabled ? 1 : 0, (unsigned long long)s.locateCalls,
             s.fusion.usingTrackers() ? 1 : 0);
    out += buf;
    {
        std::lock_guard<std::mutex> lk(s.trk);
        snprintf(buf, sizeof(buf), "hook_us_avg=%lld\nhook_us_max=%lld\ntracker_us_avg=%lld\ntracker_us_max=%lld\n",
                 (long long)(s.hookCount ? s.hookNsSum / s.hookCount / 1000 : 0), (long long)(s.hookNsMax / 1000),
                 (long long)(s.trkCount ? s.trkNsSum / s.trkCount / 1000 : 0), (long long)(s.trkNsMax / 1000));
        s.hookNsSum = s.hookNsMax = s.hookCount = s.trkNsSum = s.trkNsMax = s.trkCount = 0;
    }
    out += buf;
    out += "trackers=";
    for (int i = 0; i < s.cacheCount; i++) {
        snprintf(buf, sizeof(buf), "%s%llu:%s", i ? "," : "", (unsigned long long)s.cache[i].id,
                 s.cache[i].valid ? "ok" : "lost");
        out += buf;
    }
    out += "\n";
    snprintf(buf, sizeof(buf), "updated_ns=%lld\n", (long long)now);
    out += buf;
    out += s.fusion.status();

    // One-line summary in logcat every 5 s, for tuning sessions.
    if (++s.statusWrites % 5 == 1) {
        std::string line = out;
        for (char& c : line)
            if (c == '\n') c = ' ';
        HF_LOGI("status: %s", line.c_str());
    }

    const std::string path = s.dataDir + "/hand_fusion_status.txt";
    const std::string tmp = path + ".tmp";
    FILE* f = fopen(tmp.c_str(), "w");
    if (!f) return;
    fwrite(out.data(), 1, out.size(), f);
    fclose(f);
    rename(tmp.c_str(), path.c_str());
}

// ---------------------------------------------------------------- hooks

XrResult XRAPI_CALL hook_xrDestroyInstance(XrInstance instance) {
    State& s = S();
    PFN_xrDestroyInstance next = s.nextDestroyInstance;
    {
        std::lock_guard<std::mutex> lk(s.m);
        if (instance == s.instance) {
            detachWorker(s);
            s.instance = XR_NULL_HANDLE;
            s.session = XR_NULL_HANDLE;
            s.handOf.clear();
            {
                std::lock_guard<std::mutex> t(s.trk);
                s.trackerIds.clear();
            }
            s.cacheCount = 0;
            saveOffsets(s, nowNs(), true);
            s.fusion.reset();
        }
    }
    return next(instance);
}

XrResult XRAPI_CALL hook_xrCreateSession(XrInstance instance, const XrSessionCreateInfo* info, XrSession* session) {
    State& s = S();
    XrResult r = s.nextCreateSession(instance, info, session);
    if (XR_SUCCEEDED(r)) {
        std::lock_guard<std::mutex> lk(s.m);
        s.session = *session;
        s.requestedDevices = false;
        HF_LOGI("session created");
        startTrackerWorker(s);
    }
    return r;
}

XrResult XRAPI_CALL hook_xrDestroySession(XrSession session) {
    State& s = S();
    {
        std::lock_guard<std::mutex> lk(s.m);
        if (session == s.session) {
            detachWorker(s);
            s.session = XR_NULL_HANDLE;
            s.cacheCount = 0;
        }
    }
    return s.nextDestroySession(session);
}

XrResult XRAPI_CALL hook_xrPollEvent(XrInstance instance, XrEventDataBuffer* event) {
    State& s = S();
    XrResult r = s.nextPollEvent(instance, event);
    if (r != XR_SUCCESS || !event) return r;
    if (event->type != XR_TYPE_EVENT_DATA_REQUEST_MOTION_TRACKER_COMPLETE_PICO &&
        event->type != XR_TYPE_EVENT_DATA_MOTION_TRACKER_CONNECTION_STATE_CHANGED_PICO)
        return r;
    std::lock_guard<std::mutex> lk(s.trk);
    switch (event->type) {
        case XR_TYPE_EVENT_DATA_REQUEST_MOTION_TRACKER_COMPLETE_PICO: {
            auto* e = reinterpret_cast<const XrEventDataRequestMotionTrackerCompletePICO*>(event);
            HF_LOGI("trackers: request complete, result %d, %u tracker(s)", (int)e->result, e->trackerCount);
            for (uint32_t i = 0; i < e->trackerCount && i < XR_MOTION_TRACKER_MAX_SIZE_PICO; i++)
                s.trackerIds.insert((uint64_t)e->trackerIds[i]);
            break;
        }
        case XR_TYPE_EVENT_DATA_MOTION_TRACKER_CONNECTION_STATE_CHANGED_PICO: {
            auto* e = reinterpret_cast<const XrEventDataMotionTrackerConnectionStateChangedPICO*>(event);
            const bool connected = e->state == XR_MOTION_TRACKER_CONNECTION_STATE_CONNECTED_PICO;
            HF_LOGI("trackers: id %llu %s", (unsigned long long)e->trackerId, connected ? "connected" : "disconnected");
            if (connected) s.trackerIds.insert((uint64_t)e->trackerId);
            else s.trackerIds.erase((uint64_t)e->trackerId);
            break;
        }
        default:
            break;
    }
    return r;
}

XrResult XRAPI_CALL hook_xrCreateHandTrackerEXT(XrSession session, const XrHandTrackerCreateInfoEXT* info,
                                                XrHandTrackerEXT* tracker) {
    State& s = S();
    XrResult r = s.nextCreateHandTracker(session, info, tracker);
    if (XR_SUCCEEDED(r) && info) {
        std::lock_guard<std::mutex> lk(s.m);
        s.handOf[*tracker] = info->hand == XR_HAND_LEFT_EXT ? 0 : 1;
        HF_LOGI("hand tracker created for %s hand", info->hand == XR_HAND_LEFT_EXT ? "left" : "right");
    }
    return r;
}

XrResult XRAPI_CALL hook_xrDestroyHandTrackerEXT(XrHandTrackerEXT tracker) {
    State& s = S();
    {
        std::lock_guard<std::mutex> lk(s.m);
        s.handOf.erase(tracker);
    }
    return s.nextDestroyHandTracker(tracker);
}

XrResult XRAPI_CALL hook_xrLocateHandJointsEXT(XrHandTrackerEXT tracker, const XrHandJointsLocateInfoEXT* info,
                                               XrHandJointLocationsEXT* locations) {
    State& s = S();
    XrResult r = s.nextLocateHandJoints(tracker, info, locations);
    if (XR_FAILED(r) || !info || !locations) return r;

    std::lock_guard<std::mutex> lk(s.m);
    s.locateCalls++;
    const int64_t now = nowNs();
    struct HookTimer {
        State& s;
        int64_t t0;
        ~HookTimer() {
            const int64_t dt = nowNs() - t0;
            std::lock_guard<std::mutex> l(s.trk);
            s.hookNsSum += dt;
            s.hookCount++;
            if (dt > s.hookNsMax) s.hookNsMax = dt;
        }
    } timer{s, now};
    if (s.config.poll(now)) {
        const Config& c = s.config.get();
        if (c.calibEpoch != s.epoch) {
            s.epoch = c.calibEpoch;
            s.fusion.reset();
            forgetOffsets(s);
            s.fusion.resetCameraLag();
        }
        s.recorder.arm(c.recordEpoch, c.recordSeconds, now);
    }
    const Config& cfg = s.config.get();

    if (!s.loggedChain) {
        s.loggedChain = true;
        std::string chain;
        for (auto* p = reinterpret_cast<const XrBaseOutStructure*>(locations->next); p; p = p->next) {
            chain += std::to_string((int)p->type) + " ";
        }
        HF_LOGI("first locate: jointCount=%u, chained types: %s", locations->jointCount,
                chain.empty() ? "none" : chain.c_str());
    }

    auto it = s.handOf.find(tracker);
    if (it == s.handOf.end() || !cfg.enabled || !s.locateMotionTracker || s.session == XR_NULL_HANDLE) {
        writeStatus(s, now);
        return r;
    }

    requestTrackers(s, cfg, now);
    refreshTrackers(s, info->baseSpace, info->time);
    if (s.recorder.active()) {
        const int64_t id = s.fusion.assigned(it->second);
        const TrackerSample* t = nullptr;
        for (int i = 0; i < s.cacheCount; i++)
            if ((int64_t)s.cache[i].id == id) t = &s.cache[i];
        s.recorder.captureInput(it->second, info->time, locations, t != nullptr, t ? t->id : 0, t && t->valid,
                                t ? t->pose : XrPosef{}, t ? (uint32_t)t->flags : 0);
    }
    s.fusion.process(it->second, info->time, s.cache, s.cacheCount, locations, cfg,
                     s.cacheViewValid ? &s.cacheView : nullptr);
    if (s.recorder.active()) s.recorder.captureOutput(it->second, locations, s.fusion.mode(it->second),
                                                     s.fusion.shapeDelta(it->second), s.fusion.viewAzimuth(it->second),
                                                     s.fusion.viewElevation(it->second));
    s.recorder.finishIfDue(now, s.dataDir);
    saveOffsets(s, now, false);
    writeStatus(s, now);
    return r;
}

// ---------------------------------------------------------------- dispatch

XrResult XRAPI_CALL layer_xrGetInstanceProcAddr(XrInstance instance, const char* name, PFN_xrVoidFunction* function) {
    State& s = S();
    if (!s.nextGipa) return XR_ERROR_HANDLE_INVALID;
    if (s.active && name && function) {
#define HF_HOOK(fn)                                                       \
    if (!strcmp(name, #fn)) {                                             \
        XrResult r = s.nextGipa(instance, name, function);                \
        if (XR_SUCCEEDED(r) && *function) *function = (PFN_xrVoidFunction)hook_##fn; \
        return r;                                                         \
    }
        HF_HOOK(xrDestroyInstance)
        HF_HOOK(xrCreateSession)
        HF_HOOK(xrDestroySession)
        HF_HOOK(xrPollEvent)
        HF_HOOK(xrCreateHandTrackerEXT)
        HF_HOOK(xrDestroyHandTrackerEXT)
        HF_HOOK(xrLocateHandJointsEXT)
#undef HF_HOOK
    }
    return s.nextGipa(instance, name, function);
}

template <typename T>
void resolve(State& s, XrInstance instance, const char* name, T* out) {
    PFN_xrVoidFunction fn = nullptr;
    if (XR_SUCCEEDED(s.nextGipa(instance, name, &fn))) *out = reinterpret_cast<T>(fn);
    else *out = nullptr;
}

// Instance creation shared by both entry points; `create` calls the next implementation.
template <typename CreateFn>
XrResult createInstanceCommon(const XrInstanceCreateInfo* info, XrInstance* instance, CreateFn create) {
    State& s = S();
    // PICO Connect streams from a sub-process ("<package>:deviceservice"): match on the package.
    const std::string proc = processName();
    const std::string package = proc.substr(0, proc.find(':'));
    s.active = false;
    for (const char* host : kHostProcesses)
        if (package == host) s.active = true;

    if (!s.active || !info) {
        HF_LOGD("passthrough in %s", proc.c_str());
        return create(info);
    }

    HF_LOGI("active in %s", proc.c_str());
    s.dataDir = "/data/data/" + package + "/files";
    mkdir(s.dataDir.c_str(), 0700);
    s.config.init(s.dataDir);
    s.config.poll(nowNs());
    // Baselines: only later changes of these values reset the calibration or start a recording.
    s.epoch = s.config.get().calibEpoch;
    s.recorder.arm(s.config.get().recordEpoch, s.config.get().recordSeconds, nowNs());
    loadOffsets(s);

    // Enable XR_PICO_motion_tracking if the application did not.
    bool already = false;
    for (uint32_t i = 0; i < info->enabledExtensionCount; i++)
        if (!strcmp(info->enabledExtensionNames[i], XR_PICO_MOTION_TRACKING_EXTENSION_NAME)) already = true;

    XrResult r;
    if (already) {
        r = create(info);
        s.motionExtEnabled = XR_SUCCEEDED(r);
    } else {
        std::vector<const char*> exts(info->enabledExtensionNames,
                                      info->enabledExtensionNames + info->enabledExtensionCount);
        exts.push_back(XR_PICO_MOTION_TRACKING_EXTENSION_NAME);
        XrInstanceCreateInfo patched = *info;
        patched.enabledExtensionCount = (uint32_t)exts.size();
        patched.enabledExtensionNames = exts.data();
        r = create(&patched);
        s.motionExtEnabled = XR_SUCCEEDED(r);
        if (r == XR_ERROR_EXTENSION_NOT_PRESENT) {
            HF_LOGE("runtime refused %s, fusion disabled", XR_PICO_MOTION_TRACKING_EXTENSION_NAME);
            r = create(info);
        }
    }
    if (XR_FAILED(r)) return r;

    std::lock_guard<std::mutex> lk(s.m);
    s.instance = *instance;
    resolve(s, *instance, "xrDestroyInstance", &s.nextDestroyInstance);
    resolve(s, *instance, "xrCreateSession", &s.nextCreateSession);
    resolve(s, *instance, "xrDestroySession", &s.nextDestroySession);
    resolve(s, *instance, "xrPollEvent", &s.nextPollEvent);
    resolve(s, *instance, "xrCreateHandTrackerEXT", &s.nextCreateHandTracker);
    resolve(s, *instance, "xrDestroyHandTrackerEXT", &s.nextDestroyHandTracker);
    resolve(s, *instance, "xrLocateHandJointsEXT", &s.nextLocateHandJoints);
    if (s.motionExtEnabled) {
        resolve(s, *instance, "xrLocateMotionTrackerPICO", &s.locateMotionTracker);
        resolve(s, *instance, "xrRequestMotionTrackerDevicePICO", &s.requestMotionTracker);
        resolve(s, *instance, "xrCreateReferenceSpace", &s.createReferenceSpace);
        resolve(s, *instance, "xrLocateSpace", &s.locateSpace);
        resolve(s, *instance, "xrDestroySpace", &s.destroySpace);
    }
    HF_LOGI("instance created, motion tracking %s", s.locateMotionTracker ? "available" : "unavailable");
    return r;
}

// ---------------------------------------------------------------- API layer entry point
// Used by loaders that load implicit API layers (the Khronos loader of most system apps).

XrResult XRAPI_CALL layer_xrCreateApiLayerInstance(const XrInstanceCreateInfo* info,
                                                    const XrApiLayerCreateInfo* layerInfo, XrInstance* instance) {
    if (!layerInfo || !layerInfo->nextInfo || strcmp(layerInfo->nextInfo->layerName, kLayerName) != 0) {
        HF_LOGE("unexpected layer chain");
        return XR_ERROR_INITIALIZATION_FAILED;
    }
    State& s = S();
    s.nextGipa = layerInfo->nextInfo->nextGetInstanceProcAddr;
    PFN_xrCreateApiLayerInstance nextCreate = layerInfo->nextInfo->nextCreateApiLayerInstance;
    XrApiLayerCreateInfo chained = *layerInfo;
    chained.nextInfo = layerInfo->nextInfo->next;
    return createInstanceCommon(info, instance, [&](const XrInstanceCreateInfo* ci) {
        return nextCreate(ci, &chained, instance);
    });
}

// ---------------------------------------------------------------- in-process entry point (Zygisk)
// PICO Connect's bundled loader (PxrLoader 1.0.24) ignores API layers and loads the runtime
// through the PICO runtime's Java DriverLoader, so neither a layer nor a runtime manifest
// reaches it. The Zygisk module instead patches the import table (GOT) of
// libstreaming_server.so inside the streaming process.

constexpr const char* kStreamingLib = "/libstreaming_server.so";
constexpr const char* kLoaderLib = "/libpico_openxr_loader.so";

PFN_xrCreateInstance gOrigCreateInstance = nullptr;

XrResult XRAPI_CALL got_xrCreateInstance(const XrInstanceCreateInfo* info, XrInstance* instance) {
    PFN_xrCreateInstance real = gOrigCreateInstance;
    if (!real) return XR_ERROR_RUNTIME_FAILURE;
    return createInstanceCommon(info, instance, [&](const XrInstanceCreateInfo* ci) { return real(ci, instance); });
}

struct GotTarget {
    const char* name;
    void* hook;
    void** original;  // where the original function pointer is kept
};

bool patchStreamingLib(const LibMapping& lib, const LibMapping& loader, int* patched) {
    State& s = S();
    GotTarget targets[] = {
        {"xrGetInstanceProcAddr", (void*)layer_xrGetInstanceProcAddr, (void**)&s.nextGipa},
        {"xrCreateInstance", (void*)got_xrCreateInstance, (void**)&gOrigCreateInstance},
        {"xrDestroyInstance", (void*)hook_xrDestroyInstance, (void**)&s.nextDestroyInstance},
        {"xrCreateSession", (void*)hook_xrCreateSession, (void**)&s.nextCreateSession},
        {"xrDestroySession", (void*)hook_xrDestroySession, (void**)&s.nextDestroySession},
        {"xrPollEvent", (void*)hook_xrPollEvent, (void**)&s.nextPollEvent},
    };
    int done = 0;
    bool ok = forEachJumpSlot(lib.base, [&](const char* sym, void** slot) {
        for (GotTarget& t : targets) {
            if (strcmp(sym, t.name) != 0) continue;
            void* cur = __atomic_load_n(slot, __ATOMIC_SEQ_CST);
            if (cur == t.hook) {
                done++;
            } else {
                const uintptr_t a = reinterpret_cast<uintptr_t>(cur);
                // Only patch once the linker has bound the slot to the loader.
                if (a >= loader.start && a < loader.end) {
                    *t.original = cur;
                    if (writeSlot(slot, t.hook)) done++;
                    else HF_LOGE("got: cannot patch %s", sym);
                }
            }
        }
    });
    *patched = done;
    return ok && done == (int)(sizeof(targets) / sizeof(targets[0]));
}

void* hookThreadMain(void*) {
    // The streaming server may be loaded long after the process starts: wait as long as needed,
    // poll fast once it is mapped (the linker binds its imports right after), then keep checking
    // for a few seconds that nothing rebinds the slots.
    constexpr int kIdlePollUs = 20000;
    constexpr int kFastPollUs = 1000;
    constexpr int kStableUs = 10 * 1000000;
    int stable = 0, patched = 0, waitedMapped = 0;
    bool announced = false;
    while (stable < kStableUs) {
        const LibMapping lib = findLibrary(kStreamingLib);
        const LibMapping loader = lib.found ? findLibrary(kLoaderLib) : LibMapping();
        if (!lib.found || !loader.found) {
            usleep(kIdlePollUs);
            continue;
        }
        if (patchStreamingLib(lib, loader, &patched)) {
            if (!announced) {
                announced = true;
                HF_LOGI("got: streaming server hooked (%d imports)", patched);
            }
            stable += kFastPollUs * 10;
            usleep(kFastPollUs * 10);
        } else {
            stable = 0;
            if ((waitedMapped += kFastPollUs) % 5000000 == 0)
                HF_LOGI("got: streaming server mapped, %d/6 imports bound so far", patched);
            usleep(kFastPollUs);
        }
    }
    return nullptr;
}

}  // namespace
}  // namespace hf

HF_EXPORT XrResult XRAPI_CALL xrNegotiateLoaderApiLayerInterface(const XrNegotiateLoaderInfo* loaderInfo,
                                                                  const char* layerName,
                                                                  XrNegotiateApiLayerRequest* request) {
    if (!loaderInfo || !request || loaderInfo->structType != XR_LOADER_INTERFACE_STRUCT_LOADER_INFO ||
        request->structType != XR_LOADER_INTERFACE_STRUCT_API_LAYER_REQUEST) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }
    if (layerName && strcmp(layerName, hf::kLayerName) != 0) return XR_ERROR_INITIALIZATION_FAILED;
    if (loaderInfo->minInterfaceVersion > XR_CURRENT_LOADER_API_LAYER_VERSION ||
        loaderInfo->maxInterfaceVersion < XR_CURRENT_LOADER_API_LAYER_VERSION) {
        return XR_ERROR_INITIALIZATION_FAILED;
    }
    {
        const std::string proc = hf::processName();
        if (proc.rfind(hf::kHostProcesses[0], 0) == 0) HF_LOGI("layer loaded in %s", proc.c_str());
    }
    request->layerInterfaceVersion = XR_CURRENT_LOADER_API_LAYER_VERSION;
    request->layerApiVersion =
        loaderInfo->maxApiVersion < XR_CURRENT_API_VERSION ? loaderInfo->maxApiVersion : XR_CURRENT_API_VERSION;
    request->getInstanceProcAddr = hf::layer_xrGetInstanceProcAddr;
    request->createApiLayerInstance = hf::layer_xrCreateApiLayerInstance;
    return XR_SUCCESS;
}

namespace hf {
// Called by the Zygisk module inside PICO Connect's streaming process.
void startInProcessHooks() {
    pthread_t t;
    if (pthread_create(&t, nullptr, hookThreadMain, nullptr) == 0) {
        pthread_detach(t);
        HF_LOGI("waiting for the streaming server in %s", processName().c_str());
    }
}
}  // namespace hf
