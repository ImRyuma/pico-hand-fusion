#include "fusion.h"

#include <cmath>
#include <cstdio>

#include "log.h"
#include "xr_math.h"

namespace hf {

namespace {

constexpr XrSpaceLocationFlags kValid = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
constexpr XrSpaceLocationFlags kTracked =
    XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
constexpr XrSpaceLocationFlags kAll = kValid | kTracked;

constexpr int kAssignFrames = 30;      // ~0.5 s closest to a palm before locking a tracker to it
constexpr float kUnassignDist = 0.30f;  // m, tracker clearly no longer on this hand...
constexpr int kUnassignFrames = 180;    // ...for ~2 s

// Learning. The camera is ~10x noisier than the tracker (~20 mm vs ~2 mm RMS) and lags it, so
// samples are compared with the tracker pose from `camera_lag_ms` ago, and averaged over many frames.
constexpr float kLearnMaxSpeed = 0.6f;     // m/s, faster samples are skipped
constexpr float kLearnMaxAngSpeed = 3.0f;  // rad/s
constexpr float kGatePos = 0.06f;          // m, refinement sample rejected beyond this (3 sigma of camera noise)
constexpr float kGateAngle = 30.f;         // deg
constexpr float kErrAvgRate = 0.02f;       // smoothing of the disagreement (~0.6 s at 90 Hz)
constexpr float kRelearnErr = 0.08f;       // m, lasting average disagreement that triggers a new learning
constexpr int kRelearnFrames = 450;        // ~5 s

constexpr int kFrozenFrames = 8;                  // identical camera palm positions in a row (~90 ms)
// Out of the cameras' view the runtime may keep moving the last skeleton seen as one rigid block, flagged
// as tracked. Real camera frames always reshape the fingers a little.
constexpr float kRigidShapeEps = 0.00003f;  // m, largest finger joint move in the palm frame between fresh frames
constexpr float kRigidMinPalmMove = 0.001f; // m, palm move needed for the frame to count (else it is just still)
constexpr int kRigidFrames = 3;             // rigid fresh frames in a row before the camera is distrusted
constexpr float kJumpDist = 0.08f;               // m in one frame (> 7 m/s): the runtime re-snapped the tracker
constexpr int64_t kJumpDistrustNs = 300000000LL;  // tracker position ignored this long after a jump
constexpr float kDeadReckonTau = 0.15f;          // s, how fast a hidden tracker is pulled toward the camera
constexpr float kDeadReckonFastTau = 0.05f;      // s, same when far off
constexpr float kDeadReckonFarErr = 0.10f;       // m
constexpr float kSourceBlendTau = 0.25f;          // s, time constant of the position hand-over between sources
constexpr float kResidualLatency = 0.03f;          // s, outlier threshold grows by this * tracker speed
constexpr int64_t kOutlierGraceNs = 2000000000LL;  // outlier rejection only once the offset is settled
constexpr float kRadToDeg = 57.2957795f;
constexpr float kDefaultDt = 1.f / 90.f;
constexpr int64_t kFilterResetNs = 500000000LL;

// Camera latency measurement: during moves, the lag that best lines the camera palm up with the
// tracker-predicted palm (smallest residual spread) is the camera latency.
constexpr float kLagMinSpeed = 0.25f;           // m/s, slower frames carry no timing information
constexpr int64_t kLagMaxGapNs = 150000000LL;   // a longer gap between samples restarts the window
constexpr int kLagMaxMs = 150;
constexpr int kLagStepMs = 5;
constexpr float kLagMaxRms = 0.03f;             // m, residual spread above this: window rejected
constexpr float kLagMinContrast = 0.6f;         // best spread must be below this fraction of the worst (flat curve: no information)
constexpr float kLagRate = 0.15f;               // smoothing of the estimate across windows
// Average finger shape (relaxed pose used out of view): ~5 s time constant on fresh frames.
constexpr float kMeanShapeRate = 0.005f;

XrHandJointVelocitiesEXT* findVelocities(XrHandJointLocationsEXT* locs) {
    auto* p = reinterpret_cast<XrBaseOutStructure*>(locs->next);
    while (p) {
        if (p->type == XR_TYPE_HAND_JOINT_VELOCITIES_EXT) return reinterpret_cast<XrHandJointVelocitiesEXT*>(p);
        p = p->next;
    }
    return nullptr;
}

// True when `p` is outside the hand tracking cameras' field of view (headset pose `view`, OpenXR
// view axes: -Z forward, +X right, +Y up). Also fills the direction angles.
bool outsideFov(const XrPosef& view, const XrVector3f& p, const Config& cfg, float* az, float* el) {
    const XrVector3f l = pmul(pinv(view), XrPosef{qidentity(), p}).position;
    const float fwd = -l.z;
    *az = std::atan2(l.x, fwd) * 57.2957795f;
    *el = std::atan2(l.y, fwd) * 57.2957795f;
    if (fwd < 0.05f) return true;  // beside or behind the head
    return std::fabs(*az) > cfg.fovSideDeg || *el > cfg.fovUpDeg || *el < -cfg.fovDownDeg;
}

bool isManual(int hand, const Config& cfg) { return (hand == 0 ? cfg.leftTracker : cfg.rightTracker) >= 0; }

}  // namespace

const char* modeName(HandMode m) {
    switch (m) {
        case HandMode::Idle: return "idle";
        case HandMode::Camera: return "camera";
        case HandMode::Learning: return "learning";
        case HandMode::Fused: return "fused";
        case HandMode::TrackerOnly: return "tracker_only";
        case HandMode::Lost: return "lost";
        case HandMode::ImuCamera: return "imu_camera";
    }
    return "?";
}

void Fusion::reset() {
    hands_[0] = Hand();
    hands_[1] = Hand();
    HF_LOGI("fusion: reset");
}

bool Fusion::usingTrackers() const {
    for (const Hand& h : hands_)
        if (h.assigned >= 0) return true;
    return false;
}

void Fusion::resetFilters(Hand& h) {
    h.palmFilter.reset();
    for (auto& f : h.jointFilter) f.reset();
    h.lastOutTime = 0;
}

void Fusion::unassign(Hand& h) {
    if (h.samples > 0 && h.learnedAt > 0) {
        h.seed = h.offset;  // the tracker stays rigid on the back of the hand: reuse it
        h.haveSeed = true;
    }
    h.assigned = -1;
    h.candidate = -1;
    h.candidateFrames = 0;
    h.farFrames = 0;
    h.samples = 0;
    h.relearning = false;
    h.histCount = 0;
}

const TrackerSample* Fusion::find(int64_t id, const TrackerSample* trackers, int n) const {
    if (id < 0) return nullptr;
    for (int i = 0; i < n; i++)
        if ((int64_t)trackers[i].id == id) return &trackers[i];
    return nullptr;
}

// Starts from the remembered offset (previous assignment or previous session) when there is one;
// the lasting-disagreement check relearns it if the tracker moved on the hand.
void Fusion::applySeed(Hand& h, int hand, const Config& cfg) {
    h.samples = 0;
    h.relearning = false;
    if (!h.haveSeed) return;
    h.offset = h.seed;
    h.samples = cfg.minSamples;
    h.learnedAt = 0;  // set on the next frame: outlier rejection waits for the usual grace period
    h.seeded = true;
    HF_LOGI("fusion: hand %d starts from its remembered offset", hand);
}

void Fusion::seedOffset(int hand, const XrPosef& offset) {
    if (hand < 0 || hand > 1) return;
    hands_[hand].seed = offset;
    hands_[hand].haveSeed = true;
}

void Fusion::seedCameraLag(float ms) {
    if (ms > 0.f && ms <= (float)kLagMaxMs) lagEstMs_ = ms;
}

int64_t Fusion::lagNs(const Config& cfg) const {
    const float ms = (cfg.cameraLagAuto && lagEstMs_ > 0.f) ? lagEstMs_ : (float)cfg.cameraLagMs;
    return (int64_t)(ms * 1e6f);
}

void Fusion::measureLag(Hand& h, XrTime time, const XrVector3f& camPalm) {
    if (h.speed < kLagMinSpeed || (h.lagCount > 0 && time - h.lagTime[h.lagCount - 1] > kLagMaxGapNs)) {
        h.lagCount = 0;
        if (h.speed < kLagMinSpeed) return;
    }
    h.lagTime[h.lagCount] = time;
    h.lagCam[h.lagCount] = camPalm;
    if (++h.lagCount < kLagWindow) return;
    h.lagCount = 0;

    // The whole window must be covered by the tracker history, even at the largest lag.
    const int oldest = (h.histHead - h.histCount + kHistory) % kHistory;
    if (h.histCount < kHistory / 2 || h.lagTime[0] - (int64_t)kLagMaxMs * 1000000LL < h.histTime[oldest]) return;

    constexpr int kSteps = kLagMaxMs / kLagStepMs + 1;
    float spread[kSteps];
    int best = 0;
    float worst = 0.f;
    for (int k = 0; k < kSteps; k++) {
        const int64_t lag = (int64_t)k * kLagStepMs * 1000000LL;
        XrVector3f r[kLagWindow];
        XrVector3f mean = v3(0.f, 0.f, 0.f);
        for (int i = 0; i < kLagWindow; i++) {
            r[i] = sub(h.lagCam[i], pmul(trackerAt(h, h.lagTime[i] - lag), h.offset).position);
            mean = add(mean, r[i]);
        }
        mean = scale(mean, 1.f / kLagWindow);
        float var = 0.f;
        for (int i = 0; i < kLagWindow; i++) {
            const XrVector3f d = sub(r[i], mean);
            var += dot(d, d);
        }
        spread[k] = var / kLagWindow;
        if (spread[k] < spread[best]) best = k;
        worst = std::fmax(worst, spread[k]);
    }
    // A minimum on the edge of the range, a flat curve or a large residual is not a measurement.
    if (best == 0 || best == kSteps - 1 || spread[best] > kLagMinContrast * worst ||
        std::sqrt(spread[best]) > kLagMaxRms)
        return;
    // Parabola through the three points around the minimum for a sub-step estimate.
    const float a = spread[best - 1], b = spread[best], c = spread[best + 1];
    const float den = a - 2.f * b + c;
    const float frac = den > 0.f ? 0.5f * (a - c) / den : 0.f;
    const float ms = ((float)best + std::fmax(-0.5f, std::fmin(0.5f, frac))) * kLagStepMs;
    lagEstMs_ = lagEstMs_ < 0.f ? ms : lagEstMs_ + kLagRate * (ms - lagEstMs_);
    lagUpdates_++;
    lagRmsMm_ = std::sqrt(spread[best]) * 1000.f;
    if (lagUpdates_ <= 3 || lagUpdates_ % 20 == 0)
        HF_LOGI("fusion: camera latency %.0f ms (window %.0f ms, residual %.0f mm, %d windows)", lagEstMs_, ms,
                lagRmsMm_, lagUpdates_);
}

bool Fusion::learnedOffset(int hand, XrPosef* out) const {
    const Hand& h = hands_[hand];
    if (h.assigned < 0 || h.learnedAt <= 0) return false;
    *out = h.offset;
    return true;
}

void Fusion::assign(int hand, const XrVector3f& palm, const TrackerSample* trackers, int n, const Config& cfg) {
    Hand& h = hands_[hand];
    const int64_t manual = hand == 0 ? cfg.leftTracker : cfg.rightTracker;
    if (manual >= 0) {
        if (h.assigned != manual) {
            unassign(h);
            h.assigned = manual;
            applySeed(h, hand, cfg);
            HF_LOGI("fusion: hand %d uses tracker %lld (manual)", hand, (long long)manual);
        }
        return;
    }
    if (h.assigned >= 0) return;

    const int64_t other = hands_[1 - hand].assigned;
    int64_t best = -1;
    float bestDist = cfg.assignMaxDist;
    for (int i = 0; i < n; i++) {
        if (!trackers[i].valid || (int64_t)trackers[i].id == other) continue;
        const float d = length(sub(trackers[i].pose.position, palm));
        if (d < bestDist) {
            bestDist = d;
            best = (int64_t)trackers[i].id;
        }
    }
    if (best < 0) {
        h.candidate = -1;
        h.candidateFrames = 0;
        return;
    }
    if (best == h.candidate) {
        if (++h.candidateFrames >= kAssignFrames) {
            h.assigned = best;
            applySeed(h, hand, cfg);
            HF_LOGI("fusion: hand %d assigned to tracker %lld (%.3f m from the palm)", hand, (long long)best,
                    bestDist);
        }
    } else {
        h.candidate = best;
        h.candidateFrames = 1;
    }
}

void Fusion::pushTracker(Hand& h, XrTime time, const XrPosef& pose) {
    if (h.histCount > 0) {
        const int last = (h.histHead + kHistory - 1) % kHistory;
        const XrTime dtNs = time - h.histTime[last];
        if (dtNs <= 0) return;  // same frame (both hands located with the same time)
        if (dtNs < 100000000LL) {
            const float dt = (float)dtNs * 1e-9f;
            const float step = length(sub(pose.position, h.histPose[last].position));
            if (step > kJumpDist) {
                h.untrustedUntil = time + kJumpDistrustNs;
                h.jumps++;
            }
            const float lin = step / dt;
            const float ang = qangle(pose.orientation, h.histPose[last].orientation) / dt;
            h.speed += 0.3f * (lin - h.speed);
            h.angSpeed += 0.3f * (ang - h.angSpeed);
        } else {
            h.speed = h.angSpeed = 0.f;
        }
    }
    h.histTime[h.histHead] = time;
    h.histPose[h.histHead] = pose;
    h.histHead = (h.histHead + 1) % kHistory;
    if (h.histCount < kHistory) h.histCount++;
}

// Tracker pose at `time`, interpolated from the history (clamped to the oldest/newest sample).
XrPosef Fusion::trackerAt(const Hand& h, XrTime time) const {
    const int newest = (h.histHead + kHistory - 1) % kHistory;
    if (h.histCount == 0) return pidentity();
    if (time >= h.histTime[newest]) return h.histPose[newest];
    for (int k = 1; k < h.histCount; k++) {
        const int i = (newest - k + kHistory) % kHistory;
        const int j = (i + 1) % kHistory;
        if (h.histTime[i] <= time) {
            const float span = (float)(h.histTime[j] - h.histTime[i]);
            const float a = span > 0.f ? (float)(time - h.histTime[i]) / span : 0.f;
            return pblend(h.histPose[i], h.histPose[j], a);
        }
    }
    return h.histPose[(newest - h.histCount + 1 + kHistory) % kHistory];
}

void Fusion::learn(Hand& h, XrTime time, const XrPosef& lagged, const XrPosef& palm, const Config& cfg) {
    if (h.speed > kLearnMaxSpeed || h.angSpeed > kLearnMaxAngSpeed) return;
    const XrPosef sample = pmul(pinv(lagged), palm);
    if (h.samples < cfg.minSamples) {
        // Plain running mean: camera noise averages out over many frames.
        h.offset = h.samples == 0 ? sample : pblend(h.offset, sample, 1.f / (float)(h.samples + 1));
        if (++h.samples == cfg.minSamples) {
            h.learnedAt = time;
            h.errAvg = 0.f;
            h.disagreeFrames = 0;
            HF_LOGI("fusion: offset learned (%d samples)", h.samples);
        }
        return;
    }
    if (h.relearning) {
        // A new offset is learned in the background; the current one keeps driving the hand.
        h.candOffset = h.candSamples == 0 ? sample : pblend(h.candOffset, sample, 1.f / (float)(h.candSamples + 1));
        if (++h.candSamples >= cfg.minSamples) {
            const float moved = length(sub(h.candOffset.position, h.offset.position));
            const float turned = qangle(h.candOffset.orientation, h.offset.orientation) * kRadToDeg;
            h.offset = h.candOffset;
            h.relearning = false;
            h.seeded = false;
            h.learnedAt = time;
            h.errAvg = 0.f;
            h.disagreeFrames = 0;
            HF_LOGI("fusion: new offset applied (moved %.0f mm, %.0f deg)", moved * 1000.f, turned);
        }
        return;
    }
    const XrPosef predicted = pmul(lagged, h.offset);
    const float ePos = length(sub(predicted.position, palm.position));
    const float eAng = qangle(predicted.orientation, palm.orientation) * kRadToDeg;
    if (ePos < kGatePos && eAng < kGateAngle) h.offset = pblend(h.offset, sample, cfg.learnRate);
    h.errAvg += kErrAvgRate * (ePos - h.errAvg);
    if (h.errAvg > kRelearnErr) {
        if (++h.disagreeFrames > kRelearnFrames) {
            HF_LOGI("fusion: lasting disagreement (avg %.3f m): learning a new offset in the background", h.errAvg);
            h.relearning = true;
            h.candSamples = 0;
            h.disagreeFrames = 0;
            h.relearns++;
        }
    } else {
        h.disagreeFrames = 0;
    }
}

void Fusion::process(int hand, XrTime time, const TrackerSample* trackers, int trackerCount,
                     XrHandJointLocationsEXT* locs, const Config& cfg, const XrPosef* view) {
    if (hand < 0 || hand > 1 || !locs || !locs->jointLocations || locs->jointCount < (uint32_t)kJointCount) return;
    Hand& h = hands_[hand];
    XrHandJointLocationEXT* j = locs->jointLocations;
    const XrPosef camPalm = j[XR_HAND_JOINT_PALM_EXT].pose;
    // When the cameras lose the hand, the runtime keeps returning its last pose flagged as tracked.
    // Normal frames repeat at most ~5 times (camera slower than the 90 Hz app); a longer run is frozen.
    const bool sameAsLast = camPalm.position.x == h.lastCamPos.x && camPalm.position.y == h.lastCamPos.y &&
                            camPalm.position.z == h.lastCamPos.z;
    h.camRepeat = sameAsLast ? h.camRepeat + 1 : 0;
    h.lastCamPos = camPalm.position;
    const bool cameraFrozen = h.camRepeat >= kFrozenFrames;
    if (cameraFrozen) h.cameraFrozenFrames++;
    const bool cameraFresh = h.camRepeat == 0;
    if (cameraFresh && locs->isActive) {
        const XrPosef invCam = pinv(camPalm);
        float maxMove = 0.f;
        XrVector3f local[kJointCount];
        for (int i = 0; i < kJointCount; i++) {
            local[i] = pmul(invCam, j[i].pose).position;
            if (h.havePrevLocal) maxMove = std::fmax(maxMove, length(sub(local[i], h.prevLocal[i])));
            h.prevLocal[i] = local[i];
        }
        const float palmMove = length(sub(camPalm.position, h.prevCamPalm));
        h.prevCamPalm = camPalm.position;
        if (h.havePrevLocal) {
            h.shapeDelta = maxMove;
            if (maxMove < kRigidShapeEps && palmMove > kRigidMinPalmMove) h.rigidRepeat++;
            else if (maxMove >= kRigidShapeEps) h.rigidRepeat = 0;
        }
        h.havePrevLocal = true;
    } else if (!locs->isActive) {
        h.havePrevLocal = false;
        h.rigidRepeat = 0;
    }
    const bool cameraExtrapolated = h.rigidRepeat >= kRigidFrames;
    if (cameraExtrapolated) h.cameraExtrapolatedFrames++;
    const bool cameraReported = locs->isActive && (j[XR_HAND_JOINT_PALM_EXT].locationFlags & kAll) == kAll &&
                                !cameraFrozen && !cameraExtrapolated;

    // ---- tracker
    if (cameraReported) assign(hand, camPalm.position, trackers, trackerCount, cfg);
    const TrackerSample* t = find(h.assigned, trackers, trackerCount);
    const bool trackerOk = t && t->valid;
    if (trackerOk) pushTracker(h, time, t->pose);

    // Where is the hand really? The tracker knows even when the cameras cannot see it. Outside the
    // cameras' field of view, the runtime keeps reporting a stale hand as tracked: ignore it.
    bool outOfView = false;
    if (view) {
        const bool calibrated = trackerOk && h.samples >= cfg.minSamples;
        const XrVector3f where = calibrated ? pmul(t->pose, h.offset).position : camPalm.position;
        outOfView = outsideFov(*view, where, cfg, &h.viewAz, &h.viewEl) && cfg.fovCheck && calibrated;
        if (outOfView) h.outOfViewFrames++;
    }
    const bool cameraTracked = cameraReported && !outOfView;
    if (h.seeded && h.learnedAt == 0) h.learnedAt = time;
    const XrPosef lagged = trackerOk ? trackerAt(h, time - lagNs(cfg)) : pidentity();

    // The tracker sits on the back of the hand: with the palm facing the headset it is hidden from
    // the headset cameras and the runtime only dead-reckons its position (orientation stays good).
    const bool trackerOptical = trackerOk && t->positionTracked && time >= h.untrustedUntil;
    if (trackerOk && !trackerOptical) h.trackerHidden++;

    // A tracker that left the hand is only detected from trustworthy data: optically tracked
    // (a hidden one dead-reckons and drifts) and a fresh camera frame.
    if (cameraTracked && cameraFresh && trackerOptical && !isManual(hand, cfg)) {
        if (length(sub(lagged.position, camPalm.position)) > kUnassignDist) {
            if (++h.farFrames > kUnassignFrames) {
                HF_LOGI("fusion: tracker %lld is no longer on hand %d, unassigned", (long long)h.assigned, hand);
                unassign(h);
                return;
            }
        } else {
            h.farFrames = 0;
        }
    }


    // Learn only from fresh camera frames: a repeated one pairs an old camera pose with a new tracker pose.
    if (cameraTracked && cameraFresh && trackerOptical) learn(h, time, lagged, camPalm, cfg);
    const bool offsetReady = trackerOk && h.samples >= cfg.minSamples;
    if (cfg.cameraLagAuto && offsetReady && !h.relearning && cameraTracked && cameraFresh && trackerOptical)
        measureLag(h, time, camPalm.position);

    // ---- is this camera frame usable? (the runtime keeps reporting "tracked" while it extrapolates)
    bool cameraGood = cameraTracked;
    XrPosef predicted{};
    if (offsetReady) {
        predicted = pmul(t->pose, h.offset);
        if (cameraTracked && trackerOptical) {
            const XrPosef predictedLagged = pmul(lagged, h.offset);
            h.lastErrPos = length(sub(predictedLagged.position, camPalm.position));
            h.lastErrAngle = qangle(predictedLagged.orientation, camPalm.orientation) * kRadToDeg;
            const float threshold = cfg.outlierDist + kResidualLatency * h.speed;
            const bool settled = time - h.learnedAt > kOutlierGraceNs;
            if (cfg.outlierDist > 0.f && settled && h.lastErrPos > threshold) {
                cameraGood = false;
                h.outliers++;
            }
        }
    }

    // ---- finger shape in the palm frame
    if (cameraGood) {
        const XrPosef invPalm = pinv(camPalm);
        if (!h.wasGood) {
            // Blend from what was shown (last pose seen, possibly relaxed) to the cameras.
            h.reacquireStart = time;
            for (int i = 0; i < kJointCount; i++) h.frozen[i] = h.shape[i];
        }
        const float span = (float)cfg.reacquireMs * 1e6f;
        float b = (h.haveShape && span > 0.f) ? (float)(time - h.reacquireStart) / span : 1.f;
        if (b < 0.f) b = 0.f;
        for (int i = 0; i < kJointCount; i++) {
            const XrPosef s = pmul(invPalm, j[i].pose);
            h.shape[i] = b >= 1.f ? s : pblend(h.frozen[i], s, b);
            h.radius[i] = j[i].radius;
            if (cameraFresh) h.meanShape[i] = h.haveMean ? pblend(h.meanShape[i], s, kMeanShapeRate) : s;
        }
        if (cameraFresh) h.haveMean = true;
        h.haveShape = true;
        h.lastGood = time;
    } else if (h.haveShape) {
        if (h.wasGood)
            for (int i = 0; i < kJointCount; i++) h.lostShape[i] = h.shape[i];
        // Out of view: rather than keeping the fingers frozen in the last pose seen (often mid-gesture),
        // relax them toward this hand's average pose.
        const int64_t since = time - h.lastGood - cfg.fingerRelaxDelayMs * 1000000LL;
        if (h.haveMean && cfg.fingerRelaxMs > 0 && since > 0) {
            float b = (float)since / ((float)cfg.fingerRelaxMs * 1e6f);
            if (b > 1.f) b = 1.f;
            b = b * b * (3.f - 2.f * b);  // smoothstep: no sudden start or stop
            for (int i = 0; i < kJointCount; i++) h.shape[i] = pblend(h.lostShape[i], h.meanShape[i], b);
        }
    }
    h.wasGood = cameraGood;

    // ---- palm pose: orientation from the tracker IMU, position from the best source available
    if (h.lastOutTime == 0 || time - h.lastOutTime > kFilterResetNs || time < h.lastOutTime) resetFilters(h);
    float dt = h.lastOutTime ? (float)(time - h.lastOutTime) * 1e-9f : kDefaultDt;
    if (dt <= 0.f) dt = kDefaultDt;

    XrPosef palmRaw;
    int source;
    const bool holding = h.haveShape &&
                         (cfg.holdTimeoutMs <= 0 || (time - h.lastGood) <= cfg.holdTimeoutMs * 1000000LL);
    if (offsetReady && (cameraGood || holding)) {
        palmRaw.orientation =
            cameraGood ? qslerp(camPalm.orientation, predicted.orientation, cfg.rotationWeight) : predicted.orientation;
        if (trackerOptical) {
            source = 0;
            palmRaw.position = cameraGood ? lerp(camPalm.position, predicted.position, cfg.trackedWeight)
                                          : predicted.position;
            h.mode = cameraGood ? HandMode::Fused : HandMode::TrackerOnly;
            if (cameraGood) h.framesFused++;
            else h.framesTrackerOnly++;
        } else if (cameraGood) {
            // Tracker hidden: its IMU keeps the orientation. For the position, a complementary filter:
            // the tracker's dead-reckoned position is smooth but drifts, the camera is absolute but
            // noisy. Follow the dead reckoning and pull it slowly toward the camera.
            source = 1;
            if (h.lastSource != 1) {
                // Start from the last output so the hand-over is seamless.
                h.drPos = h.lastSource >= 0 ? h.lastRawPos : camPalm.position;
            } else {
                XrVector3f delta = sub(predicted.position, h.drPrevPred);
                if (length(delta) > kJumpDist) delta = v3(0.f, 0.f, 0.f);  // runtime re-snap, not motion
                h.drPos = add(h.drPos, delta);
                const XrVector3f err = sub(camPalm.position, h.drPos);
                const float tau = length(err) > kDeadReckonFarErr ? kDeadReckonFastTau : kDeadReckonTau;
                h.drPos = add(h.drPos, scale(err, std::fmin(1.f, dt / tau)));
            }
            h.drPrevPred = predicted.position;
            palmRaw.position = h.drPos;
            h.mode = HandMode::ImuCamera;
            h.framesImu++;
        } else {
            // Neither sees the hand: short-term dead reckoning by the runtime.
            source = 2;
            palmRaw.position = predicted.position;
            h.mode = HandMode::TrackerOnly;
            h.framesTrackerOnly++;
        }
    } else if (cameraGood) {
        source = 1;
        palmRaw = camPalm;
        h.mode = trackerOk ? HandMode::Learning : HandMode::Camera;
    } else {
        h.mode = h.assigned >= 0 ? HandMode::Lost : HandMode::Idle;
        resetFilters(h);
        h.lastSource = -1;
        return;  // nothing reliable: leave the runtime output untouched
    }

    // Hand the position over smoothly when the source changes: keep the previous output position
    // and let the difference decay, instead of jumping by the camera/tracker disagreement.
    if (h.lastSource >= 0 && source != h.lastSource) h.posBias = sub(h.lastRawPos, palmRaw.position);
    else h.posBias = scale(h.posBias, std::exp(-dt / kSourceBlendTau));
    h.lastSource = source;
    palmRaw.position = add(palmRaw.position, h.posBias);
    h.lastRawPos = palmRaw.position;

    // ---- filters
    h.lastOutTime = time;

    const OneEuroParams palmPos{cfg.wristMinCutoff, cfg.wristBeta, 1.f};
    const OneEuroParams palmRot{cfg.wristMinCutoff, cfg.wristBeta, 1.f};
    // Finger joints move a few cm at most: scale beta so the same value works for m/s and rad/s.
    const OneEuroParams fingerPos{cfg.fingerMinCutoff, cfg.fingerBeta * 10.f, 1.f};
    const OneEuroParams fingerRot{cfg.fingerMinCutoff, cfg.fingerBeta, 1.f};
    const XrPosef palmOut = h.palmFilter.filter(palmRaw, dt, palmPos, palmRot);

    // ---- write the hand back
    const XrPosef correction = pmul(palmOut, pinv(camPalm));
    for (int i = 0; i < kJointCount; i++) {
        XrPosef local = h.shape[i];
        if (i != XR_HAND_JOINT_PALM_EXT) local = h.jointFilter[i].filter(local, dt, fingerPos, fingerRot);
        j[i].pose = pmul(palmOut, local);
        j[i].radius = h.radius[i];
        j[i].locationFlags = kAll;
    }
    locs->isActive = XR_TRUE;

    if (XrHandJointVelocitiesEXT* v = findVelocities(locs)) {
        for (uint32_t i = 0; i < v->jointCount && v->jointVelocities; i++) {
            if (cameraGood) {
                v->jointVelocities[i].linearVelocity = qrotate(correction.orientation, v->jointVelocities[i].linearVelocity);
                v->jointVelocities[i].angularVelocity =
                    qrotate(correction.orientation, v->jointVelocities[i].angularVelocity);
            } else {
                v->jointVelocities[i].velocityFlags = 0;
            }
        }
    }
}

std::string Fusion::status() const {
    std::string s;
    char buf[700];
    for (int i = 0; i < 2; i++) {
        const Hand& h = hands_[i];
        const char* n = i ? "right" : "left";
        snprintf(buf, sizeof(buf),
                 "%s.mode=%s\n%s.tracker=%lld\n%s.samples=%d\n%s.err_mm=%.0f\n%s.err_deg=%.1f\n%s.err_avg_mm=%.0f\n"
                 "%s.speed_cms=%.0f\n%s.frames_fused=%llu\n%s.frames_tracker_only=%llu\n%s.camera_outliers=%llu\n"
                 "%s.relearns=%d\n%s.frames_imu_camera=%llu\n%s.tracker_hidden=%llu\n%s.tracker_jumps=%llu\n"
                 "%s.camera_frozen=%llu\n%s.camera_extrapolated=%llu\n%s.out_of_view=%llu\n",
                 n, modeName(h.mode), n, (long long)h.assigned, n, h.samples, n, h.lastErrPos * 1000.f, n,
                 h.lastErrAngle, n, h.errAvg * 1000.f, n, h.speed * 100.f, n, (unsigned long long)h.framesFused, n,
                 (unsigned long long)h.framesTrackerOnly, n, (unsigned long long)h.outliers, n, h.relearns, n,
                 (unsigned long long)h.framesImu, n, (unsigned long long)h.trackerHidden, n,
                 (unsigned long long)h.jumps, n, (unsigned long long)h.cameraFrozenFrames, n,
                 (unsigned long long)h.cameraExtrapolatedFrames, n, (unsigned long long)h.outOfViewFrames);
        s += buf;
    }
    snprintf(buf, sizeof(buf), "camera_lag_ms=%.0f\ncamera_lag_windows=%d\ncamera_lag_residual_mm=%.0f\n",
             lagEstMs_, lagUpdates_, lagRmsMm_);
    s += buf;
    return s;
}

}  // namespace hf
