#include "recorder.h"

#include <cstdio>

#include "log.h"

namespace hf {

namespace {
void putPose(float* dst, const XrPosef& p) {
    dst[0] = p.position.x;
    dst[1] = p.position.y;
    dst[2] = p.position.z;
    dst[3] = p.orientation.x;
    dst[4] = p.orientation.y;
    dst[5] = p.orientation.z;
    dst[6] = p.orientation.w;
}
}  // namespace

void Recorder::arm(int64_t epoch, int seconds, int64_t nowNs) {
    // The value present when the session starts is only a baseline: a change triggers a recording.
    if (!baseline_) {
        baseline_ = true;
        epoch_ = epoch;
        return;
    }
    if (epoch == epoch_) return;
    epoch_ = epoch;
    if (seconds <= 0) return;
    rows_.clear();
    rows_.reserve((size_t)seconds * 2 * 120);
    endNs_ = nowNs + (int64_t)seconds * 1000000000LL;
    recording_ = true;
    HF_LOGI("record: started for %d s", seconds);
}

void Recorder::captureInput(int hand, XrTime time, const XrHandJointLocationsEXT* locs, bool haveTracker,
                            uint64_t trackerId, bool trackerValid, const XrPosef& tracker, uint32_t trackerFlags) {
    if (!recording_ || !locs || !locs->jointLocations || locs->jointCount < XR_HAND_JOINT_COUNT_EXT) return;
    Row r{};
    r.time = time;
    r.hand = (int8_t)hand;
    r.active = locs->isActive ? 1 : 0;
    r.wristFlags = (uint32_t)locs->jointLocations[XR_HAND_JOINT_WRIST_EXT].locationFlags;
    putPose(r.cam, locs->jointLocations[XR_HAND_JOINT_WRIST_EXT].pose);
    const XrVector3f& palm = locs->jointLocations[XR_HAND_JOINT_PALM_EXT].pose.position;
    r.palm[0] = palm.x;
    r.palm[1] = palm.y;
    r.palm[2] = palm.z;
    r.trackerId = haveTracker ? (int64_t)trackerId : -1;
    r.trackerValid = trackerValid ? 1 : 0;
    r.trackerFlags = trackerFlags;
    putPose(r.trk, tracker);
    r.mode = -1;
    rows_.push_back(r);
}

void Recorder::captureOutput(int hand, const XrHandJointLocationsEXT* locs, int mode) {
    if (!recording_ || rows_.empty() || rows_.back().hand != hand || !locs || !locs->jointLocations) return;
    Row& r = rows_.back();
    r.mode = (int8_t)mode;
    putPose(r.out, locs->jointLocations[XR_HAND_JOINT_WRIST_EXT].pose);
    const XrVector3f& tip = locs->jointLocations[XR_HAND_JOINT_INDEX_TIP_EXT].pose.position;
    r.tip[0] = tip.x;
    r.tip[1] = tip.y;
    r.tip[2] = tip.z;
}

bool Recorder::finishIfDue(int64_t nowNs, const std::string& dir) {
    if (!recording_ || nowNs < endNs_) return false;
    recording_ = false;
    const std::string path = dir + "/hand_fusion_record.csv";
    FILE* f = fopen(path.c_str(), "w");
    if (!f) {
        HF_LOGE("record: cannot write %s", path.c_str());
        return false;
    }
    fputs("time_ns,hand,active,wrist_flags,mode,tracker_id,tracker_valid,tracker_flags,"
          "cam_px,cam_py,cam_pz,cam_qx,cam_qy,cam_qz,cam_qw,palm_x,palm_y,palm_z,"
          "trk_px,trk_py,trk_pz,trk_qx,trk_qy,trk_qz,trk_qw,"
          "out_px,out_py,out_pz,out_qx,out_qy,out_qz,out_qw,tip_x,tip_y,tip_z\n",
          f);
    for (const Row& r : rows_) {
        fprintf(f, "%lld,%d,%d,%u,%d,%lld,%d,%u", (long long)r.time, r.hand, r.active, r.wristFlags, r.mode,
                (long long)r.trackerId, r.trackerValid, r.trackerFlags);
        for (float v : r.cam) fprintf(f, ",%.5f", v);
        for (float v : r.palm) fprintf(f, ",%.5f", v);
        for (float v : r.trk) fprintf(f, ",%.5f", v);
        for (float v : r.out) fprintf(f, ",%.5f", v);
        for (float v : r.tip) fprintf(f, ",%.5f", v);
        fputc('\n', f);
    }
    fclose(f);
    HF_LOGI("record: wrote %zu rows to %s", rows_.size(), path.c_str());
    rows_.clear();
    rows_.shrink_to_fit();
    return true;
}

}  // namespace hf
