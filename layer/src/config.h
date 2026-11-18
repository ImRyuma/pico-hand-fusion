// Runtime settings, read from a key=value file in the host app's data dir.
#pragma once

#include <cstdint>
#include <string>

namespace hf {

struct Config {
    bool enabled = true;
    // Weight of the tracker-predicted wrist when the cameras also see the hand (0..1).
    float trackedWeight = 1.0f;   // position
    float rotationWeight = 1.0f;  // orientation (the tracker IMU is far steadier than the cameras)
    // Camera wrist farther than this from the tracker prediction is treated as a glitch (m, 0 = off).
    float outlierDist = 0.08f;
    // How long a hand lost by the cameras keeps being driven by its tracker (ms, 0 = forever).
    int64_t holdTimeoutMs = 0;
    // Fingers blend back from the frozen pose to the camera pose over this time (ms).
    int64_t reacquireMs = 150;
    // Hand out of the cameras' view: after this delay, the fingers relax from the last pose seen toward the
    // average pose of this hand over this time (ms, finger_relax_ms 0 = keep the last pose).
    int64_t fingerRelaxDelayMs = 300;
    int64_t fingerRelaxMs = 1500;
    // Slow learning rate of the tracker -> palm offset once it is established.
    float learnRate = 0.01f;
    // Camera hand latency relative to the tracker; learning compares the camera with the tracker this long ago.
    // With camera_lag_auto, it is measured during fast moves and this value is only the starting point.
    int64_t cameraLagMs = 45;
    bool cameraLagAuto = true;
    // Samples needed before the offset is trusted.
    int minSamples = 120;
    // Max distance between a tracker and the palm for automatic assignment (m).
    float assignMaxDist = 0.12f;
    // Hand tracking cameras' field of view around the headset's forward axis (deg). Camera data for a
    // hand whose tracker puts it outside is ignored: the runtime keeps sending a stale hand there.
    bool fovCheck = true;
    float fovSideDeg = 65.f;
    float fovUpDeg = 45.f;
    float fovDownDeg = 65.f;
    // Manual tracker ids (-1 = automatic assignment).
    int64_t leftTracker = -1;
    int64_t rightTracker = -1;
    // Motion trackers requested from the runtime once per session (their ids come back in an event).
    int requestDevices = 2;
    // One Euro filters (min cutoff in Hz, <= 0 disables; beta per m/s or rad/s).
    float wristMinCutoff = 4.0f;
    float wristBeta = 1.0f;
    float fingerMinCutoff = 1.5f;
    float fingerBeta = 0.4f;
    // Bumping this value resets every learned offset and assignment.
    int64_t calibEpoch = 0;
    // Changing record_epoch records record_seconds of raw data to hand_fusion_record.csv.
    int64_t recordEpoch = 0;
    int recordSeconds = 60;
    // 0 = errors, 1 = info, 2 = debug.
    int logLevel = 1;
};

class ConfigStore {
public:
    void init(const std::string& dir);
    // Reloads the file if it changed; cheap to call often.
    bool poll(int64_t nowNs);
    const Config& get() const { return cfg_; }
    const std::string& dir() const { return dir_; }

private:
    std::string dir_;
    std::string path_;
    Config cfg_;
    int64_t lastCheckNs_ = 0;
    int64_t lastMtime_ = -1;
    int64_t lastSize_ = -1;
    uint64_t lastInode_ = 0;
};

}  // namespace hf
