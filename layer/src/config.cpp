#include "config.h"

#include <sys/stat.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "log.h"

namespace hf {

static constexpr int64_t kCheckIntervalNs = 1000000000LL;

void ConfigStore::init(const std::string& dir) {
    dir_ = dir;
    path_ = dir + "/hand_fusion.conf";
    lastMtime_ = -1;
    lastCheckNs_ = 0;
}

static char* trim(char* s) {
    while (*s == ' ' || *s == '\t') s++;
    char* e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n')) *--e = 0;
    return s;
}

static void apply(Config& c, const char* k, const char* v) {
    if (!strcmp(k, "enabled")) c.enabled = atoi(v) != 0;
    else if (!strcmp(k, "tracked_weight")) c.trackedWeight = (float)atof(v);
    else if (!strcmp(k, "rotation_weight")) c.rotationWeight = (float)atof(v);
    else if (!strcmp(k, "outlier_dist")) c.outlierDist = (float)atof(v);
    else if (!strcmp(k, "reacquire_ms")) c.reacquireMs = atoll(v);
    else if (!strcmp(k, "camera_lag_ms")) c.cameraLagMs = atoll(v);
    else if (!strcmp(k, "camera_lag_auto")) c.cameraLagAuto = atoi(v) != 0;
    else if (!strcmp(k, "finger_relax_delay_ms")) c.fingerRelaxDelayMs = atoll(v);
    else if (!strcmp(k, "finger_relax_ms")) c.fingerRelaxMs = atoll(v);
    else if (!strcmp(k, "wrist_min_cutoff")) c.wristMinCutoff = (float)atof(v);
    else if (!strcmp(k, "wrist_beta")) c.wristBeta = (float)atof(v);
    else if (!strcmp(k, "finger_min_cutoff")) c.fingerMinCutoff = (float)atof(v);
    else if (!strcmp(k, "finger_beta")) c.fingerBeta = (float)atof(v);
    else if (!strcmp(k, "full_frequency")) { /* read by the module's root daemon */ }
    else if (!strcmp(k, "hold_timeout_ms")) c.holdTimeoutMs = atoll(v);
    else if (!strcmp(k, "learn_rate")) c.learnRate = (float)atof(v);
    else if (!strcmp(k, "min_samples")) c.minSamples = atoi(v);
    else if (!strcmp(k, "assign_max_dist")) c.assignMaxDist = (float)atof(v);
    else if (!strcmp(k, "fov_check")) c.fovCheck = atoi(v) != 0;
    else if (!strcmp(k, "fov_side_deg")) c.fovSideDeg = (float)atof(v);
    else if (!strcmp(k, "fov_up_deg")) c.fovUpDeg = (float)atof(v);
    else if (!strcmp(k, "fov_down_deg")) c.fovDownDeg = (float)atof(v);
    else if (!strcmp(k, "left_tracker")) c.leftTracker = atoll(v);
    else if (!strcmp(k, "right_tracker")) c.rightTracker = atoll(v);
    else if (!strcmp(k, "request_devices")) c.requestDevices = atoi(v);
    else if (!strcmp(k, "calib_epoch")) c.calibEpoch = atoll(v);
    else if (!strcmp(k, "record_epoch")) c.recordEpoch = atoll(v);
    else if (!strcmp(k, "record_seconds")) c.recordSeconds = atoi(v);
    else if (!strcmp(k, "log_level")) c.logLevel = atoi(v);
    else HF_LOGI("config: unknown key '%s'", k);
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

bool ConfigStore::poll(int64_t nowNs) {
    if (nowNs - lastCheckNs_ < kCheckIntervalNs && lastMtime_ != -1) return false;
    lastCheckNs_ = nowNs;

    struct stat st;
    if (stat(path_.c_str(), &st) != 0) {
        if (lastMtime_ != 0) {
            // No file: defaults.
            cfg_ = Config();
            lastMtime_ = 0;
            lastSize_ = -1;
            lastInode_ = 0;
            gLogLevel = cfg_.logLevel;
            HF_LOGI("config: %s not found, using defaults", path_.c_str());
            return true;
        }
        return false;
    }
    // The mtime alone is not enough: a file replaced within the same clock tick keeps the same
    // mtime, so the inode (new file after a rename) and the size are compared too.
    int64_t mtime = (int64_t)st.st_mtime * 1000000000LL + st.st_mtim.tv_nsec;
    if (mtime == lastMtime_ && (int64_t)st.st_size == lastSize_ && (uint64_t)st.st_ino == lastInode_) return false;
    lastMtime_ = mtime;
    lastSize_ = (int64_t)st.st_size;
    lastInode_ = (uint64_t)st.st_ino;

    FILE* f = fopen(path_.c_str(), "r");
    if (!f) {
        HF_LOGE("config: cannot open %s", path_.c_str());
        return false;
    }
    Config c;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char* s = trim(line);
        if (!*s || *s == '#') continue;
        char* eq = strchr(s, '=');
        if (!eq) continue;
        *eq = 0;
        apply(c, trim(s), trim(eq + 1));
    }
    fclose(f);

    c.trackedWeight = clampf(c.trackedWeight, 0.f, 1.f);
    c.rotationWeight = clampf(c.rotationWeight, 0.f, 1.f);
    if (c.reacquireMs < 0) c.reacquireMs = 0;
    if (c.cameraLagMs < 0) c.cameraLagMs = 0;
    if (c.cameraLagMs > 150) c.cameraLagMs = 150;
    if (c.fingerRelaxDelayMs < 0) c.fingerRelaxDelayMs = 0;
    if (c.fingerRelaxMs < 0) c.fingerRelaxMs = 0;
    c.learnRate = clampf(c.learnRate, 0.f, 1.f);
    if (c.minSamples < 1) c.minSamples = 1;
    cfg_ = c;
    gLogLevel = c.logLevel;
    HF_LOGI("config: loaded (enabled=%d pos_w=%.2f rot_w=%.2f outlier=%.3f wrist=%.1f/%.2f finger=%.1f/%.2f "
            "hold=%lldms lag=%lldms%s relax=%lld+%lldms epoch=%lld)",
            c.enabled, c.trackedWeight, c.rotationWeight, c.outlierDist, c.wristMinCutoff, c.wristBeta,
            c.fingerMinCutoff, c.fingerBeta, (long long)c.holdTimeoutMs, (long long)c.cameraLagMs,
            c.cameraLagAuto ? " auto" : "", (long long)c.fingerRelaxDelayMs, (long long)c.fingerRelaxMs,
            (long long)c.calibEpoch);
    return true;
}

}  // namespace hf
