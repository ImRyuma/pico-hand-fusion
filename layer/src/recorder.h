// Records raw camera / tracker / output poses for offline analysis (tuning sessions).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <openxr/openxr.h>

namespace hf {

class Recorder {
public:
    // Starts a recording of `seconds` if `epoch` differs from the last one seen.
    void arm(int64_t epoch, int seconds, int64_t nowNs);
    bool active() const { return recording_; }

    // Raw camera data, before fusion.
    void captureInput(int hand, XrTime time, const XrHandJointLocationsEXT* locs, bool haveTracker, uint64_t trackerId,
                      bool trackerValid, const XrPosef& tracker, uint32_t trackerFlags);
    // Output after fusion, for the same hand and frame.
    void captureOutput(int hand, const XrHandJointLocationsEXT* locs, int mode);

    // Writes the CSV when the time is up. Returns true when a file was written.
    bool finishIfDue(int64_t nowNs, const std::string& dir);

private:
    struct Row {
        XrTime time;
        int8_t hand;
        int8_t active;
        int8_t trackerValid;
        int8_t mode;
        uint32_t wristFlags;
        uint32_t trackerFlags;
        int64_t trackerId;
        float cam[7];   // wrist px py pz qx qy qz qw
        float palm[3];
        float trk[7];
        float out[7];
        float tip[3];   // index tip, output
    };

    bool recording_ = false;
    bool baseline_ = false;
    int64_t epoch_ = 0;
    int64_t endNs_ = 0;
    std::vector<Row> rows_;
};

}  // namespace hf
