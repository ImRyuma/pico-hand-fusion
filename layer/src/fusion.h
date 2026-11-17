// Hand fusion between camera hand tracking and a motion tracker worn on the back of each hand.
//
// The tracker is rigid with the palm, so the palm joint is the reference: the layer learns the
// palm pose in the tracker frame, then drives the whole hand from the tracker and keeps only the
// finger shape (relative to the palm) from the cameras.
#pragma once

#include <cstdint>
#include <string>

#include <openxr/openxr.h>

#include "config.h"
#include "filter.h"

namespace hf {

struct TrackerSample {
    uint64_t id = 0;
    bool valid = false;
    bool positionTracked = true;  // false while the runtime only dead-reckons it (hidden from the cameras)
    uint64_t flags = 0;
    XrPosef pose{};
};

enum class HandMode { Idle, Camera, Learning, Fused, TrackerOnly, Lost, ImuCamera };

const char* modeName(HandMode m);

class Fusion {
public:
    static constexpr int kMaxTrackers = 8;
    static constexpr int kJointCount = XR_HAND_JOINT_COUNT_EXT;

    void reset();
    // Called after the runtime has filled `locs` for `hand` (0 = left, 1 = right).
    void process(int hand, XrTime time, const TrackerSample* trackers, int trackerCount,
                 XrHandJointLocationsEXT* locs, const Config& cfg);
    std::string status() const;
    int64_t assigned(int hand) const { return hands_[hand].assigned; }
    // True while at least one hand has a tracker (used to hold full frequency).
    bool usingTrackers() const;
    int mode(int hand) const { return (int)hands_[hand].mode; }
    // Offset persistence across sessions (palm pose in the tracker frame, per hand).
    void seedOffset(int hand, const XrPosef& offset);
    bool learnedOffset(int hand, XrPosef* out) const;

private:
    static constexpr int kHistory = 64;  // tracker poses kept for camera lag compensation (~0.7 s)

    struct Hand {
        // Tracker assignment.
        int64_t assigned = -1;
        int64_t candidate = -1;
        int candidateFrames = 0;
        int farFrames = 0;
        // Tracker history and motion.
        XrTime histTime[kHistory]{};
        XrPosef histPose[kHistory]{};
        int histHead = 0;
        int histCount = 0;
        XrTime untrustedUntil = 0;  // tracker position not trusted before this (jump detected)
        float speed = 0.f;     // m/s, smoothed
        float angSpeed = 0.f;  // rad/s, smoothed
        // Palm pose in the tracker frame.
        XrPosef offset{};
        int samples = 0;
        XrTime learnedAt = 0;
        float errAvg = 0.f;  // smoothed camera vs tracker disagreement (lag compensated), m
        int disagreeFrames = 0;
        int relearns = 0;
        bool relearning = false;  // a replacement offset is being learned in the background
        XrPosef candOffset{};
        int candSamples = 0;
        bool haveSeed = false;  // remembered offset (earlier assignment or session)
        bool seeded = false;    // current offset comes from the seed
        XrPosef seed{};
        // Finger shape: joints in the palm frame.
        bool haveShape = false;
        XrPosef shape[kJointCount]{};
        XrPosef frozen[kJointCount]{};
        float radius[kJointCount]{};
        XrTime lastGood = 0;
        XrTime reacquireStart = 0;
        bool wasGood = false;
        // Position source switching (0 = tracker, 1 = camera, 2 = tracker dead reckoning).
        int lastSource = -1;
        XrVector3f posBias{};    // decays to zero after a source switch, so the hand never jumps
        XrVector3f lastRawPos{};
        XrVector3f drPos{};      // complementary filter state while the tracker is hidden
        XrVector3f drPrevPred{};
        // Output filters.
        XrTime lastOutTime = 0;
        OneEuroPose palmFilter;
        OneEuroPose jointFilter[kJointCount];
        // Diagnostics.
        HandMode mode = HandMode::Idle;
        float lastErrPos = 0.f;
        float lastErrAngle = 0.f;
        uint64_t framesFused = 0;
        uint64_t framesTrackerOnly = 0;
        uint64_t outliers = 0;
        uint64_t framesImu = 0;     // tracker hidden: IMU orientation + camera position
        uint64_t trackerHidden = 0; // frames without optical tracking of the tracker
        uint64_t jumps = 0;
        uint64_t cameraFrozenFrames = 0;
        // Frozen camera detection.
        XrVector3f lastCamPos{};
        int camRepeat = 0;
    };

    void assign(int hand, const XrVector3f& palm, const TrackerSample* trackers, int n, const Config& cfg);
    const TrackerSample* find(int64_t id, const TrackerSample* trackers, int n) const;
    void pushTracker(Hand& h, XrTime time, const XrPosef& pose);
    XrPosef trackerAt(const Hand& h, XrTime time) const;
    void learn(Hand& h, XrTime time, const XrPosef& laggedTracker, const XrPosef& palm, const Config& cfg);
    void resetFilters(Hand& h);
    void unassign(Hand& h);
    void applySeed(Hand& h, int hand, const Config& cfg);

    Hand hands_[2];
};

}  // namespace hf
