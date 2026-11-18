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
    // `view` is the headset pose in the same space as the hand (nullptr if unknown).
    void process(int hand, XrTime time, const TrackerSample* trackers, int trackerCount,
                 XrHandJointLocationsEXT* locs, const Config& cfg, const XrPosef* view = nullptr);
    std::string status() const;
    int64_t assigned(int hand) const { return hands_[hand].assigned; }
    // True while at least one hand has a tracker (used to hold full frequency).
    bool usingTrackers() const;
    int mode(int hand) const { return (int)hands_[hand].mode; }
    // Largest finger joint move in the palm frame since the previous fresh camera frame (m), for the recorder.
    float shapeDelta(int hand) const { return hands_[hand].shapeDelta; }
    // Direction of the hand seen from the headset (deg, right and up positive), for the recorder.
    float viewAzimuth(int hand) const { return hands_[hand].viewAz; }
    float viewElevation(int hand) const { return hands_[hand].viewEl; }
    // Offset persistence across sessions (palm pose in the tracker frame, per hand).
    void seedOffset(int hand, const XrPosef& offset);
    bool learnedOffset(int hand, XrPosef* out) const;
    // Measured camera latency (ms, < 0 = not measured yet), persisted with the offsets.
    float measuredLagMs() const { return lagEstMs_; }
    void seedCameraLag(float ms);
    void resetCameraLag() { lagEstMs_ = -1.f; lagUpdates_ = 0; lagRmsMm_ = 0.f; }

private:
    static constexpr int kHistory = 192;  // tracker poses kept for camera lag compensation and measurement (~2 s)
    static constexpr int kLagWindow = 40; // fresh camera frames per camera latency measurement

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
        XrPosef frozen[kJointCount]{};     // output shape when the cameras found the hand again
        XrPosef lostShape[kJointCount]{};  // shape when the cameras lost the hand
        bool haveMean = false;
        XrPosef meanShape[kJointCount]{};  // slow average of the shapes seen: relaxed pose out of view
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
        // Camera latency measurement: fresh camera palm positions during moves.
        XrTime lagTime[kLagWindow]{};
        XrVector3f lagCam[kLagWindow]{};
        int lagCount = 0;
        // Frozen camera detection.
        XrVector3f lastCamPos{};
        int camRepeat = 0;
        // Extrapolated camera detection: the runtime moves the last skeleton seen as one rigid block.
        XrVector3f prevLocal[kJointCount]{};
        bool havePrevLocal = false;
        XrVector3f prevCamPalm{};
        int rigidRepeat = 0;
        float shapeDelta = 0.f;
        uint64_t cameraExtrapolatedFrames = 0;
        // Field of view check.
        float viewAz = 0.f, viewEl = 0.f;
        uint64_t outOfViewFrames = 0;
    };

    void assign(int hand, const XrVector3f& palm, const TrackerSample* trackers, int n, const Config& cfg);
    const TrackerSample* find(int64_t id, const TrackerSample* trackers, int n) const;
    void pushTracker(Hand& h, XrTime time, const XrPosef& pose);
    XrPosef trackerAt(const Hand& h, XrTime time) const;
    void learn(Hand& h, XrTime time, const XrPosef& laggedTracker, const XrPosef& palm, const Config& cfg);
    void resetFilters(Hand& h);
    void unassign(Hand& h);
    void applySeed(Hand& h, int hand, const Config& cfg);
    void measureLag(Hand& h, XrTime time, const XrVector3f& camPalm);
    int64_t lagNs(const Config& cfg) const;

    Hand hands_[2];
    float lagEstMs_ = -1.f;
    int lagUpdates_ = 0;
    float lagRmsMm_ = 0.f;
};

}  // namespace hf
