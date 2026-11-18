// Synthetic check of the fusion, run on the headset:
//   a hand held still with camera jitter, a steady tracker on the back of the hand,
//   then the cameras lose the hand. Prints the output jitter and the hold behavior.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "../src/config.h"
#include "../src/fusion.h"
#include "../src/xr_math.h"

namespace hf {
int gLogLevel = 0;
}

using namespace hf;

static float noise(float amp) { return amp * (2.f * (float)rand() / (float)RAND_MAX - 1.f); }

int main() {
    srand(42);
    Config cfg;
    Fusion fusion;
    XrHandJointLocationEXT joints[XR_HAND_JOINT_COUNT_EXT];
    XrHandJointLocationsEXT locs{XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
    locs.jointCount = XR_HAND_JOINT_COUNT_EXT;
    locs.jointLocations = joints;

    // True wrist, still; tracker 6 cm above the back of the hand, rotated 90 deg about X.
    XrPosef wrist = pidentity();
    wrist.position = v3(0.2f, 1.1f, -0.4f);
    XrPosef trackerToWrist = pidentity();
    trackerToWrist.position = v3(0.f, -0.02f, 0.06f);
    trackerToWrist.orientation = qnormalize({0.7071f, 0.f, 0.f, 0.7071f});
    const XrPosef trackerPose = pmul(wrist, pinv(trackerToWrist));

    const float jitter = 0.004f;  // +-4 mm camera jitter
    const XrTime dt = 11111111;   // 90 Hz
    XrTime t = 1000000000LL;
    double sumIn = 0, sumOut = 0, sumFinger = 0;
    int n = 0;
    int trackerOnlyOk = 0, trackerOnlyFrames = 0;

    for (int frame = 0; frame < 900; frame++, t += dt) {
        const bool cameraSees = frame < 700;
        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) {
            XrPosef p = wrist;
            p.position = add(wrist.position, v3(0.f, 0.f, -0.01f * (float)i));  // a straight "hand"
            p.position = add(p.position, v3(noise(jitter), noise(jitter), noise(jitter)));
            joints[i].pose = p;
            joints[i].radius = 0.01f;
            joints[i].locationFlags = cameraSees ? 0xF : 0;
        }
        locs.isActive = cameraSees ? XR_TRUE : XR_FALSE;
        TrackerSample tr;
        tr.id = 1;
        tr.valid = true;
        tr.pose = trackerPose;

        const XrVector3f inWrist = joints[XR_HAND_JOINT_WRIST_EXT].pose.position;
        fusion.process(1, t, &tr, 1, &locs, cfg);
        const XrVector3f outWrist = joints[XR_HAND_JOINT_WRIST_EXT].pose.position;
        const XrVector3f outTip = joints[XR_HAND_JOINT_INDEX_TIP_EXT].pose.position;
        const XrVector3f trueTip = add(wrist.position, v3(0.f, 0.f, -0.01f * XR_HAND_JOINT_INDEX_TIP_EXT));
        const XrVector3f trueWrist = add(wrist.position, v3(0.f, 0.f, -0.01f * XR_HAND_JOINT_WRIST_EXT));

        if (frame >= 300 && cameraSees) {
            sumIn += length(sub(inWrist, trueWrist));
            sumOut += length(sub(outWrist, trueWrist));
            sumFinger += length(sub(outTip, trueTip));
            n++;
        }
        if (!cameraSees) {
            trackerOnlyFrames++;
            if (locs.isActive && length(sub(outWrist, trueWrist)) < 0.01f) trackerOnlyOk++;
        }
    }

    printf("%s", fusion.status().c_str());
    printf("camera wrist error (mean): %.2f mm\n", 1000.0 * sumIn / n);
    printf("fused  wrist error (mean): %.2f mm\n", 1000.0 * sumOut / n);
    printf("fused  index tip error   : %.2f mm\n", 1000.0 * sumFinger / n);
    printf("hand kept by tracker when out of view: %d / %d frames\n", trackerOnlyOk, trackerOnlyFrames);
    const bool ok1 = sumOut < sumIn * 0.5 && trackerOnlyOk == trackerOnlyFrames;
    printf("static scenario: %s\n", ok1 ? "PASS" : "FAIL");

    // Scenario 2: the hand alternates 2 s still / 2 s moving (40 cm circles, ~1.2 m/s) and the
    // camera lags the tracker by 60 ms. Expect no relearn loop and few camera rejections.
    Fusion f2;
    const int lag = 5;  // frames (~55 ms at 90 Hz)
    XrPosef history[2000];
    t = 1000000000LL;
    int fusedFrames = 0;
    for (int frame = 0; frame < 2000; frame++, t += dt) {
        const float sec = frame / 90.f;
        const bool moving = ((int)(sec / 2.f)) % 2 == 1;
        XrPosef w = wrist;
        if (moving) {
            const float a = sec * 3.f;
            w.position = add(wrist.position, v3(0.2f * std::cos(a), 0.2f * std::sin(a), 0.f));
            w.orientation = qnormalize({0.f, std::sin(a * 0.25f), 0.f, std::cos(a * 0.25f)});
        }
        history[frame] = w;
        const XrPosef cam = history[frame >= lag ? frame - lag : 0];  // delayed camera
        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) {
            joints[i].pose = pmul(cam, XrPosef{qidentity(), v3(0.f, 0.f, -0.01f * (float)i)});
            joints[i].pose.position = add(joints[i].pose.position, v3(noise(jitter), noise(jitter), noise(jitter)));
            joints[i].radius = 0.01f;
            joints[i].locationFlags = 0xF;
        }
        locs.isActive = XR_TRUE;
        TrackerSample tr;
        tr.id = 1;
        tr.valid = true;
        tr.pose = pmul(w, pinv(trackerToWrist));
        f2.process(0, t, &tr, 1, &locs, cfg);
        if (frame > 300) fusedFrames++;
    }
    const std::string st = f2.status();
    printf("%s", st.c_str());
    const bool noLoop = st.find("left.relearns=0") != std::string::npos;
    const size_t p = st.find("left.camera_outliers=");
    const long outliers = p == std::string::npos ? -1 : atol(st.c_str() + p + 21);
    const bool ok2 = noLoop && outliers >= 0 && outliers < fusedFrames / 10;
    printf("moving scenario (relearns 0, outliers %ld < %d): %s\n", outliers, fusedFrames / 10, ok2 ? "PASS" : "FAIL");

    // Scenario 3: palm facing the headset. The tracker (back of the hand) is hidden for 0.6 s: the
    // runtime keeps a good orientation but its dead-reckoned position drifts at 10 cm/s (measured). The camera
    // sees the palm. Expect the hand to stay near the truth and never jump.
    Fusion f3;
    t = 1000000000LL;
    float maxErrHidden = 0.f, maxStep = 0.f;
    XrVector3f prevOut{};
    bool havePrev = false;
    for (int frame = 0; frame < 900; frame++, t += dt) {
        const bool hidden = frame >= 600 && frame < 654;
        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) {
            joints[i].pose = pmul(wrist, XrPosef{qidentity(), v3(0.f, 0.f, -0.01f * (float)i)});
            joints[i].pose.position = add(joints[i].pose.position, v3(noise(jitter), noise(jitter), noise(jitter)));
            joints[i].radius = 0.01f;
            joints[i].locationFlags = 0xF;
        }
        locs.isActive = XR_TRUE;
        TrackerSample tr;
        tr.id = 1;
        tr.valid = true;
        tr.positionTracked = !hidden;
        tr.pose = trackerPose;
        if (hidden) tr.pose.position = add(tr.pose.position, v3(0.1f * (frame - 600) / 90.f, 0.f, 0.f));
        f3.process(1, t, &tr, 1, &locs, cfg);
        const XrVector3f out = joints[XR_HAND_JOINT_WRIST_EXT].pose.position;
        const XrVector3f truth = add(wrist.position, v3(0.f, 0.f, -0.01f * XR_HAND_JOINT_WRIST_EXT));
        if (frame >= 590 && frame < 760) {
            maxErrHidden = std::fmax(maxErrHidden, length(sub(out, truth)));
            if (havePrev) maxStep = std::fmax(maxStep, length(sub(out, prevOut)));
        }
        prevOut = out;
        havePrev = true;
    }
    const std::string st3 = f3.status();
    const size_t pi = st3.find("right.frames_imu_camera=");
    printf("hidden tracker: max error %.1f mm, max frame step %.1f mm, imu_camera frames %ld\n",
           1000.f * maxErrHidden, 1000.f * maxStep, pi == std::string::npos ? -1L : atol(st3.c_str() + pi + 24));
    const bool ok3 = maxErrHidden < 0.02f && maxStep < 0.005f;
    printf("hidden scenario: %s\n", ok3 ? "PASS" : "FAIL");

    // Scenario 4: camera latency measured during the moving scenario (camera delayed by 5 frames = 55.6 ms).
    const size_t pl = st.find("camera_lag_ms=");
    const float lagMs = pl == std::string::npos ? -1.f : (float)atof(st.c_str() + pl + 14);
    const size_t pw = st.find("camera_lag_windows=");
    const long windows = pw == std::string::npos ? 0 : atol(st.c_str() + pw + 19);
    const bool ok4 = std::fabs(lagMs - 55.6f) < 12.f && windows > 0;
    printf("camera latency measured: %.0f ms over %ld windows (truth 56 ms): %s\n", lagMs, windows,
           ok4 ? "PASS" : "FAIL");

    // Scenario 5: fingers mostly straight, curled for the last second, then the hand leaves the cameras'
    // view. The fingers should relax from the curled pose toward the usual (straight) one.
    Fusion f5;
    t = 1000000000LL;
    float tipCurled = 0.f, tipAfter = 0.f, tipStraight = 0.f;
    for (int frame = 0; frame < 1200; frame++, t += dt) {
        const bool cameraSees = frame < 900;
        const bool curled = frame >= 860;
        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) {
            const XrVector3f local = curled ? v3(0.f, -0.004f * (float)i, -0.004f * (float)i) : v3(0.f, 0.f, -0.01f * (float)i);
            joints[i].pose = pmul(wrist, XrPosef{qidentity(), local});
            joints[i].pose.position = add(joints[i].pose.position, v3(noise(jitter), noise(jitter), noise(jitter)));
            joints[i].radius = 0.01f;
            joints[i].locationFlags = cameraSees ? 0xF : 0;
        }
        locs.isActive = cameraSees ? XR_TRUE : XR_FALSE;
        TrackerSample tr;
        tr.id = 1;
        tr.valid = true;
        tr.pose = trackerPose;
        f5.process(1, t, &tr, 1, &locs, cfg);
        const XrVector3f tip = sub(joints[XR_HAND_JOINT_INDEX_TIP_EXT].pose.position, joints[XR_HAND_JOINT_PALM_EXT].pose.position);
        const XrVector3f straight = v3(0.f, 0.f, -0.01f * (XR_HAND_JOINT_INDEX_TIP_EXT - XR_HAND_JOINT_PALM_EXT));
        if (frame == 899) tipCurled = length(sub(tip, straight));
        if (frame == 1199) {
            tipAfter = length(sub(tip, straight));
            tipStraight = tipAfter;
        }
    }
    const bool ok5 = tipCurled > 0.02f && tipAfter < tipCurled * 0.4f;
    printf("fingers out of view: index tip %.0f mm from the usual pose when lost, %.0f mm after 3 s: %s\n",
           1000.f * tipCurled, 1000.f * tipStraight, ok5 ? "PASS" : "FAIL");

    // Scenario 6: the hand leaves the cameras' view holding a fist, but the runtime keeps sending the last
    // open hand it saw, moved as one rigid block and flagged as tracked. It must not be taken as camera data.
    Fusion f6;
    t = 1000000000LL;
    XrPosef last[XR_HAND_JOINT_COUNT_EXT];
    long extrapolated = 0;
    float tipEnd = 0.f;
    for (int frame = 0; frame < 1100; frame++, t += dt) {
        const bool seen = frame < 800;
        if (seen) {
            for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) {
                joints[i].pose = pmul(wrist, XrPosef{qidentity(), v3(0.f, 0.f, -0.01f * (float)i)});
                joints[i].pose.position = add(joints[i].pose.position, v3(noise(jitter), noise(jitter), noise(jitter)));
                last[i] = joints[i].pose;
            }
        } else {
            for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) {
                joints[i].pose = last[i];
                joints[i].pose.position = add(last[i].position, v3(0.003f * (frame - 800), 0.f, 0.f));
            }
        }
        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) {
            joints[i].radius = 0.01f;
            joints[i].locationFlags = 0xF;
        }
        locs.isActive = XR_TRUE;
        TrackerSample tr;
        tr.id = 1;
        tr.valid = true;
        tr.positionTracked = seen;
        tr.pose = trackerPose;
        f6.process(1, t, &tr, 1, &locs, cfg);
        (void)tipEnd;
    }
    const std::string st6 = f6.status();
    const size_t pe = st6.find("right.camera_extrapolated=");
    extrapolated = pe == std::string::npos ? 0 : atol(st6.c_str() + pe + 26);
    const bool ok6 = extrapolated > 250;
    printf("rigid extrapolation detected on %ld / 300 frames: %s\n", extrapolated, ok6 ? "PASS" : "FAIL");

    // Scenario 7: headset at the origin looking down -Z. The hand is calibrated in front of it, then the
    // tracker goes to the side (out of the cameras' view) while the runtime keeps sending a noisy open hand
    // in front. That camera data must be ignored: the hand follows its tracker and the fingers relax.
    Fusion f7;
    t = 1000000000LL;
    XrPosef head = pidentity();
    head.position = v3(0.2f, 1.5f, 0.f);
    const XrPosef side = XrPosef{qidentity(), v3(0.9f, 1.1f, 0.0f)};  // 90 deg to the right of the head
    float sideErr = 0.f;
    for (int frame = 0; frame < 1100; frame++, t += dt) {
        const bool away = frame >= 800;
        const XrPosef truth = away ? side : wrist;
        const bool curled = frame >= 760 && frame < 800;
        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) {
            const XrVector3f local = curled ? v3(0.f, -0.004f * (float)i, -0.004f * (float)i) : v3(0.f, 0.f, -0.01f * (float)i);
            joints[i].pose = pmul(away ? wrist : truth, XrPosef{qidentity(), away ? v3(0.f, 0.f, -0.01f * (float)i) : local});
            joints[i].pose.position = add(joints[i].pose.position, v3(noise(jitter), noise(jitter), noise(jitter)));
            joints[i].radius = 0.01f;
            joints[i].locationFlags = 0xF;
        }
        locs.isActive = XR_TRUE;
        TrackerSample tr;
        tr.id = 1;
        tr.valid = true;
        tr.positionTracked = true;
        tr.pose = pmul(truth, pinv(trackerToWrist));
        f7.process(1, t, &tr, 1, &locs, cfg, &head);
        if (frame == 1099) sideErr = length(sub(joints[XR_HAND_JOINT_WRIST_EXT].pose.position,
                                                add(side.position, v3(0.f, 0.f, -0.01f * XR_HAND_JOINT_WRIST_EXT))));
    }
    const std::string st7 = f7.status();
    const size_t po = st7.find("right.out_of_view=");
    const long outView = po == std::string::npos ? 0 : atol(st7.c_str() + po + 18);
    const bool ok7 = outView >= 290 && sideErr < 0.02f;
    printf("out of the cameras' view: %ld / 300 frames ignored, hand %.0f mm from its tracker position: %s\n",
           outView, 1000.f * sideErr, ok7 ? "PASS" : "FAIL");

    const bool ok = ok1 && ok2 && ok3 && ok4 && ok5 && ok6 && ok7;
    printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
