# Pico Hand Fusion

> ⚠️ **Experimental, not stable yet.** This is a work in progress, tested on a single headset. It can still make PICO Connect misbehave (stream lag, crashes, hands jumping). Use it at your own risk, and keep a way to disable it (see [Uninstall](#uninstall)). Personal project, not affiliated with PICO / ByteDance. Requires root (Magisk with Zygisk).

Uses a **Pico Motion Tracker worn on the back of each hand** to stabilize the **camera hand tracking** of a rooted Pico 4 / Pico 4 Pro in **PICO Connect** (PC VR streaming to SteamVR): much less hand tremor, and hands that keep being tracked outside the cameras' field of view.

Tested on Pico 4 Pro (A8110), Pico OS 5.13.7, PICO Connect 10.6.6, Magisk 30.7.

## Results so far

Measured with the built-in recorder (60 s, both hands, 4 phases):

| Phase | Camera only | Fused output |
|---|---|---|
| Hands still | 13-23 mm, 3-4° jitter | 1.8-2.2 mm, 0.6-0.7° |
| Slow moves | 18-20 mm | 3.3-4.0 mm |
| Fast moves | 16-22 mm | 3.8-5.1 mm |
| Hands out of view | 23-33 mm | 5.1-6.0 mm |

## How it works

### Where the data flows

- The headset side of PICO Connect streams from the `com.picovr.picostreamassistant:deviceservice` process. Its `libstreaming_server.so` is an OpenXR client: it reads hands with `xrLocateHandJointsEXT` and sends them to the PC.
- PICO Connect bundles its own OpenXR loader (`PxrLoader` 1.0.24). It **ignores API layers** and loads the runtime through the PICO runtime's Java `DriverLoader`, so neither an implicit layer nor an `active_runtime.json` override reaches it.
- The module is therefore a **Zygisk module** that only runs inside that process (it unloads itself everywhere else). It patches 6 OpenXR imports of `libstreaming_server.so` (`xrGetInstanceProcAddr`, `xrCreateInstance`, `xrDestroyInstance`, `xrCreateSession`, `xrDestroySession`, `xrPollEvent`) and intercepts the hand tracking calls through `xrGetInstanceProcAddr`.
- Trackers come from the runtime's `XR_PICO_motion_tracking` extension. Tracker ids are opaque runtime handles: they are requested with `xrRequestMotionTrackerDevicePICO` (guessing ids crashes the runtime). Tracker poses are located on a worker thread, never on PICO Connect's threads (blocking there starved the video decoder).

Nothing changes on the PC side.

### The fusion, per hand

1. **Assignment**: the tracker closest to the palm (under 12 cm) for about half a second is locked to that hand.
2. **Calibration**: the palm pose in the tracker frame is learned from camera frames (the tracker is rigid with the palm). The camera lags the tracker by ~45 ms, so camera frames are compared with the tracker pose from that long ago. The offset is remembered across sessions; a lasting disagreement relearns it in the background.
3. **Tracker seen by the headset**: the hand position and orientation come from the tracker (~2 mm jitter); the cameras only provide the finger shape, smoothed by One Euro filters.
4. **Tracker hidden** (palm facing the headset): the tracker IMU keeps the orientation; the position follows the tracker's dead-reckoned motion, pulled toward the camera by a complementary filter. Runtime re-snaps are ignored.
5. **Camera lost or frozen** (the runtime keeps returning a frozen pose flagged as tracked): the hand follows its tracker with the last finger shape.
6. **Full frequency**: a root daemon keeps the trackers at full frequency while a hand is driven by a tracker.

## Contents

| Path | Purpose |
|---|---|
| `layer/src/zygisk_main.cpp` | Zygisk entry point (streaming process only) |
| `layer/src/layer.cpp` | OpenXR interception, tracker worker, status file, offset persistence |
| `layer/src/plthook.cpp` | GOT patching of `libstreaming_server.so` |
| `layer/src/fusion.cpp` | The fusion |
| `layer/src/recorder.cpp` | Raw data recorder for tuning |
| `layer/test/fusion_test.cpp` | Synthetic tests (run on the headset) |
| `companion/` | Companion app: settings, live status, recalibration, PICO Connect restart |
| `magisk/` | Magisk module (Zygisk library, full-frequency daemon) |
| `tools/` | `record.sh`, `analyze_record.py`, `setconf.sh` |
| `build.sh` | Builds everything into `out/` |

## Build

Requirements: Android NDK r27+, Android SDK build-tools (aapt2, d8, zipalign, apksigner), an `android.jar` (API 29+) and a JDK 17+. Default locations are under `~/Android`, see the top of `build.sh`.

```bash
./build.sh
```

## Install

Download `pico_hand_fusion.zip` and `companion.apk` from the [Releases](https://github.com/ImRyuma/pico-hand-fusion/releases) page (pre-releases, not stable yet), or build them yourself (see above). Zygisk must be enabled in the Magisk app.

```bash
adb push out/pico_hand_fusion.zip /data/local/tmp/
adb shell su -c "magisk --install-module /data/local/tmp/pico_hand_fusion.zip"
adb install -r out/companion.apk
adb reboot
```

Then:

1. In the **Motion Tracker** app, switch the hand trackers to **Object tracking** mode.
2. Strap them on the back of the hands.
3. Start PICO Connect with hand tracking (controllers off) and keep the hands in view for a few seconds the first time.

## Settings

`/data/data/com.picovr.picostreamassistant/files/hand_fusion.conf`, reloaded live (written by the companion app, or from the PC with `tools/setconf.sh key=value`):

| Key | Default | Purpose |
|---|---|---|
| `enabled` | `1` | Turns the fusion on or off |
| `full_frequency` | `1` | Keep trackers at full frequency during sessions |
| `tracked_weight` / `rotation_weight` | `1.0` / `1.0` | Tracker weight for position / orientation when the cameras see the hand |
| `outlier_dist` | `0.08` | Camera palm farther than this from the tracker is rejected (m) |
| `camera_lag_ms` | `45` | Camera latency relative to the tracker |
| `wrist_min_cutoff` / `wrist_beta` | `4.0` / `1.0` | One Euro filter of the palm pose |
| `finger_min_cutoff` / `finger_beta` | `1.5` / `0.4` | One Euro filter of the fingers |
| `hold_timeout_ms` | `0` | How long a hand lost by the cameras follows its tracker (`0` = forever) |
| `min_samples` / `learn_rate` | `120` / `0.01` | Calibration |
| `assign_max_dist` | `0.12` | Max tracker to palm distance for automatic assignment (m) |
| `left_tracker` / `right_tracker` | `-1` | Manual tracker ids (`-1` = automatic) |
| `calib_epoch` | `0` | Changing it forgets every learned offset |
| `record_epoch` / `record_seconds` | `0` / `60` | Changing `record_epoch` records raw data to `hand_fusion_record.csv` |

Live state: `hand_fusion_status.txt` in the same folder, or `adb logcat -s HandFusion`. Tuning session: `tools/record.sh 60 out.csv`, then `python tools/analyze_record.py out.csv`.

## Known limitations

- Not stable yet: tested on one headset and one firmware; a PICO Connect update can break the hooks.
- The trackers on the back of the hands are often hidden from the headset (30-70 % of the time in tests).
- When a hand is out of view, fingers stay frozen in the last pose the cameras saw.
- Trackers in Object tracking mode are no longer available for full body tracking; full frequency uses more tracker battery.
- The companion app does not expose every setting yet.

## Uninstall

Disable or remove the module in the Magisk app, or:

```bash
adb shell su -c "touch /data/adb/modules/pico_hand_fusion/remove"
adb uninstall com.ryuma.handfusion
adb reboot
```

## Related

[Pico Motion Tracker Tuner](https://github.com/ImRyuma/pico-motion-tracker-tuner): Magisk module that keeps the Motion Trackers awake in SteamVR when they stay still. The investigation done there (tracking service, Swift libraries) is the starting point of this project.

## License

[MIT](LICENSE). Third-party headers keep their own licenses: OpenXR headers (`layer/include/openxr`, Apache-2.0 OR MIT, The Khronos Group) and the Zygisk API header (`layer/include/zygisk`, ISC-style, John Wu).
