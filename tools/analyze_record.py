"""Analyzes hand_fusion_record.csv (written by the layer when record_epoch changes).

Usage: python analyze_record.py hand_fusion_record.csv [phases]

The recording is split into `phases` equal parts (default 4, e.g. still / slow / fast /
out of view). For each hand and phase it reports availability, high-frequency jitter of
the camera wrist, the tracker and the fused output, tracker jumps, the spread of the
tracker -> wrist offset, and whether the trackers look swapped between hands.
"""
import csv
import math
import sys

MODES = ["idle", "camera", "learning", "fused", "tracker_only", "lost", "imu_camera"]


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def qrot(q, v):
    p = qmul(qmul(q, (v[0], v[1], v[2], 0.0)), qconj(q))
    return p[:3]


def qangle(a, b):
    d = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2 * math.acos(min(1.0, d)))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def offset(trk_p, trk_q, cam_p, cam_q):
    """Wrist pose in the tracker frame."""
    inv = qconj(trk_q)
    return qrot(inv, sub(cam_p, trk_p)), qmul(inv, cam_q)


def jitter_pos(points, win=9):
    """RMS distance (mm) of each point to the centered moving average: tremor, not motion."""
    if len(points) < win:
        return float("nan")
    h = win // 2
    acc = []
    for i in range(h, len(points) - h):
        avg = [sum(p[k] for p in points[i - h:i + h + 1]) / win for k in range(3)]
        acc.append(norm(sub(points[i], avg)) ** 2)
    return 1000 * math.sqrt(sum(acc) / len(acc))


def jitter_rot(quats, win=9):
    """RMS angle (deg) between each orientation and its neighbors' average."""
    if len(quats) < win:
        return float("nan")
    h = win // 2
    acc = []
    for i in range(h, len(quats) - h):
        ref = quats[i]
        s = [0.0, 0.0, 0.0, 0.0]
        for q in quats[i - h:i + h + 1]:
            sign = 1 if sum(a * b for a, b in zip(q, ref)) >= 0 else -1
            for k in range(4):
                s[k] += sign * q[k]
        n = math.sqrt(sum(x * x for x in s))
        avg = tuple(x / n for x in s)
        acc.append(qangle(ref, avg) ** 2)
    return math.sqrt(sum(acc) / len(acc))


def fmt(v, unit=""):
    return "   n/a" if v != v else f"{v:6.1f}{unit}"


def main():
    path = sys.argv[1]
    phases = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    rows = list(csv.DictReader(open(path, newline="")))
    if not rows:
        print("empty recording")
        return
    t0 = int(rows[0]["time_ns"])
    t1 = int(rows[-1]["time_ns"])
    span = max(1, t1 - t0)
    print(f"{len(rows)} rows, {span / 1e9:.1f} s, {phases} phases\n")

    for r in rows:
        r["phase"] = min(phases - 1, (int(r["time_ns"]) - t0) * phases // span)
        r["cam_p"] = tuple(float(r[k]) for k in ("cam_px", "cam_py", "cam_pz"))
        r["cam_q"] = tuple(float(r[k]) for k in ("cam_qx", "cam_qy", "cam_qz", "cam_qw"))
        r["trk_p"] = tuple(float(r[k]) for k in ("trk_px", "trk_py", "trk_pz"))
        r["trk_q"] = tuple(float(r[k]) for k in ("trk_qx", "trk_qy", "trk_qz", "trk_qw"))
        r["out_p"] = tuple(float(r[k]) for k in ("out_px", "out_py", "out_pz"))
        r["out_q"] = tuple(float(r[k]) for k in ("out_qx", "out_qy", "out_qz", "out_qw"))
        r["palm"] = tuple(float(r[k]) for k in ("palm_x", "palm_y", "palm_z"))
        r["cam_ok"] = r["active"] == "1" and (int(r["wrist_flags"]) & 0xF) == 0xF
        r["trk_ok"] = r["tracker_valid"] == "1"
        # POSITION_TRACKED bit (0x8): the tracker is seen by the headset cameras
        r["trk_optical"] = r["trk_ok"] and (int(r.get("tracker_flags", "15") or 15) & 0x8) != 0

    for hand in ("0", "1"):
        name = "LEFT" if hand == "0" else "RIGHT"
        hrows = [r for r in rows if r["hand"] == hand]
        ids = sorted({r["tracker_id"] for r in hrows if r["tracker_id"] != "-1"})
        print(f"===== {name} hand: {len(hrows)} frames, tracker id(s) {', '.join(ids) or 'none'}")
        print("phase | cam% trk% opt% | jitter pos mm (cam/trk/out) | jitter rot deg (cam/trk/out) | "
              "trk jumps | offset spread mm/deg | palm-trk mm | modes")
        for ph in range(phases):
            pr = [r for r in hrows if r["phase"] == ph]
            if not pr:
                continue
            cam = [r for r in pr if r["cam_ok"]]
            trk = [r for r in pr if r["trk_ok"]]
            both = [r for r in pr if r["cam_ok"] and r["trk_ok"]]
            jumps = sum(1 for a, b in zip(trk, trk[1:]) if norm(sub(a["trk_p"], b["trk_p"])) > 0.05)
            offs = [offset(r["trk_p"], r["trk_q"], r["cam_p"], r["cam_q"]) for r in both]
            spread_p = spread_a = float("nan")
            if len(offs) > 5:
                mp = [sum(o[0][k] for o in offs) / len(offs) for k in range(3)]
                spread_p = 1000 * math.sqrt(sum(norm(sub(o[0], mp)) ** 2 for o in offs) / len(offs))
                ref = offs[len(offs) // 2][1]
                spread_a = math.sqrt(sum(qangle(o[1], ref) ** 2 for o in offs) / len(offs))
            palm_trk = (1000 * sum(norm(sub(r["palm"], r["trk_p"])) for r in both) / len(both)) if both else float("nan")
            modes = {}
            for r in pr:
                m = int(r["mode"])
                key = MODES[m] if 0 <= m < len(MODES) else "?"
                modes[key] = modes.get(key, 0) + 1
            mode_txt = " ".join(f"{k}:{100 * v // len(pr)}%" for k, v in sorted(modes.items(), key=lambda x: -x[1]))
            opt = sum(1 for r in pr if r["trk_optical"])
            print(f"  {ph + 1}   | {100 * len(cam) // len(pr):3d}% {100 * len(trk) // len(pr):3d}% {100 * opt // len(pr):3d}% | "
                  f"{fmt(jitter_pos([r['cam_p'] for r in cam]))} {fmt(jitter_pos([r['trk_p'] for r in trk]))} "
                  f"{fmt(jitter_pos([r['out_p'] for r in pr]))} | "
                  f"{fmt(jitter_rot([r['cam_q'] for r in cam]))} {fmt(jitter_rot([r['trk_q'] for r in trk]))} "
                  f"{fmt(jitter_rot([r['out_q'] for r in pr]))} | {jumps:5d} | "
                  f"{fmt(spread_p)} {fmt(spread_a)} | {fmt(palm_trk)} | {mode_txt}")
        print()

    # Swap check: is each tracker closer to its own hand's palm than to the other palm?
    by_time = {}
    for r in rows:
        by_time.setdefault(r["time_ns"], {})[r["hand"]] = r
    own = other = n = 0
    for pair in by_time.values():
        if "0" in pair and "1" in pair:
            l, rr = pair["0"], pair["1"]
            if l["cam_ok"] and rr["cam_ok"] and l["trk_ok"] and rr["trk_ok"]:
                own += norm(sub(l["trk_p"], l["palm"])) + norm(sub(rr["trk_p"], rr["palm"]))
                other += norm(sub(l["trk_p"], rr["palm"])) + norm(sub(rr["trk_p"], l["palm"]))
                n += 1
    if n:
        print(f"swap check over {n} frames: own palm {500 * own / n:.0f} mm, other palm {500 * other / n:.0f} mm "
              f"-> {'OK' if own < other else 'TRACKERS LOOK SWAPPED'}")


if __name__ == "__main__":
    main()
