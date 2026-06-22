#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VtonaxAR Benchmark analysis report
"""
import json, csv, math
from pathlib import Path

REPORT_DIR = Path(__file__).resolve().parent / "build"

def state_name(s):
    return {0:"NO_IMG", 1:"INIT", 2:"OK", 3:"LOST"}.get(s, f"UNK({s})")

def load_data():
    with open(REPORT_DIR / "benchmark_report.json", "r", encoding="utf-8", errors="replace") as f:
        j = json.load(f)
    frames = []
    with open(REPORT_DIR / "benchmark_report.csv", "r", encoding="utf-8", errors="replace") as f:
        for row in csv.DictReader(f):
            row['frame_id'] = int(row['frame_id'])
            row['tracking_state'] = int(row['tracking_state'])
            row['process_ms'] = float(row['process_ms'])
            row['tracked_points'] = int(row['tracked_points'])
            row['keypoints'] = int(row['keypoints'])
            row['is_keyframe'] = int(row['is_keyframe'])
            row['total_map_points'] = int(row['total_map_points'])
            row['total_keyframes'] = int(row['total_keyframes'])
            row['timestamp'] = float(row['timestamp'])
            frames.append(row)
    return j, frames

def pct(a, b):
    return a / b * 100 if b > 0 else 0.0

def main():
    j, frames = load_data()
    total = len(frames)
    vfps = j['video']['fps']
    ok_ct = sum(1 for f in frames if f['tracking_state'] == 2)
    lost_ct = sum(1 for f in frames if f['tracking_state'] == 3)
    init_ct = sum(1 for f in frames if f['tracking_state'] == 1)

    times = [f['process_ms'] for f in frames]
    ok_times = [f['process_ms'] for f in frames if f['tracking_state'] == 2]
    mps_ok = [f['tracked_points'] for f in frames if f['tracking_state'] == 2]

    print("=" * 78)
    print("  VtonaxAR Benchmark  Complete Analysis Report")
    print("=" * 78)
    print(f"  Video       : {j['video']['path']}")
    print(f"  Duration    : {total / vfps:.1f}s ({total} frames @ {vfps:.1f} FPS)")
    print()

    # 1. 跟踪稳定性
    print("-" * 78)
    print("  1. Tracking Stability")
    print("-" * 78)
    print(f"  OK    : {ok_ct:6d}  ({pct(ok_ct, total):5.1f}%)")
    print(f"  LOST  : {lost_ct:6d}  ({pct(lost_ct, total):5.1f}%)")
    print(f"  INIT  : {init_ct:6d}  ({pct(init_ct, total):5.1f}%)")
    print(f"  Total : {total:6d}")

    # 连续跟踪段分析
    segs_ok, segs_lost = [], []
    cur_state = frames[0]['tracking_state'] if frames else -1
    cur_start = 0
    for i, f in enumerate(frames):
        if f['tracking_state'] != cur_state:
            length = i - cur_start
            if cur_state == 2:
                segs_ok.append(length)
            elif cur_state == 3:
                segs_lost.append(length)
            cur_start = i
            cur_state = f['tracking_state']
    length = total - cur_start
    if cur_state == 2:
        segs_ok.append(length)
    elif cur_state == 3:
        segs_lost.append(length)

    max_ok = max(segs_ok) if segs_ok else 0
    max_lost = max(segs_lost) if segs_lost else 0
    print(f"\n  Longest OK segment   : {max_ok:6d} frames ({max_ok/vfps:.1f}s)")
    print(f"  Longest LOST segment : {max_lost:6d} frames ({max_lost/vfps:.1f}s)")
    print(f"  OK segment count     : {len(segs_ok):6d}")
    print(f"  LOST segment count   : {len(segs_lost):6d}")

    # 2. 实时性分析
    print()
    print("-" * 78)
    print("  2. Real-time Performance")
    print("-" * 78)
    times_sorted = sorted(times)
    ok_times_sorted = sorted(ok_times)
    def percentile(data, p):
        if not data: return 0.0
        idx = min(len(data)-1, int(len(data)*p/100))
        return data[idx]

    mean_t = sum(times) / len(times) if times else 0.0
    mean_ok = sum(ok_times) / len(ok_times) if ok_times else 0.0

    print(f"  All frames:")
    print(f"    Mean  : {mean_t:.2f} ms")
    print(f"    P50   : {percentile(times_sorted, 50):.2f} ms")
    print(f"    P95   : {percentile(times_sorted, 95):.2f} ms")
    print(f"    P99   : {percentile(times_sorted, 99):.2f} ms")
    budget_33 = sum(1 for t in times if t < 1000/30)
    print(f"    <33ms : {pct(budget_33, total):.1f}% ({budget_33}/{total})")
    print(f"  OK frames only:")
    print(f"    Mean  : {mean_ok:.2f} ms")
    print(f"    P50   : {percentile(ok_times_sorted, 50):.2f} ms")
    print(f"    P95   : {percentile(ok_times_sorted, 95):.2f} ms")

    # 耗时分布
    print(f"\n  Timing Distribution:")
    buckets = [10, 20, 30, 50, 80, 120, 200, 500]
    hist = [0] * len(buckets)
    for t in times:
        for i, b in enumerate(buckets):
            if t <= b:
                hist[i] += 1
                break
        else:
            hist.append(1)
    prev = 0
    for i, b in enumerate(buckets):
        if hist[i] > 0:
            print(f"    {prev:4d}-{b:3d}ms : {'█' * (hist[i]//max(1,total//40))} {hist[i]}")
        prev = b

    # 3. 建图质量
    print()
    print("-" * 78)
    print("  3. Mapping Quality")
    print("-" * 78)
    kf_counts = [f['total_keyframes'] for f in frames if f['is_keyframe'] or True]
    mp_counts = [f['total_map_points'] for f in frames]
    final_kf = kf_counts[-1] if kf_counts else 0
    final_mp = mp_counts[-1] if mp_counts else 0
    print(f"  Final Keyframes     : {final_kf}")
    print(f"  Final Map Points    : {final_mp}")
    print(f"  Avg OK tracked MP  : {sum(mps_ok)/len(mps_ok):.1f}" if mps_ok else "  Avg OK tracked MP  : N/A")

    # 4. 场景难度分类
    print()
    print("-" * 78)
    print("  4. Scene Difficulty Classification")
    print("-" * 78)
    # 基于最后一段帧的跟踪质量分类
    good = ok_ct
    medium = init_ct
    hard = lost_ct // 2
    severe = lost_ct - hard
    print(f"  Good   (OK ratio>80%) : {good:5d} frames ({pct(good, total):5.1f}%)")
    print(f"  Medium (INIT periods) : {medium:5d} frames ({pct(medium, total):5.1f}%)")
    print(f"  Hard   (short LOST)   : {hard:5d} frames ({pct(hard, total):5.1f}%)")
    print(f"  Severe (long LOST)    : {severe:5d} frames ({pct(severe, total):5.1f}%)")

    print()
    print("=" * 78)
    print("  Analysis Complete")
    print("=" * 78)

if __name__ == "__main__":
    main()
