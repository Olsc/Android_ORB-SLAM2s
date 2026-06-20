#!/usr/bin/env python3
"""
分析 profile_trace.json 中的 SLAM 各阶段耗时分布，
找出导致扫描/跟踪不稳定的性能瓶颈。
"""

import json
import sys
from collections import defaultdict
import statistics

TRACE_FILE = r"profile_trace.json"

def load_trace(path):
    with open(path, 'r') as f:
        data = json.load(f)
    print(f"加载了 {len(data)} 条 trace 事件")
    return data

def analyze(events):
    # 按线程分组
    threads = defaultdict(list)
    for ev in events:
        threads[ev['tid']].append(ev)

    print(f"共有 {len(threads)} 个线程")
    for tid, evs in threads.items():
        names = set(e['name'] for e in evs)
        print(f"  线程 {tid}: {len(evs)} 个事件, 包含 {len(names)} 种函数")

    # 解析完整函数区间 (B/E)
    durations = defaultdict(list)

    # 用栈追踪每线程的调用
    stacks = defaultdict(list)

    for ev in events:
        tid = ev['tid']
        name = ev['name']
        ts = ev['ts']
        ph = ev['ph']

        if ph == 'B':
            stacks[tid].append((name, ts))
        elif ph == 'E':
            if stacks[tid]:
                bname, bts = stacks[tid].pop()
                if bname == name:  # 匹配B/E
                    dur = ts - bts
                    durations[name].append(dur)
                else:
                    # B/E 不匹配，回退到名字匹配
                    found = False
                    for i in range(len(stacks[tid]) - 1, -1, -1):
                        if stacks[tid][i][0] == name:
                            bts = stacks[tid][i][1]
                            dur = ts - bts
                            durations[name].append(dur)
                            stacks[tid].pop(i)
                            found = True
                            break
                    if not found:
                        pass  # 无法匹配

    # 打印分析结果
    print("\n" + "="*80)
    print("各函数耗时统计 (按总耗时降序)")
    print("="*80)
    print(f"{'函数名':<55} {'调用次数':>8} {'平均(us)':>10} {'中位(us)':>10} {'最小(us)':>10} {'最大(us)':>10} {'总耗时(us)':>12}")
    print("-"*80)

    # 按总耗时排序
    sorted_funcs = sorted(durations.items(), key=lambda x: sum(x[1]), reverse=True)

    for name, durs in sorted_funcs:
        if len(durs) < 2:
            continue
        avg = statistics.mean(durs)
        med = statistics.median(durs)
        mn = min(durs)
        mx = max(durs)
        total = sum(durs)
        print(f"{name:<55} {len(durs):>8} {avg:>10.1f} {med:>10.1f} {mn:>10.1f} {mx:>10.1f} {total:>12.1f}")

    # 分析关键路径
    print("\n" + "="*80)
    print("关键性能分析")
    print("="*80)

    # 1. processImage 总体耗时
    if 'int processImage(cv::Mat &, cv::Mat &, int *)' in durations:
        pid = 'int processImage(cv::Mat &, cv::Mat &, int *)'
        durs = durations[pid]
        print(f"\n▶ processImage (主循环单帧总耗时):")
        print(f"  调用 {len(durs)} 次, 平均 {statistics.mean(durs)/1000:.1f}ms, "
              f"中位 {statistics.median(durs)/1000:.1f}ms, "
              f"最大 {max(durs)/1000:.1f}ms")

        # 统计>33ms(30fps) 的帧
        slow = [d for d in durs if d > 33000]
        print(f"  超过 33ms (30FPS 帧预算) 的帧: {len(slow)}/{len(durs)} = {len(slow)/len(durs)*100:.1f}%")
        very_slow = [d for d in durs if d > 66000]
        print(f"  超过 66ms (15FPS) 的帧: {len(very_slow)}/{len(durs)} = {len(very_slow)/len(durs)*100:.1f}%")

        # 耗时分布
        buckets = [(0, 16), (16, 33), (33, 50), (50, 66), (66, 100), (100, 200), (200, 999999)]
        print(f"\n  耗时分布:")
        for lo, hi in buckets:
            cnt = sum(1 for d in durs if lo*1000 <= d < hi*1000)
            bar = '█' * int(cnt / max(1, len(durs)) * 80)
            label = f"{lo}-{hi}ms" if hi < 999999 else f">{lo}ms"
            print(f"    {label:>10}: {cnt:>5} ({cnt/len(durs)*100:5.1f}%) {bar}")

    # 2. TrackMonocular 内部耗时
    track_funcs = [k for k in durations if 'Track' in k and 'processImage' not in k]
    if track_funcs:
        print(f"\n▶ 跟踪阶段细分:")
        for name in sorted(track_funcs, key=lambda n: statistics.mean(durations[n]), reverse=True):
            durs = durations[name]
            avg = statistics.mean(durs)
            total = sum(durs)
            print(f"  {name:<55}  avg={avg/1000:>7.2f}ms  cnt={len(durs):>5}  total={total/1000:>8.1f}ms")

    # 3. ORB特征提取耗时
    orb_funcs = [k for k in durations if 'ORB' in k or 'ComputePyramid' in k or 'Extractor' in k]
    if orb_funcs:
        print(f"\n▶ ORB 特征提取:")
        for name in sorted(orb_funcs, key=lambda n: sum(durations[n]), reverse=True):
            durs = durations[name]
            avg = statistics.mean(durs)
            print(f"  {name:<55}  avg={avg/1000:>7.2f}ms  cnt={len(durs):>5}")

    # 4. LocalMapping 耗时
    lm_funcs = [k for k in durations if 'LocalMapping' in k]
    if lm_funcs:
        print(f"\n▶ 局部建图 (LocalMapping) 线程:")
        for name in sorted(lm_funcs, key=lambda n: sum(durations[n]), reverse=True):
            durs = durations[name]
            avg = statistics.mean(durs)
            total = sum(durs)
            print(f"  {name:<55}  avg={avg/1000:>7.2f}ms  cnt={len(durs):>5}  total={total/1000:>8.1f}ms")

    # 5. 帧率稳定性分析
    print(f"\n▶ 帧率稳定性分析:")
    pid_events = [k for k in durations if 'processImage' in k]
    if pid_events:
        durs = durations[pid_events[0]]
        if len(durs) > 5:
            # 计算帧间间隔变异系数
            intervals = []
            for i in range(1, len(durs)):
                pass  # 没有时间戳起始信息，无法算间隔

            # 计算耗时变异系数
            cv = statistics.stdev(durs) / statistics.mean(durs) if statistics.mean(durs) > 0 else 0
            print(f"  帧耗时变异系数 (CV): {cv:.2f}  (越小越稳定, >0.5 表示剧烈波动)")

            # 检测卡顿帧比例
            durs_ms = [d/1000 for d in durs]
            p99 = sorted(durs_ms)[int(len(durs_ms) * 0.99)]
            p95 = sorted(durs_ms)[int(len(durs_ms) * 0.95)]
            print(f"  P50: {statistics.median(durs_ms):.1f}ms")
            print(f"  P95: {p95:.1f}ms")
            print(f"  P99: {p99:.1f}ms")
            print(f"  最大: {max(durs_ms):.1f}ms")

    # 6. 查找最耗时的单个操作
    print(f"\n▶ 单次调用最耗时的操作 (Top 10):")
    single_ops = []
    for name, durs in durations.items():
        for i, d in enumerate(durs):
            single_ops.append((d, name, i))
    single_ops.sort(reverse=True)
    for d, name, i in single_ops[:10]:
        print(f"  {d/1000:>8.1f}ms  {name}")


def find_fps_problems(events):
    """分析帧率不稳定问题"""
    # 提取 processImage 的 B/E 时间戳
    starts = []
    ends = []

    for ev in events:
        if ev['name'] == 'int processImage(cv::Mat &, cv::Mat &, int *)':
            if ev['ph'] == 'B':
                starts.append(ev['ts'])
            elif ev['ph'] == 'E':
                ends.append(ev['ts'])

    if len(starts) < 2:
        return

    print("\n" + "="*80)
    print("帧间隔分析 (帧率抖动检测)")
    print("="*80)

    # 帧间间隔 (从一帧开始到下一帧开始)
    intervals = [starts[i+1] - starts[i] for i in range(len(starts)-1)]

    if intervals:
        # 转毫秒
        intervals_ms = [i/1000 for i in intervals]
        avg_interval = statistics.mean(intervals_ms)
        min_interval = min(intervals_ms)
        max_interval = max(intervals_ms)
        cv_interval = statistics.stdev(intervals_ms) / avg_interval if avg_interval > 0 else 0

        print(f"  帧间隔 (开始->开始):")
        print(f"    平均: {avg_interval:.1f}ms ({1000/avg_interval:.1f} FPS)")
        print(f"    最小: {min_interval:.1f}ms")
        print(f"    最大: {max_interval:.1f}ms ({1000/max_interval:.1f} FPS)")
        print(f"    变异系数: {cv_interval:.2f}")

        # 检测是否掉帧
        dropped = [i for i in intervals_ms if i > 50]  # >50ms 视为掉帧
        if dropped:
            print(f"\n  掉帧 (>50ms 间隔): {len(dropped)}/{len(intervals_ms)} = {len(dropped)/len(intervals_ms)*100:.1f}%")
            freeze = [i for i in intervals_ms if i > 200]  # >200ms 视为卡死
            if freeze:
                print(f"  卡顿 (>200ms 间隔): {len(freeze)}/{len(intervals_ms)} = {len(freeze)/len(intervals_ms)*100:.1f}%")


if __name__ == '__main__':
    print(f"分析文件: {TRACE_FILE}")
    print("="*80)

    events = load_trace(TRACE_FILE)
    analyze(events)
    find_fps_problems(events)

    print("\n✅ 分析完成")
