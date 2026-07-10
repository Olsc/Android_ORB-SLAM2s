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
        print(f"\n-> processImage (主循环单帧总耗时):")
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
            bar = '#' * int(cnt / max(1, len(durs)) * 80)
            label = f"{lo}-{hi}ms" if hi < 999999 else f">{lo}ms"
            print(f"    {label:>10}: {cnt:>5} ({cnt/len(durs)*100:5.1f}%) {bar}")

    # 2. TrackMonocular 内部耗时
    track_funcs = [k for k in durations if 'Track' in k and 'processImage' not in k]
    if track_funcs:
        print(f"\n-> 跟踪阶段细分:")
        for name in sorted(track_funcs, key=lambda n: statistics.mean(durations[n]), reverse=True):
            durs = durations[name]
            avg = statistics.mean(durs)
            total = sum(durs)
            print(f"  {name:<55}  avg={avg/1000:>7.2f}ms  cnt={len(durs):>5}  total={total/1000:>8.1f}ms")

    # 3. ORB特征提取耗时
    orb_funcs = [k for k in durations if 'ORB' in k or 'ComputePyramid' in k or 'Extractor' in k]
    if orb_funcs:
        print(f"\n-> ORB 特征提取:")
        for name in sorted(orb_funcs, key=lambda n: sum(durations[n]), reverse=True):
            durs = durations[name]
            avg = statistics.mean(durs)
            print(f"  {name:<55}  avg={avg/1000:>7.2f}ms  cnt={len(durs):>5}")

    # 4. LocalMapping 耗时
    lm_funcs = [k for k in durations if 'LocalMapping' in k]
    if lm_funcs:
        print(f"\n-> 局部建图 (LocalMapping) 线程:")
        for name in sorted(lm_funcs, key=lambda n: sum(durations[n]), reverse=True):
            durs = durations[name]
            avg = statistics.mean(durs)
            total = sum(durs)
            print(f"  {name:<55}  avg={avg/1000:>7.2f}ms  cnt={len(durs):>5}  total={total/1000:>8.1f}ms")

    # 5. 帧率稳定性分析
    print(f"\n-> 帧率稳定性分析:")
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
    print(f"\n-> 单次调用最耗时的操作 (Top 10):")
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


def detect_deadlock(events):
    """检测卡死模式：未闭合函数、线程永久等待、BA过长"""
    print("\n" + "="*80)
    print("卡死/死锁检测")
    print("="*80)

    # 1. 按 TID 分组并检测未闭合 Begin
    threads = defaultdict(list)
    for ev in events:
        threads[ev['tid']].append(ev)

    # 供后续综合发现的变量
    has_unclosed = False
    open_stopped_tids = []
    ba_ops = []
    overlap_found = False

    print("\n-> 未闭合函数 (Begin 无对应 End):")
    for tid, evs in sorted(threads.items()):
        stack = []
        for ev in evs:
            if ev['ph'] == 'B':
                stack.append(ev['name'])
            elif ev['ph'] == 'E':
                if stack and stack[-1] == ev['name']:
                    stack.pop()
        if stack:
            has_unclosed = True
            print(f"  线程 {tid}: 调用栈(中断时):")
            for name in stack:
                print(f"    +- {name}")

    if not has_unclosed:
        print("    OK - 所有事件已闭合")

    # 2. Stopped 状态分析
    print("\n-> 线程停止状态 (Stopped) 检测:")
    open_stopped_tids = []
    closed_stopped_cnt = 0
    for tid, evs in sorted(threads.items()):
        in_stopped = False
        for ev in evs:
            if 'Stopped' not in ev['name']:
                continue
            if ev['ph'] == 'B':
                in_stopped = True
                closed_stopped_cnt += 1  # count each begin as one occurrence
            elif ev['ph'] == 'E':
                in_stopped = False
        if in_stopped:
            open_stopped_tids.append(tid)

    if not open_stopped_tids and closed_stopped_cnt == 0:
        print("    OK - 无线程停止状态")
    elif open_stopped_tids:
        print(f"    !!! 线程 {open_stopped_tids} 进入 Stopped 后未退出 (trace结束时仍卡在其中)")
        for tid in open_stopped_tids:
            print(f"      线程 {tid} 在 Stopped 中等待被唤醒(Release)，但无人调用 Release")
    else:
        print(f"    OK - {closed_stopped_cnt} 次 Stopped 均已正常退出")

    # 3. 长耗时操作检测 (>500ms)
    print("\n-> 长耗时操作 (单次 >500ms):")
    stacks = defaultdict(list)
    long_ops = []
    for ev in events:
        tid = ev['tid']
        if ev['ph'] == 'B':
            stacks[tid].append((ev['name'], ev['ts']))
        elif ev['ph'] == 'E' and stacks[tid]:
            bname, bts = stacks[tid].pop()
            if bname == ev['name']:
                dur = ev['ts'] - bts
                if dur > 500000:
                    long_ops.append((dur/1000, bname, tid))

    if long_ops:
        long_ops.sort(reverse=True)
        print(f"  发现 {len(long_ops)} 次 >500ms 操作:")
        for ms, name, tid in long_ops[:10]:
            print(f"    {ms:>8.0f}ms  {name}")

        ba_ops = [ms for ms, name, _ in long_ops if 'BundleAdjustment' in name]
        if ba_ops:
            print(f"\n    其中 BA {len(ba_ops)} 次, 平均 {sum(ba_ops)/len(ba_ops):.0f}ms, "
                  f"最大 {max(ba_ops):.0f}ms")
            if max(ba_ops) > 2000:
                print("    !!! BA 最大耗时 > 2s, 建议限制 BA 窗口大小")
    else:
        print("    OK - 无 >500ms 操作")

    # 4. 线程间重叠检测 (主线程 vs LM线程)
    print("\n-> 线程间堵塞检测:")
    main_tid = lm_tid = None
    for tid, evs in sorted(threads.items()):
        names = [e['name'] for e in evs]
        if any('processImage' in n for n in names):
            main_tid = tid
        if any('LocalMapping::' in n for n in names):
            lm_tid = tid

    if main_tid and lm_tid:
        print(f"    主线程={main_tid}, LocalMapping={lm_tid}")

        # 提取主线程所有帧区间
        main_iv = []
        stack = []
        for ev in threads[main_tid]:
            if ev['ph'] == 'B':
                stack.append((ev['name'], ev['ts']))
            elif ev['ph'] == 'E' and stack:
                name, ts = stack.pop()
                if name == ev['name']:
                    main_iv.append((name, ts, ev['ts'], ev['ts']-ts))

        # 提取 LM 的 BA/Search 区间
        lm_long = []
        stack = []
        for ev in threads[lm_tid]:
            if ev['ph'] == 'B':
                stack.append((ev['name'], ev['ts']))
            elif ev['ph'] == 'E' and stack:
                name, ts = stack.pop()
                if name == ev['name']:
                    dur = ev['ts'] - ts
                    if dur > 500000 or 'SearchInNeighbors' in name:
                        lm_long.append((name, ts, ev['ts'], dur))

        # 找长帧+LM操作重叠
        overlap_found = False
        for mn, ms, me, md in main_iv:
            if md < 60000:  # 只看 >60ms 的长帧
                continue
            md_ms = md/1000
            for ln, ls, le, ld in lm_long:
                if me > ls and ms < le:  # 区间重叠
                    overlap_found = True
                    print(f"    !!! 主线程({mn})耗时{md_ms:.0f}ms 与 "
                          f"{ln}({ld/1000:.0f}ms) 重叠")
                    break

        if not overlap_found:
            print("    OK - 主线程与 LM 线程无严重重叠")

        # 帧间隔极端值
        print("\n-> 帧间隔极端值:")
        starts = [ev['ts'] for ev in threads[main_tid]
                  if ev['name'] == 'int processImage(cv::Mat &, cv::Mat &, int *)' and ev['ph'] == 'B']
        if len(starts) > 2:
            gaps_ms = [(starts[i+1]-starts[i])/1000 for i in range(len(starts)-1)]
            extreme = [(i, g) for i, g in enumerate(gaps_ms) if g > 100]
            if extreme:
                print(f"    {len(extreme)} 次 >100ms 间隔:")
                for i, g in extreme[-5:]:
                    print(f"      帧{i}: 间隔{g:.0f}ms")
            else:
                print("    OK - 无 >100ms 极端间隔")
    else:
        print("    无法确定线程角色")

    # 综合发现列表：只列事实，不下定论
    print("\n-> 综合发现列表:")
    findings = []

    if has_unclosed:
        findings.append("trace中断时仍有函数未闭合")
    if open_stopped_tids:
        findings.append(f"线程 {open_stopped_tids} 在 Stopped 状态中未退出")
    if ba_ops:
        findings.append(f"BA 最大耗时 {max(ba_ops):.0f}ms")

    extreme_count = 0
    if main_tid:
        starts2 = [ev['ts'] for ev in threads[main_tid]
                   if ev['name'] == 'int processImage(cv::Mat &, cv::Mat &, int *)' and ev['ph'] == 'B']
        if len(starts2) > 2:
            gaps_ms2 = [(starts2[i+1]-starts2[i])/1000 for i in range(len(starts2)-1)]
            extreme_count = len([g for g in gaps_ms2 if g > 150])
    if extreme_count > 0:
        findings.append(f"{extreme_count} 次帧间隔 >150ms")
    if overlap_found:
        findings.append("主线程耗时与 LM 操作重叠")

    if findings:
        for f in findings:
            print(f"  - {f}")
    else:
        print("  - 未检测到异常")


if __name__ == '__main__':
    print(f"分析文件: {TRACE_FILE}")
    print("="*80)

    events = load_trace(TRACE_FILE)
    analyze(events)
    find_fps_problems(events)
    detect_deadlock(events)

    print("\nOK 分析完成")
