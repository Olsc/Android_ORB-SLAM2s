# -*- coding: utf-8 -*-
"""native-lib.cpp 合并冲突批量解决：按冲突序号应用策略。
策略依据：
  ours  = main 侧（原子量/帧计数/投影缓存等，IPC 侧为同逻辑旧版）
  theirs= IPC 侧新功能（AR_RenderFrame 锚点管线、共享内存帧路径、JNI 入口删除）
  custom= 双方叠加
"""
import sys
from pathlib import Path

F = Path("MenthaAR/src/main/cpp/native-lib.cpp")

CUSTOM_6 = """    // 进程亲和性诊断：SLAM 独立进程（:slam_process）可能被分配受限 cpuset，
    // 启动时打印可用核数便于排查调度受限问题
    {
        char cpuset[128] = {0};
        FILE* f = fopen("/proc/self/cpuset", "r");
        if (f) {
            size_t n = fread(cpuset, 1, sizeof(cpuset) - 1, f);
            cpuset[n] = 0;
            fclose(f);
        }
        cpu_set_t cs;
        CPU_ZERO(&cs);
        if (sched_getaffinity(0, sizeof(cs), &cs) == 0) {
            int nCpu = 0;
            for (int i = 0; i < CPU_SETSIZE; ++i) if (CPU_ISSET(i, &cs)) nCpu++;
            LOGD("SLAM 进程 cpuset=%s 可用核数=%d", cpuset, nCpu);
        } else {
            LOGD("SLAM 进程 cpuset=%s 可用核数=未知", cpuset);
        }
    }

    // 复核加固：赋值与首次校准在 gSlamPtrLock 内发布（构造与校准均为启动期
    // 一次性操作，此时相机/GL 尚未开始调用其他 JNI 入口，持锁无争用）
    {
        std::lock_guard<std::mutex> ptrLock(gSlamPtrLock);
        slamSys = new ORB_SLAM2::System("", ORB_SLAM2::System::MONOCULAR);
        slamSys->UpdateCalibration(fx, fy, cx, cy);
    }
"""

CUSTOM_11 = """        Plane* detected = detectPlane(TcwForPlane, vMPs, ORB_SLAM2::PLANE_DETECT_RANSAC_ITERS);
        if(detected && sys->MapChanged())
            detected->Recompute();
        statusBuf[1]=detected? ORB_SLAM2::PLANE_DETECTED : ORB_SLAM2::PLANE_NOT_DETECTED;
"""

CUSTOM_14 = """        stats[0] = sys->GetNumKeyFrames();
        stats[1] = sys->GetNumMapPoints();
        stats[2] = (gAnchor.plane != nullptr) ? 1 : 0;
"""

POLICY = {
    1: "ours",     # FrameRefGuard
    2: "ours",     # 递减移至函数末尾的注释
    3: "ours",     # currentSlamSys 快照 + 提前返回
    4: "theirs",   # AR_RenderFrame 锚点渲染管线（取代内联对齐门控，语义超集）
    5: "ours",     # 帧计数丢失重置 + CPU 绘制段
    6: ("custom", CUSTOM_6),      # cpuset 诊断（IPC）+ 锁内构造（main）
    7: "ours",     # 锁内 UpdateCalibration
    8: "ours",     # sys 快照 + maxPoints 保存
    9: "theirs",   # IPC 删除 getCurrentMapId/nativeProcessFrameMat（Java 侧无调用方）
    10: "ours",    # detect 锁内快照
    11: ("custom", CUSTOM_11),    # AR_OnArPlaced 事件（IPC）+ sys 快照（main）
    12: "ours",    # nativeGetMVP 快路径 + 投影缓存
    13: "ours",    # getV 快路径
    14: ("custom", CUSTOM_14),    # sys 快照 + gAnchor.plane（锚点集中状态）
    15: "ours",    # getMiniMapPoints 持锁采样
    16: "theirs",  # isEnableSLAM + 共享内存帧路径
}

def main():
    lines = F.read_text(encoding="utf-8").splitlines(keepends=True)
    out, i, idx = [], 0, 0
    while i < len(lines):
        if lines[i].startswith("<<<<<<<"):
            ours, j = [], i + 1
            while not lines[j].startswith("======="):
                ours.append(lines[j]); j += 1
            theirs, j = [], j + 1
            while not lines[j].startswith(">>>>>>>"):
                theirs.append(lines[j]); j += 1
            idx += 1
            pol = POLICY[idx]
            if pol == "ours":
                out.extend(ours)
            elif pol == "theirs":
                out.extend(theirs)
            else:
                out.extend(pol[1].splitlines(keepends=True))
            i = j + 1
        else:
            out.append(lines[i]); i += 1
    F.write_text("".join(out), encoding="utf-8")
    print(f"已解决 {idx} 处冲突")
    assert idx == 16, f"预期 16 处，实际 {idx}"

if __name__ == "__main__":
    sys.exit(main())