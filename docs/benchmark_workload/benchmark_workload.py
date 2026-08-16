#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visual SLAM Theoretical Workload Benchmark & Stress Testing Engine for Git Commits
测算每次 Git Commit 代码在不同场景/视频规格下的理论计算量（FLOPs / Ops），支持多线程并行分析与压力测试。

环境要求:
    须安装 matplotlib: pip install matplotlib
"""

import os
import sys
import re
import json
import time
import argparse
import subprocess
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List, Dict, Any, Tuple, Optional

# Windows 终端输出 UTF-8 编码安全兼容
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# Matplotlib 图表生成
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    # 尝试配置跨平台中文字体
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Segoe UI', 'DejaVu Sans', 'Arial', 'sans-serif']
    plt.rcParams['axes.unicode_minus'] = False
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# ==============================================================================
# 1. 压力测试基准预设场景 (Workload Profiles & Presets)
# ==============================================================================
WORKLOAD_PRESETS = {
    "mobile_low": {
        "name": "低功耗移动端 (QVGA 30FPS / 500特征)",
        "duration_sec": 60,
        "fps": 30,
        "width": 320,
        "height": 240,
        "n_features": 500,
        "keyframe_ratio": 0.05,
        "loop_freq": 0.5,
        "description": "适用于穿戴式设备/极低功耗 IoT SLAM"
    },
    "standard": {
        "name": "标准基准 (VGA 30FPS / 1000特征)",
        "duration_sec": 60,
        "fps": 30,
        "width": 640,
        "height": 480,
        "n_features": 1000,
        "keyframe_ratio": 0.08,
        "loop_freq": 1.0,
        "description": "ORB-SLAM2 官方推荐默认标清基准"
    },
    "hd_1080p": {
        "name": "高清性能 (1080P 60FPS / 1500特征)",
        "duration_sec": 60,
        "fps": 60,
        "width": 1920,
        "height": 1080,
        "n_features": 1500,
        "keyframe_ratio": 0.10,
        "loop_freq": 2.0,
        "description": "适用于车载/无人机高精度实时定位"
    },
    "extreme_4k": {
        "name": "极限压力测试 (4K 60FPS / 3000特征)",
        "duration_sec": 60,
        "fps": 60,
        "width": 3840,
        "height": 2160,
        "n_features": 3000,
        "keyframe_ratio": 0.12,
        "loop_freq": 5.0,
        "description": "高分辨率多通道高帧率极限算力压力测试"
    },
    "heavy_loop": {
        "name": "密集回环压力测试 (VGA 30FPS / 10Hz回环)",
        "duration_sec": 60,
        "fps": 30,
        "width": 640,
        "height": 480,
        "n_features": 1000,
        "keyframe_ratio": 0.15,
        "loop_freq": 10.0,
        "description": "针对 LoopClosing 与局部建图高频优化调度的压力测试"
    }
}

# 默认基准常数
DEFAULT_PRESET = "standard"
DEFAULT_DURATION_SEC = WORKLOAD_PRESETS[DEFAULT_PRESET]["duration_sec"]
DEFAULT_FPS = WORKLOAD_PRESETS[DEFAULT_PRESET]["fps"]
DEFAULT_WIDTH = WORKLOAD_PRESETS[DEFAULT_PRESET]["width"]
DEFAULT_HEIGHT = WORKLOAD_PRESETS[DEFAULT_PRESET]["height"]
DEFAULT_N_FEATURES = WORKLOAD_PRESETS[DEFAULT_PRESET]["n_features"]
DEFAULT_KEYFRAME_RATIO = WORKLOAD_PRESETS[DEFAULT_PRESET]["keyframe_ratio"]
DEFAULT_LOOP_CHECK_FREQ = WORKLOAD_PRESETS[DEFAULT_PRESET]["loop_freq"]

# 核心算法文件名关键字
KEY_FILE_PATTERNS = {
    "ORBextractor": ["*ORBextractor.cc", "*ORBextractor.cpp", "ORBextractor.cc"],
    "ORBmatcher": ["*ORBmatcher.cc", "*ORBmatcher.cpp", "ORBmatcher.cc"],
    "Tracking": ["*Tracking.cc", "*Tracking.cpp", "Tracking.cc"],
    "Optimizer": ["*Optimizer.cc", "*Optimizer.cpp", "Optimizer.cc"],
    "LocalMapping": ["*LocalMapping.cc", "*LocalMapping.cpp", "LocalMapping.cc"],
    "LoopClosing": ["*LoopClosing.cc", "*LoopClosing.cpp", "LoopClosing.cc"],
    "PnPsolver": ["*PnPsolver.cc", "*PnPsolver.cpp", "PnPsolver.cc"],
    "Initializer": ["*Initializer.cc", "*Initializer.cpp", "Initializer.cc"],
    "Matrix": ["*Matrix.cpp", "*Matrix.cc", "Matrix.cpp"],
    "Plane": ["*Plane.cpp", "*Plane.cc", "Plane.cpp"],
    "native-lib": ["*native-lib.cpp", "native-lib.cpp"]
}


# ==============================================================================
# 2. 理论计算量评估核心算法 (Theoretical Workload Analyzer)
# ==============================================================================
class SLAMWorkloadModel:
    """
    视觉 SLAM 理论计算量与压力测试评估模型
    根据视频分辨率、帧率、特征点数、关键帧比例和算法静态特征估算理论计算量 (Operations / FLOPs)。
    """

    def __init__(self,
                 duration_sec: int = DEFAULT_DURATION_SEC,
                 fps: int = DEFAULT_FPS,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT,
                 n_features: int = DEFAULT_N_FEATURES,
                 keyframe_ratio: float = DEFAULT_KEYFRAME_RATIO,
                 loop_freq: float = DEFAULT_LOOP_CHECK_FREQ):
        self.duration_sec = duration_sec
        self.fps = fps
        self.total_frames = duration_sec * fps
        self.width = width
        self.height = height
        self.n_features = n_features
        self.keyframe_ratio = keyframe_ratio
        self.loop_freq = loop_freq
        
        self.total_pixels_per_frame = width * height
        self.total_keyframes = int(self.total_frames * keyframe_ratio)
        self.total_loop_checks = int(duration_sec * loop_freq)

    def analyze_commit_code(self, module_sources: Dict[str, str]) -> Dict[str, Any]:
        """
        对提取到的单次 Commit 的 C++ 源码进行静态特征提取和理论工作量估算
        """
        orb_ext_src = module_sources.get("ORBextractor", "")
        orb_match_src = module_sources.get("ORBmatcher", "")
        tracking_src = module_sources.get("Tracking", "")
        opt_src = module_sources.get("Optimizer", "")
        local_map_src = module_sources.get("LocalMapping", "")
        loop_src = module_sources.get("LoopClosing", "")
        matrix_src = module_sources.get("Matrix", "")
        native_src = module_sources.get("native-lib", "")

        all_code = "".join(module_sources.values())
        if len(all_code.strip()) == 0 or len(orb_ext_src.strip()) == 0:
            return {
                "total_ops": 0,
                "feature_extraction_ops": 0,
                "feature_matching_ops": 0,
                "tracking_optimization_ops": 0,
                "mapping_loop_ops": 0,
                "misc_overhead_ops": 0,
                "ops_per_frame": 0,
                "optimizations_detected": [],
                "total_code_lines": 0,
                "complexity_score": 0.0,
                "is_valid_slam": False
            }

        # ----------------------------------------------------------------------
        # 静态优化特征检测 (Optimization Signatures)
        # ----------------------------------------------------------------------
        optimizations_detected = []

        # (1) 描述子 LUT 查表优化 (消除 sin/cos 与动态乘法)
        has_descriptor_lut = bool(re.search(r"descriptorOffsetLUT|bDescriptorLUTInit", orb_ext_src))
        has_level_step_lut = bool(re.search(r"levelStepOffsetLUT|PrepareLevelStepOffsetLUT", orb_ext_src))
        if has_descriptor_lut:
            optimizations_detected.append("Descriptor LUT (Sin/Cos Eliminated)")
        if has_level_step_lut:
            optimizations_detected.append("Direct Level-Step Offset LUT")

        # (2) 汉明距离硬件优化 (__builtin_popcount / _mm_popcnt / bitwise LUT)
        has_popcount_opt = bool(re.search(r"__builtin_popcount|_mm_popcnt|popcount_lut|PopCount", orb_match_src + orb_ext_src))
        if has_popcount_opt:
            optimizations_detected.append("Hardware Popcount Matching")

        # (3) 锁优化与内存复用 (Lock Splitting, Buffer Reuse)
        has_lock_opt = bool(re.search(r"锁优化|内存复用|unique_lock|shared_mutex|thread_local|buffer_pool|cachedStep", all_code, re.I))
        if has_lock_opt:
            optimizations_detected.append("Lock Optimization & Memory Reuse")

        # (4) 矩阵与变换优化
        has_matrix_opt = bool(re.search(r"平移向量|齐次矩阵|fast_matrix|inline.*transform", all_code, re.I) or (len(matrix_src) > 500 and "Matrix" in module_sources))
        if has_matrix_opt:
            optimizations_detected.append("Fast Matrix Operations")

        # (5) 线程池 / 并行化 / NEON 指令
        has_parallel_opt = bool(re.search(r"setNumThreads|parallel_for|ThreadPool|std::async|OpenMP|arm_neon|__ARM_NEON", all_code, re.I))
        if has_parallel_opt:
            optimizations_detected.append("Multithreading / SIMD Acceleration")

        # ----------------------------------------------------------------------
        # 算力模型：子模块理论计算量测算
        # ----------------------------------------------------------------------
        # 1. 特征提取模块
        pyr_pixel_multiplier = 1.6
        total_pyr_pixels_per_frame = self.total_pixels_per_frame * pyr_pixel_multiplier

        blur_ops_per_pixel = 10.0
        blur_factor = 0.75 if ("cv::GaussianBlur" in orb_ext_src or "filter2D" in orb_ext_src) else 1.0
        blur_total_ops = self.total_frames * (total_pyr_pixels_per_frame * blur_ops_per_pixel * blur_factor)

        fast_ops_per_pixel = 8.0
        fast_total_ops = self.total_frames * (total_pyr_pixels_per_frame * fast_ops_per_pixel)

        ic_angle_ops_per_kp = 700 * 3 + 60
        ic_angle_total_ops = self.total_frames * (self.n_features * ic_angle_ops_per_kp)

        if has_level_step_lut:
            desc_ops_per_kp = 280.0
        elif has_descriptor_lut:
            desc_ops_per_kp = 650.0
        else:
            desc_ops_per_kp = 3600.0

        brief_total_ops = self.total_frames * (self.n_features * desc_ops_per_kp)
        feature_extraction_ops = blur_total_ops + fast_total_ops + ic_angle_total_ops + brief_total_ops

        # 2. 特征匹配模块
        matching_pairs_per_frame = self.n_features * 60
        if has_popcount_opt:
            hamming_ops_per_pair = 12.0
        else:
            if "popcount" in orb_match_src.lower():
                hamming_ops_per_pair = 48.0
            else:
                hamming_ops_per_pair = 680.0

        feature_matching_ops = self.total_frames * (matching_pairs_per_frame * hamming_ops_per_pair)

        # 3. 追踪与位姿优化模块
        pose_opt_ops_per_iter = (self.n_features / 1000.0 * 300) * (30 + 36 + 6) + 216
        pose_opt_ops_per_frame = 4 * pose_opt_ops_per_iter
        tracking_logic_ops_per_frame = 150000.0
        
        matrix_multiplier = 0.7 if has_matrix_opt else 1.2
        tracking_optimization_ops = self.total_frames * (pose_opt_ops_per_frame + tracking_logic_ops_per_frame) * matrix_multiplier

        # 4. 局部建图与回环检测
        local_ba_ops_per_kf = 6500000.0 * (self.n_features / 1000.0)
        local_mapping_ops = self.total_keyframes * local_ba_ops_per_kf

        loop_check_ops_per_event = 1800000.0 * (self.n_features / 1000.0)
        loop_closing_ops = self.total_loop_checks * loop_check_ops_per_event
        mapping_loop_ops = local_mapping_ops + loop_closing_ops

        # 5. 代码级架构与开销因子
        overhead_multiplier = 1.0
        if not has_lock_opt:
            overhead_multiplier += 0.18
        if not has_parallel_opt:
            overhead_multiplier += 0.05
        
        total_code_lines = sum(len(src.splitlines()) for src in module_sources.values())
        misc_overhead_ops = (feature_extraction_ops + feature_matching_ops + tracking_optimization_ops) * (overhead_multiplier - 1.0)

        # 汇总
        total_ops = feature_extraction_ops + feature_matching_ops + tracking_optimization_ops + mapping_loop_ops + misc_overhead_ops
        ops_per_frame = total_ops / self.total_frames if self.total_frames > 0 else 0

        return {
            "total_ops": round(total_ops),
            "feature_extraction_ops": round(feature_extraction_ops),
            "feature_matching_ops": round(feature_matching_ops),
            "tracking_optimization_ops": round(tracking_optimization_ops),
            "mapping_loop_ops": round(mapping_loop_ops),
            "misc_overhead_ops": round(misc_overhead_ops),
            "ops_per_frame": round(ops_per_frame),
            "optimizations_detected": optimizations_detected,
            "total_code_lines": total_code_lines,
            "complexity_score": round(total_ops / 1e9, 4), # G-Ops
            "is_valid_slam": True
        }


# ==============================================================================
# 3. Git Commit 历史提取引擎 (Git History Engine - 支持多线程并行)
# ==============================================================================
class GitCommitEngine:
    """
    高效提取 Git 仓库历史 Commit 信息及源码，支持并行拉取
    """

    def __init__(self, repo_dir: str = "."):
        self.repo_dir = os.path.abspath(repo_dir)

    def get_commit_list(self, rev_range: Optional[str] = None, max_count: Optional[int] = None) -> List[Dict[str, str]]:
        """
        获取 commit 列表，按时间顺序从最早到最新 (reverse order)
        """
        cmd = ["git", "log", "--reverse", "--format=%H|%h|%an|%ad|%s", "--date=short"]
        if rev_range:
            cmd.append(rev_range)
        elif max_count:
            cmd = ["git", "log", f"-n{max_count}", "--format=%H|%h|%an|%ad|%s", "--date=short"]

        result = subprocess.run(cmd, cwd=self.repo_dir, capture_output=True, text=True, encoding="utf-8", errors="ignore")
        if result.returncode != 0:
            raise RuntimeError(f"Git log failed: {result.stderr}")

        commits = []
        lines = result.stdout.strip().splitlines()
        if max_count and not rev_range:
            lines = list(reversed(lines))

        for line in lines:
            parts = line.split("|", 4)
            if len(parts) >= 5:
                commits.append({
                    "hash": parts[0].strip(),
                    "short_hash": parts[1].strip(),
                    "author": parts[2].strip(),
                    "date": parts[3].strip(),
                    "subject": parts[4].strip()
                })
        return commits

    def get_commit_modules(self, commit_hash: str) -> Dict[str, str]:
        """
        动态查找并读取该 commit 中的关键 SLAM 算法模块源文件内容
        """
        cmd = ["git", "ls-tree", "-r", "--name-only", commit_hash]
        res = subprocess.run(cmd, cwd=self.repo_dir, capture_output=True, text=True, encoding="utf-8", errors="ignore")
        if res.returncode != 0:
            return {}

        all_files = res.stdout.strip().splitlines()
        module_sources = {}

        for mod_name, patterns in KEY_FILE_PATTERNS.items():
            matched_file = None
            for p in patterns:
                clean_p = p.replace("*", "")
                for f in all_files:
                    if f.endswith(clean_p):
                        matched_file = f
                        break
                if matched_file:
                    break

            if matched_file:
                show_cmd = ["git", "show", f"{commit_hash}:{matched_file}"]
                s_res = subprocess.run(show_cmd, cwd=self.repo_dir, capture_output=True, text=True, encoding="utf-8", errors="ignore")
                if s_res.returncode == 0:
                    module_sources[mod_name] = s_res.stdout

        return module_sources


# ==============================================================================
# 4. 可视化与排版优化报告生成器 (Visualizer & Layout Optimizer)
# ==============================================================================
class BenchmarkVisualizer:
    """
    生成专业排版 Matplotlib 图表 (彻底解决文字遮挡/堆叠问题) 与交互式 HTML 报告
    """

    @staticmethod
    def format_ops_human(val: float) -> str:
        """智能化人性化计算量单位显示 (G-Ops, M-Ops, K-Ops)"""
        if val >= 1e9:
            return f"{val / 1e9:.2f} G-Ops"
        elif val >= 1e6:
            return f"{val / 1e6:.2f} M-Ops"
        elif val >= 1e3:
            return f"{val / 1e3:.2f} K-Ops"
        return f"{val:.0f} Ops"

    @staticmethod
    def generate_png_plot(benchmark_results: List[Dict[str, Any]],
                          output_png: str,
                          stress_matrix_data: Optional[Dict[str, Dict[str, Any]]] = None):
        """
        生成高清 Matplotlib 排版优化图表 (自动扩充头部留白，防文字重叠/堆叠)
        """
        if not HAS_MATPLOTLIB or not benchmark_results:
            print("[-] Matplotlib 未合规渲染或结果为空，跳过 PNG 图表生成。")
            return

        valid_results = [r for r in benchmark_results if r.get("is_valid_slam", True) and r["total_ops"] > 0]
        if not valid_results:
            valid_results = benchmark_results

        first_valid_ops = valid_results[0]["total_ops"]
        latest_ops = benchmark_results[-1]["total_ops"]

        indices = list(range(1, len(benchmark_results) + 1))
        # 统一转为 G-Ops 绘制，便于大算力场景阅读
        total_ops_g = [r["total_ops"] / 1e9 for r in benchmark_results]
        feat_ext_g = [r["feature_extraction_ops"] / 1e9 for r in benchmark_results]
        feat_match_g = [r["feature_matching_ops"] / 1e9 for r in benchmark_results]
        tracking_g = [r["tracking_optimization_ops"] / 1e9 for r in benchmark_results]
        mapping_g = [r["mapping_loop_ops"] / 1e9 for r in benchmark_results]
        overhead_g = [r["misc_overhead_ops"] / 1e9 for r in benchmark_results]

        valid_ops_list = [r["total_ops"] / 1e9 for r in valid_results]
        max_ops_g = max(valid_ops_list) if valid_ops_list else 1.0
        min_ops_g = min(valid_ops_list) if valid_ops_list else 1.0

        max_orig_idx = [i for i, r in enumerate(benchmark_results) if r["total_ops"]/1e9 == max_ops_g][0] + 1
        min_orig_idx = [i for i, r in enumerate(benchmark_results) if r["total_ops"]/1e9 == min_ops_g][0] + 1
        latest_orig_idx = len(benchmark_results)

        reduction_pct = ((max_ops_g - (latest_ops/1e9)) / max_ops_g * 100) if max_ops_g > 0 else 0

        # 判断是否需要 4 子图 (如果有矩阵压力测试数据)
        num_subplots = 4 if stress_matrix_data else 3
        fig_height = 18 if num_subplots == 4 else 15

        plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
        fig, axes = plt.subplots(num_subplots, 1, figsize=(14, fig_height), dpi=150)
        
        # ----------------------------------------------------------------------
        # 子图 1: 1分钟视频总计算量演变趋势
        # ----------------------------------------------------------------------
        ax1 = axes[0]
        ax1.plot(indices, total_ops_g, color='#2563EB', linewidth=2.4, label='Total Workload (G-Ops)')
        ax1.fill_between(indices, 0, total_ops_g, color='#3B82F6', alpha=0.12)
        
        # 标记峰值点、最佳点、最新点
        ax1.scatter([max_orig_idx], [max_ops_g], color='#EF4444', s=70, zorder=6, 
                    label=f'Peak: {BenchmarkVisualizer.format_ops_human(max_ops_g * 1e9)} ({benchmark_results[max_orig_idx-1]["short_hash"]})')
        ax1.scatter([min_orig_idx], [min_ops_g], color='#10B981', s=70, zorder=6, 
                    label=f'Best: {BenchmarkVisualizer.format_ops_human(min_ops_g * 1e9)} ({benchmark_results[min_orig_idx-1]["short_hash"]})')
        ax1.scatter([latest_orig_idx], [latest_ops/1e9], color='#8B5CF6', s=70, zorder=6, 
                    label=f'Latest: {BenchmarkVisualizer.format_ops_human(latest_ops)} ({benchmark_results[-1]["short_hash"]})')

        # 防文字堆叠：顶部预留 25% 的 headroom 空间
        ax1.set_ylim(0, max_ops_g * 1.25)
        ax1.xaxis.set_major_locator(ticker.MaxNLocator(nbins=12, integer=True))

        ax1.set_title(f"SLAM Workload Evolution across Git Commits\n"
                      f"Peak: {BenchmarkVisualizer.format_ops_human(max_ops_g * 1e9)}  ->  Latest: {BenchmarkVisualizer.format_ops_human(latest_ops)}  "
                      f"(Reduction: -{reduction_pct:.1f}%)", fontsize=12, fontweight='bold', pad=12)
        ax1.set_xlabel("Commit Sequence (Chronological Order)", fontsize=10)
        ax1.set_ylabel("Workload (Giga-Ops / 1 min)", fontsize=10)
        ax1.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9, edgecolor='#cbd5e1')
        ax1.grid(True, linestyle='--', alpha=0.5)

        # ----------------------------------------------------------------------
        # 子图 2: 多模块堆叠面积图 (Sub-system Workload Breakdown)
        # ----------------------------------------------------------------------
        ax2 = axes[1]
        ax2.stackplot(indices,
                      feat_ext_g, feat_match_g, tracking_g, mapping_g, overhead_g,
                      labels=['Feature Extraction', 'Feature Matching', 'Tracking & Pose Opt', 'Mapping & Loop', 'Overhead Factor'],
                      colors=['#3B82F6', '#F59E0B', '#10B981', '#EC4899', '#6B7280'],
                      alpha=0.85)
        ax2.set_ylim(0, max_ops_g * 1.25)
        ax2.xaxis.set_major_locator(ticker.MaxNLocator(nbins=12, integer=True))

        ax2.set_title("Sub-system Workload Breakdown (Stacked Area)", fontsize=12, fontweight='bold', pad=12)
        ax2.set_xlabel("Commit Sequence", fontsize=10)
        ax2.set_ylabel("Workload (Giga-Ops / 1 min)", fontsize=10)
        ax2.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9, edgecolor='#cbd5e1')
        ax2.grid(True, linestyle='--', alpha=0.5)

        # ----------------------------------------------------------------------
        # 子图 3: 关键节点对比柱状图 (Milestone Comparison - 彻底修复顶部文字碰撞)
        # ----------------------------------------------------------------------
        ax3 = axes[2]
        compare_labels = [
            f"Initial Base\n({valid_results[0]['short_hash']})",
            f"Peak Workload\n({benchmark_results[max_orig_idx-1]['short_hash']})",
            f"Latest Commit\n({benchmark_results[-1]['short_hash']})"
        ]
        compare_values = [first_valid_ops / 1e9, max_ops_g, latest_ops / 1e9]
        colors = ['#64748B', '#EF4444', '#10B981']
        bars = ax3.bar(compare_labels, compare_values, color=colors, width=0.42, edgecolor='#1e293b', linewidth=0.9)

        # ★ 关键修正：扩充 30% Y 轴顶部留白，防止标注文字遮挡 Subplot 标题
        max_bar_val = max(compare_values) if compare_values else 1.0
        ax3.set_ylim(0, max_bar_val * 1.30)

        for bar in bars:
            height = bar.get_height()
            pct_text = f"{(height / max_ops_g * 100):.1f}%" if max_ops_g > 0 else "100%"
            human_val = BenchmarkVisualizer.format_ops_human(height * 1e9)
            # 文字安全摆放于柱体上方 6pt 处，绝不与 Subplot Title 重叠
            ax3.annotate(f"{human_val}\n({pct_text})",
                         xy=(bar.get_x() + bar.get_width() / 2, height),
                         xytext=(0, 6), textcoords="offset points",
                         ha='center', va='bottom', fontsize=10, fontweight='bold',
                         bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.8))

        ax3.set_title("Milestone Workload Comparison (Initial vs Peak vs Latest)", fontsize=12, fontweight='bold', pad=16)
        ax3.set_ylabel("Workload (Giga-Ops / 1 min)", fontsize=10)
        ax3.grid(True, linestyle='--', alpha=0.5, axis='y')

        # ----------------------------------------------------------------------
        # 子图 4 (可选): 多场景压力测试对比柱状图 (Stress Testing Matrix)
        # ----------------------------------------------------------------------
        if stress_matrix_data:
            ax4 = axes[3]
            preset_keys = list(stress_matrix_data.keys())
            preset_display_names = {
                "mobile_low": "Mobile Low\n(QVGA 30fps)",
                "standard": "Standard\n(VGA 30fps)",
                "hd_1080p": "HD Performance\n(1080P 60fps)",
                "extreme_4k": "Extreme Stress\n(4K 60fps)",
                "heavy_loop": "Heavy Loop\n(10Hz Loop)"
            }
            preset_names = [preset_display_names.get(k, k) for k in preset_keys]
            stress_ops_g = [stress_matrix_data[k]["total_ops"] / 1e9 for k in preset_keys]

            stress_colors = ['#0EA5E9', '#3B82F6', '#8B5CF6', '#EC4899', '#EF4444']
            bars4 = ax4.bar(preset_names, stress_ops_g, color=stress_colors[:len(preset_keys)], width=0.45, edgecolor='#1e293b', linewidth=0.9)
            
            max_stress_g = max(stress_ops_g) if stress_ops_g else 1.0
            ax4.set_ylim(0, max_stress_g * 1.30)

            for bar in bars4:
                h = bar.get_height()
                h_human = BenchmarkVisualizer.format_ops_human(h * 1e9)
                ax4.annotate(h_human,
                             xy=(bar.get_x() + bar.get_width() / 2, h),
                             xytext=(0, 6), textcoords="offset points",
                             ha='center', va='bottom', fontsize=9, fontweight='bold',
                             bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.8))

            ax4.set_title("Stress Testing Matrix: Workload across Profiles (Latest Commit)", fontsize=12, fontweight='bold', pad=16)
            ax4.set_ylabel("Workload (Giga-Ops / 1 min)", fontsize=10)
            ax4.grid(True, linestyle='--', alpha=0.5, axis='y')

        plt.tight_layout(pad=2.5)
        plt.savefig(output_png, dpi=200, bbox_inches='tight')
        plt.close()
        print(f"[+] 排版优化后的高清 PNG 图表已保存至: {output_png}")

    @staticmethod
    def generate_html_report(benchmark_results: List[Dict[str, Any]],
                              output_html: str,
                              video_params: Dict[str, Any],
                              stress_matrix_data: Optional[Dict[str, Dict[str, Any]]] = None):
        """
        生成现代响应式 HTML 报告 (含压力测试矩阵与排版优化)
        """
        if not benchmark_results:
            return

        valid_results = [r for r in benchmark_results if r.get("is_valid_slam", True) and r["total_ops"] > 0]
        if not valid_results:
            valid_results = benchmark_results

        first_res = valid_results[0]
        latest_res = benchmark_results[-1]
        
        first_ops = first_res["total_ops"]
        latest_ops = latest_res["total_ops"]
        max_res = max(valid_results, key=lambda x: x["total_ops"])

        peak_ops = max_res["total_ops"]
        reduction_pct = ((peak_ops - latest_ops) / peak_ops * 100) if peak_ops > 0 else 0
        speedup_ratio = (peak_ops / latest_ops) if latest_ops > 0 else 1.0

        # Top 优化 commit 提取
        opt_diffs = []
        for i in range(1, len(benchmark_results)):
            prev = benchmark_results[i - 1]["total_ops"]
            curr = benchmark_results[i]["total_ops"]
            diff = prev - curr
            if diff > 0 and prev > 0:
                opt_diffs.append({
                    "commit": benchmark_results[i],
                    "reduced_ops": diff,
                    "reduced_pct": (diff / prev * 100)
                })
        opt_diffs.sort(key=lambda x: x["reduced_ops"], reverse=True)
        top_optimizations = opt_diffs[:6]

        # 构造 Chart.js 数据
        labels_json = json.dumps([f"#{i+1} {r['short_hash']}" for i, r in enumerate(benchmark_results)], ensure_ascii=False)
        total_ops_json = json.dumps([round(r["total_ops"] / 1e9, 3) for r in benchmark_results]) # G-Ops
        feat_ext_json = json.dumps([round(r["feature_extraction_ops"] / 1e9, 3) for r in benchmark_results])
        feat_match_json = json.dumps([round(r["feature_matching_ops"] / 1e9, 3) for r in benchmark_results])
        tracking_json = json.dumps([round(r["tracking_optimization_ops"] / 1e9, 3) for r in benchmark_results])
        mapping_json = json.dumps([round(r["mapping_loop_ops"] / 1e9, 3) for r in benchmark_results])
        overhead_json = json.dumps([round(r["misc_overhead_ops"] / 1e9, 3) for r in benchmark_results])

        # 压力测试矩阵 HTML 卡片生成
        stress_cards_html = ""
        if stress_matrix_data:
            for p_key, p_data in stress_matrix_data.items():
                ops_human = BenchmarkVisualizer.format_ops_human(p_data['total_ops'])
                ops_pf_human = BenchmarkVisualizer.format_ops_human(p_data['ops_per_frame'])
                stress_cards_html += f"""
                <div class="stress-card">
                    <div class="stress-title">{p_data['name']}</div>
                    <div class="stress-desc">{p_data['description']}</div>
                    <div class="stress-val">{ops_human}</div>
                    <div class="stress-sub">单帧负载: {ops_pf_human} / 帧</div>
                    <div class="stress-tag">预设: <code>{p_key}</code> ({p_data['width']}x{p_data['height']} @ {p_data['fps']}FPS)</div>
                </div>
                """

        table_rows_html = ""
        for i, r in enumerate(reversed(benchmark_results)):
            idx = len(benchmark_results) - i
            opts_badges = "".join([f'<span class="badge badge-opt">{opt}</span>' for opt in r.get("optimizations_detected", [])])
            if not opts_badges:
                opts_badges = '<span class="badge badge-none">Baseline</span>' if r['total_ops'] > 0 else '<span class="badge badge-none">Non-Code</span>'
            
            table_rows_html += f"""
            <tr>
                <td class="font-mono text-gray-500">#{idx}</td>
                <td><code class="commit-hash">{r['short_hash']}</code></td>
                <td class="font-semibold text-gray-800">{r['subject']}</td>
                <td class="text-sm text-gray-500">{r['author']}</td>
                <td class="text-sm text-gray-500">{r['date']}</td>
                <td class="font-mono font-bold text-blue-600">{BenchmarkVisualizer.format_ops_human(r['total_ops'])}</td>
                <td class="font-mono text-xs text-gray-600">{BenchmarkVisualizer.format_ops_human(r['ops_per_frame'])}</td>
                <td>{opts_badges}</td>
            </tr>
            """

        top_opts_html = ""
        for item in top_optimizations:
            c = item["commit"]
            top_opts_html += f"""
            <div class="top-opt-card">
                <div class="flex justify-between items-start">
                    <div>
                        <span class="commit-hash">{c['short_hash']}</span>
                        <strong class="ml-2 text-gray-900">{c['subject']}</strong>
                        <div class="text-xs text-gray-500 mt-1">{c['author']} · {c['date']}</div>
                    </div>
                    <div class="text-right">
                        <div class="text-emerald-600 font-bold font-mono">-{BenchmarkVisualizer.format_ops_human(item['reduced_ops'])}</div>
                        <div class="text-xs text-emerald-500 font-semibold">▼ {item['reduced_pct']:.1f}% 算力降幅</div>
                    </div>
                </div>
            </div>
            """

        html_content = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SLAM 理论计算量 Benchmark & 压力测试报告</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        :root {{
            --bg-main: #f8fafc;
            --card-bg: #ffffff;
            --primary: #2563eb;
            --success: #10b981;
            --warning: #f59e0b;
            --danger: #ef4444;
            --text-main: #1e293b;
            --text-muted: #64748b;
            --border-color: #e2e8f0;
        }}
        * {{ box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif; }}
        body {{ background-color: var(--bg-main); color: var(--text-main); line-height: 1.5; padding: 24px; }}
        .container {{ max-width: 1350px; margin: 0 auto; }}
        .header {{ margin-bottom: 24px; display: flex; justify-content: space-between; align-items: flex-end; border-bottom: 2px solid var(--border-color); padding-bottom: 16px; }}
        .header h1 {{ font-size: 26px; font-weight: 800; color: #0f172a; }}
        .header p {{ color: var(--text-muted); font-size: 14px; margin-top: 4px; }}
        .meta-tag {{ background: #e0e7ff; color: #3730a3; padding: 4px 12px; border-radius: 6px; font-size: 12px; font-weight: 600; }}
        
        .grid-stats {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 16px; margin-bottom: 24px; }}
        .stat-card {{ background: var(--card-bg); padding: 20px; border-radius: 12px; box-shadow: 0 1px 3px rgba(0,0,0,0.05); border: 1px solid var(--border-color); }}
        .stat-card .label {{ font-size: 13px; font-weight: 600; color: var(--text-muted); text-transform: uppercase; letter-spacing: 0.5px; }}
        .stat-card .value {{ font-size: 26px; font-weight: 800; margin: 8px 0; font-family: ui-monospace, monospace; }}
        .stat-card .sub {{ font-size: 12px; color: var(--text-muted); }}

        .stress-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 16px; margin-top: 14px; }}
        .stress-card {{ background: #f0f9ff; border: 1px solid #bae6fd; border-radius: 10px; padding: 16px; }}
        .stress-title {{ font-weight: 700; color: #0369a1; font-size: 15px; }}
        .stress-desc {{ font-size: 12px; color: #0284c7; margin-top: 2px; margin-bottom: 8px; }}
        .stress-val {{ font-size: 22px; font-weight: 800; color: #0284c7; font-family: monospace; }}
        .stress-sub {{ font-size: 12px; color: #64748b; margin-top: 4px; }}
        .stress-tag {{ margin-top: 8px; font-size: 11px; color: #0369a1; background: #e0f2fe; padding: 2px 6px; border-radius: 4px; display: inline-block; }}

        .card {{ background: var(--card-bg); padding: 24px; border-radius: 12px; box-shadow: 0 1px 3px rgba(0,0,0,0.05); border: 1px solid var(--border-color); margin-bottom: 24px; }}
        .card-title {{ font-size: 18px; font-weight: 700; color: #0f172a; margin-bottom: 16px; display: flex; justify-content: space-between; align-items: center; }}
        
        .chart-container {{ position: relative; width: 100%; height: 380px; }}

        .top-opt-card {{ background: #f0fdf4; border: 1px solid #bbf7d0; border-radius: 8px; padding: 14px 18px; margin-bottom: 12px; }}
        .commit-hash {{ background: #f1f5f9; color: #334155; padding: 2px 6px; border-radius: 4px; font-family: monospace; font-size: 12px; font-weight: 600; }}
        .badge {{ display: inline-block; padding: 2px 8px; border-radius: 4px; font-size: 11px; font-weight: 600; margin-right: 4px; }}
        .badge-opt {{ background: #dbeafe; color: #1e40af; border: 1px solid #bfdbfe; }}
        .badge-none {{ background: #f1f5f9; color: #64748b; }}

        table {{ width: 100%; border-collapse: collapse; text-align: left; font-size: 13px; }}
        th {{ background: #f8fafc; padding: 12px 14px; font-weight: 700; color: #475569; border-bottom: 2px solid var(--border-color); }}
        td {{ padding: 12px 14px; border-bottom: 1px solid var(--border-color); vertical-align: middle; }}
        tr:hover td {{ background-color: #f8fafc; }}

        .flex {{ display: flex; }}
        .justify-between {{ justify-content: space-between; }}
        .items-start {{ align-items: flex-start; }}
        .ml-2 {{ margin-left: 8px; }}
        .mt-1 {{ margin-top: 4px; }}
        .font-mono {{ font-family: ui-monospace, monospace; }}
        .font-bold {{ font-weight: 700; }}
        .font-semibold {{ font-weight: 600; }}
        .text-blue-600 {{ color: var(--primary); }}
        .text-emerald-600 {{ color: var(--success); }}
        .text-emerald-500 {{ color: #10b981; }}
        .text-gray-500 {{ color: #64748b; }}
        .text-gray-600 {{ color: #475569; }}
        .text-gray-800 {{ color: #1e293b; }}
        .text-gray-900 {{ color: #0f172a; }}
        .text-sm {{ font-size: 13px; }}
        .text-xs {{ font-size: 12px; }}
        .text-right {{ text-align: right; }}
    </style>
</head>
<body>
    <div class="container">
        <!-- Header -->
        <div class="header">
            <div>
                <h1>📊 SLAM 理论计算量 Benchmark & 压力测试报告</h1>
                <p>基准设定: 1 分钟 ({video_params['duration_sec']}s) | {video_params['fps']} FPS (总计 {video_params['total_frames']} 帧) | 分辨率 {video_params['width']}x{video_params['height']}</p>
            </div>
            <div>
                <span class="meta-tag">分析 Git Commits: {len(benchmark_results)} 次</span>
            </div>
        </div>

        <!-- 统计核心卡片 -->
        <div class="grid-stats">
            <div class="stat-card">
                <div class="label">基准峰值计算量 (Peak)</div>
                <div class="value text-gray-800">{BenchmarkVisualizer.format_ops_human(peak_ops)}</div>
                <div class="sub">Commit: <code>{max_res['short_hash']}</code></div>
            </div>
            <div class="stat-card">
                <div class="label">当前最新计算量 (Latest)</div>
                <div class="value text-blue-600">{BenchmarkVisualizer.format_ops_human(latest_ops)}</div>
                <div class="sub">Commit: <code>{latest_res['short_hash']}</code></div>
            </div>
            <div class="stat-card">
                <div class="label">理论计算量总降幅</div>
                <div class="value text-emerald-600">▼ {reduction_pct:.1f}%</div>
                <div class="sub">理论加速比: <strong>{speedup_ratio:.2f}x</strong></div>
            </div>
            <div class="stat-card">
                <div class="label">单帧平均理论负载</div>
                <div class="value text-gray-800">{BenchmarkVisualizer.format_ops_human(latest_res['ops_per_frame'])}</div>
                <div class="sub">1 分钟总操作: {BenchmarkVisualizer.format_ops_human(latest_ops)}</div>
            </div>
        </div>

        <!-- 压力测试矩阵卡片 -->
        {'<div class="card"><div class="card-title">⚡ 场景压力测试矩阵对比 (Multi-Scenario Stress Matrix - 最新 Commit)</div><div class="stress-grid">' + stress_cards_html + '</div></div>' if stress_cards_html else ''}

        <!-- 趋势图表 -->
        <div class="card">
            <div class="card-title">
                <span>📈 全量 Git Commit 理论计算量演变走势 (G-Ops)</span>
            </div>
            <div class="chart-container">
                <canvas id="trendChart"></canvas>
            </div>
        </div>

        <!-- 模块堆叠分解图 -->
        <div class="card">
            <div class="card-title">
                <span>🧩 各子系统计算量构成堆叠分析 (Sub-system Workload)</span>
            </div>
            <div class="chart-container">
                <canvas id="stackedChart"></canvas>
            </div>
        </div>

        <!-- Top 优化贡献 Commit -->
        <div class="card">
            <div class="card-title">
                <span>🚀 历史最大算力优化 Commit (Top Optimization Milestones)</span>
            </div>
            {top_opts_html if top_opts_html else '<p class="text-gray-500">暂无显著单次大幅优化记录</p>'}
        </div>

        <!-- 完整 Commit 详细表格 -->
        <div class="card">
            <div class="card-title">
                <span>📋 全量 Commit 理论计算量明细表</span>
            </div>
            <div style="overflow-x: auto; max-height: 500px; overflow-y: auto;">
                <table>
                    <thead>
                        <tr>
                            <th>序号</th>
                            <th>Hash</th>
                            <th>提交信息 (Subject)</th>
                            <th>作者</th>
                            <th>日期</th>
                            <th>1分钟总计算量</th>
                            <th>单帧计算量</th>
                            <th>识别到的优化项</th>
                        </tr>
                    </thead>
                    <tbody>
                        {table_rows_html}
                    </tbody>
                </table>
            </div>
        </div>
    </div>

    <script>
        const labels = {labels_json};
        const totalOps = {total_ops_json};
        const featExt = {feat_ext_json};
        const featMatch = {feat_match_json};
        const tracking = {tracking_json};
        const mapping = {mapping_json};
        const overhead = {overhead_json};

        // 1. 趋势折线图
        new Chart(document.getElementById('trendChart'), {{
            type: 'line',
            data: {{
                labels: labels,
                datasets: [{{
                    label: '1分钟总计算量 (G-Ops)',
                    data: totalOps,
                    borderColor: '#2563eb',
                    backgroundColor: 'rgba(37, 99, 235, 0.12)',
                    borderWidth: 2,
                    pointRadius: labels.length > 50 ? 1 : 3,
                    fill: true,
                    tension: 0.2
                }}]
            }},
            options: {{
                responsive: true,
                maintainAspectRatio: false,
                plugins: {{
                    tooltip: {{ mode: 'index', intersect: false }}
                }},
                scales: {{
                    y: {{
                        title: {{ display: true, text: 'Giga Operations (G-Ops)' }},
                        beginAtZero: true
                    }},
                    x: {{ ticks: {{ maxTicksLimit: 20 }} }}
                }}
            }}
        }});

        // 2. 堆叠面积图
        new Chart(document.getElementById('stackedChart'), {{
            type: 'line',
            data: {{
                labels: labels,
                datasets: [
                    {{ label: '特征提取 (Feature Extraction)', data: featExt, backgroundColor: '#3b82f6', borderColor: '#2563eb', fill: true }},
                    {{ label: '特征匹配 (Feature Matching)', data: featMatch, backgroundColor: '#f59e0b', borderColor: '#d97706', fill: true }},
                    {{ label: '位姿优化 (Tracking & Pose Opt)', data: tracking, backgroundColor: '#10b981', borderColor: '#059669', fill: true }},
                    {{ label: '局部建图与回环 (Mapping & Loop)', data: mapping, backgroundColor: '#ec4899', borderColor: '#db2777', fill: true }},
                    {{ label: '开销惩罚 (Overhead Factor)', data: overhead, backgroundColor: '#64748b', borderColor: '#475569', fill: true }}
                ]
            }},
            options: {{
                responsive: true,
                maintainAspectRatio: false,
                plugins: {{
                    tooltip: {{ mode: 'index', intersect: false }}
                }},
                scales: {{
                    y: {{
                        stacked: true,
                        title: {{ display: true, text: 'Giga Operations (G-Ops)' }},
                        beginAtZero: true
                    }},
                    x: {{ ticks: {{ maxTicksLimit: 20 }} }}
                }}
            }}
        }});
    </script>
</body>
</html>
"""
        with open(output_html, "w", encoding="utf-8") as f:
            f.write(html_content)
        print(f"[+] 交互式 HTML 报告已保存至: {output_html}")


# ==============================================================================
# 5. 主程序与命令行调度入口 (Main Orchestrator)
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="SLAM Git 提交历史理论计算量 Benchmark 与压力测试工具"
    )
    parser.add_argument("--repo", type=str, default=".", help="Git 仓库目录路径 (默认当前目录)")
    parser.add_argument("--recent", type=int, default=None, help="仅测算最近 N 个 Commit")
    parser.add_argument("--range", type=str, default=None, help="指定 Commit 范围 (例如 HEAD~30..HEAD)")
    
    # 压力测试与场景预设
    parser.add_argument("--preset", type=str, choices=list(WORKLOAD_PRESETS.keys()), default=DEFAULT_PRESET,
                        help="场景压力测试预设: mobile_low, standard, hd_1080p, extreme_4k, heavy_loop")
    parser.add_argument("--stress-matrix", action="store_true", help="开启多场景联合压力测试矩阵测算")
    
    # 自定义维度参数
    parser.add_argument("--duration", type=int, default=None, help="视频基准时长 (秒)")
    parser.add_argument("--fps", type=int, default=None, help="视频基准帧率 (FPS)")
    parser.add_argument("--width", type=int, default=None, help="画面宽")
    parser.add_argument("--height", type=int, default=None, help="画面高")
    parser.add_argument("--n-features", type=int, default=None, help="每帧提取特征点数量")
    parser.add_argument("--keyframe-ratio", type=float, default=None, help="关键帧生成比例")
    parser.add_argument("--loop-freq", type=float, default=None, help="回环检测频率 (Hz)")

    # 缓存与性能
    parser.add_argument("--jobs", "-j", type=int, default=8, help="并行 Git 提取线程数 (默认 8)")
    parser.add_argument("--cache-file", type=str, default=".benchmark_cache.json", help="增量缓存文件路径")
    parser.add_argument("--no-cache", action="store_true", help="禁用缓存，全量重新计算")
    
    # 输出
    parser.add_argument("--output-png", type=str, default="benchmark_workload.png", help="PNG 图表输出文件名")
    parser.add_argument("--output-html", type=str, default="benchmark_report.html", help="HTML 报告输出文件名")

    args = parser.parse_args()

    # 应用预设或自定义参数
    preset_config = WORKLOAD_PRESETS.get(args.preset, WORKLOAD_PRESETS[DEFAULT_PRESET])
    duration = args.duration if args.duration is not None else preset_config["duration_sec"]
    fps = args.fps if args.fps is not None else preset_config["fps"]
    width = args.width if args.width is not None else preset_config["width"]
    height = args.height if args.height is not None else preset_config["height"]
    n_features = args.n_features if args.n_features is not None else preset_config["n_features"]
    keyframe_ratio = args.keyframe_ratio if args.keyframe_ratio is not None else preset_config["keyframe_ratio"]
    loop_freq = args.loop_freq if args.loop_freq is not None else preset_config["loop_freq"]

    print("=" * 70)
    print("🚀 SLAM Git 提交历史理论计算量 Benchmark 与压力测试")
    print("=" * 70)
    print(f"[*] 预设场景模式: {preset_config['name']}")
    print(f"[*] 算力基准配置: {duration} 秒 @ {fps} FPS ({duration * fps} 帧) | {width}x{height} | {n_features} 特征点/帧")
    print(f"[*] 并行提取线程: {args.jobs} 线程")

    # 1. 获取 Git 提交列表
    git_engine = GitCommitEngine(repo_dir=args.repo)
    try:
        commits = git_engine.get_commit_list(rev_range=args.range, max_count=args.recent)
    except Exception as e:
        print(f"[!] 获取 Git 提交失败: {e}")
        sys.exit(1)

    if not commits:
        print("[-] 未找到匹配的 Git 提交记录。")
        sys.exit(0)

    print(f"[*] 目标分析 Commit 数量: {len(commits)} 个")

    # 2. 读取缓存
    cache = {}
    if not args.no_cache and os.path.exists(args.cache_file):
        try:
            with open(args.cache_file, "r", encoding="utf-8") as f:
                cache = json.load(f)
            print(f"[*] 已载入本地缓存: {len(cache)} 条记录")
        except Exception as e:
            print(f"[!] 读取缓存失败，将重新计算: {e}")
            cache = {}

    # 3. 逐个 Commit 测算理论计算量 (支持多线程并行拉取 Git File Tree)
    workload_model = SLAMWorkloadModel(
        duration_sec=duration,
        fps=fps,
        width=width,
        height=height,
        n_features=n_features,
        keyframe_ratio=keyframe_ratio,
        loop_freq=loop_freq
    )

    start_time = time.time()
    results_map = {}
    missing_commits = []

    for c in commits:
        c_hash = c["hash"]
        cache_key = f"v3_{c_hash}_{duration}_{fps}_{width}_{height}_{n_features}_{keyframe_ratio}_{loop_freq}"
        if cache_key in cache:
            results_map[c_hash] = {**c, **cache[cache_key]}
        else:
            missing_commits.append((c, cache_key))

    cache_hits = len(commits) - len(missing_commits)

    if missing_commits:
        print(f"[*] 正在使用 {args.jobs} 线程并行解析 {len(missing_commits)} 个未缓存 Commit...")

        def process_single_commit(commit_info_tuple):
            c_info, c_key = commit_info_tuple
            m_sources = git_engine.get_commit_modules(c_info["hash"])
            analysis_res = workload_model.analyze_commit_code(m_sources)
            return c_info["hash"], c_key, analysis_res, c_info

        with ThreadPoolExecutor(max_workers=args.jobs) as executor:
            future_to_commit = {executor.submit(process_single_commit, mc): mc for mc in missing_commits}
            completed_count = 0
            for future in as_completed(future_to_commit):
                c_hash, c_key, analysis_res, c_info = future.result()
                cache[c_key] = analysis_res
                results_map[c_hash] = {**c_info, **analysis_res}
                completed_count += 1
                if completed_count % 50 == 0 or completed_count == len(missing_commits):
                    print(f"    进度: 并行解析 [{completed_count}/{len(missing_commits)}] Commit {c_info['short_hash']} -> 负载: {BenchmarkVisualizer.format_ops_human(analysis_res['total_ops'])}")

    # 按原本 commit 顺序排列结果
    results = [results_map[c["hash"]] for c in commits]

    elapsed = time.time() - start_time
    print(f"[+] 测算完成! 耗时: {elapsed:.2f}s (缓存命中: {cache_hits}/{len(commits)})")

    # 4. 保存缓存
    try:
        with open(args.cache_file, "w", encoding="utf-8") as f:
            json.dump(cache, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"[!] 保存缓存失败: {e}")

    # 5. 如果开启了 --stress-matrix，对最新 Commit 执行全场景压力测试对比
    stress_matrix_data = None
    if args.stress_matrix and results:
        latest_c_hash = results[-1]["hash"]
        latest_m_sources = git_engine.get_commit_modules(latest_c_hash)
        stress_matrix_data = {}
        
        print("\n[*] 正在运行场景压力测试矩阵 (Stress Matrix)...")
        for p_key, p_cfg in WORKLOAD_PRESETS.items():
            s_model = SLAMWorkloadModel(
                duration_sec=p_cfg["duration_sec"],
                fps=p_cfg["fps"],
                width=p_cfg["width"],
                height=p_cfg["height"],
                n_features=p_cfg["n_features"],
                keyframe_ratio=p_cfg["keyframe_ratio"],
                loop_freq=p_cfg["loop_freq"]
            )
            s_analysis = s_model.analyze_commit_code(latest_m_sources)
            stress_matrix_data[p_key] = {**p_cfg, **s_analysis}
            print(f"  • {p_cfg['name']}: {BenchmarkVisualizer.format_ops_human(s_analysis['total_ops'])} (单帧: {BenchmarkVisualizer.format_ops_human(s_analysis['ops_per_frame'])})")

    # 6. 生成图表与报告
    video_params = {
        "duration_sec": duration,
        "fps": fps,
        "total_frames": duration * fps,
        "width": width,
        "height": height
    }

    BenchmarkVisualizer.generate_png_plot(results, args.output_png, stress_matrix_data)
    BenchmarkVisualizer.generate_html_report(results, args.output_html, video_params, stress_matrix_data)

    # 7. 终端摘要对比打印
    valid_res = [r for r in results if r.get("is_valid_slam", True) and r["total_ops"] > 0]
    if valid_res:
        first_r = valid_res[0]
        latest_r = results[-1]
        max_r = max(valid_res, key=lambda x: x["total_ops"])

        peak_ops = max_r["total_ops"]
        latest_ops = latest_r["total_ops"]
        diff_ops = peak_ops - latest_ops
        diff_pct = (diff_ops / peak_ops * 100) if peak_ops > 0 else 0

        print("\n" + "=" * 70)
        print("📌 BENCHMARK & 压力测试 总结摘要")
        print("=" * 70)
        print(f"• 代码基线提交 ({first_r['short_hash']} - {first_r['date']}): {BenchmarkVisualizer.format_ops_human(first_r['total_ops'])}")
        print(f"• 历史峰值负载 ({max_r['short_hash']} - {max_r['date']}): {BenchmarkVisualizer.format_ops_human(peak_ops)}")
        print(f"• 当前最新提交 ({latest_r['short_hash']} - {latest_r['date']}): {BenchmarkVisualizer.format_ops_human(latest_ops)}")
        print(f"• 峰值到最新降幅: -{diff_pct:.2f}% (优化减免算力: {BenchmarkVisualizer.format_ops_human(diff_ops)})")
        print(f"• 单帧平均理论计算量: {BenchmarkVisualizer.format_ops_human(latest_r['ops_per_frame'])} / 帧")
        print(f"• 报告文件已生成:")
        print(f"    - 交互式 HTML: {os.path.abspath(args.output_html)}")
        print(f"    - 排版优化 PNG 图表: {os.path.abspath(args.output_png)}")
        print("=" * 70)


if __name__ == "__main__":
    main()
