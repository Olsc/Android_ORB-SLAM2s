# Workload Benchmark Tools (`workload_benchmark_tools`)

本目录包含用于对 Git 历史提交进行 Visual SLAM 理论计算量（FLOPs / Ops）分析与压力测试评估的基准工具。

---

## 🛠️ 脚本工具说明

### 1. `run_workload_benchmark.py` (Visual SLAM 理论计算量基准与压力测试引擎)

测算并分析代码在不同 Git Commit 演进过程中的理论计算复杂度（包括特征提取、配准、局部地图优化等模块），并自动绘制图形化对比图表。

#### ✨ 核心特性
- **算法计算量建模**：精准建模 ORB-SLAM2 关键环节（高斯金字塔构建、FAST角点检测、ORB描述子计算、重投影匹配、g2o BA 优化等）的 FLOPs 与算力开销。
- **多线程历史分析**：支持使用多线程 ThreadPool 快速并发分析多个 Git Commit 的变动开销。
- **多场景负载预测**：支持预设 720p 30fps / 1080p 60fps / 4K 60fps 等多种视频规格与特征点配置下的压力测试。
- **可视基准图表导出**：使用 Matplotlib 生成高质量折线图与柱状图对比结果。

#### 📦 环境要求

```bash
pip install matplotlib
```

#### 🚀 使用方法

```bash
# 1. 默认分析对比历史 Git 提交开销并输出图表
python docs/workload_benchmark_tools/run_workload_benchmark.py

# 2. 指定输出图表文件名
python docs/workload_benchmark_tools/run_workload_benchmark.py --output-png benchmark_results.png

# 3. 指定起始提交与线程数
python docs/workload_benchmark_tools/run_workload_benchmark.py --start-commit 355cdc1 --threads 8
```

#### 📋 参数说明
| 参数 | 长参数 | 默认值 | 描述 |
| :--- | :--- | :--- | :--- |
| `-s` | `--start-commit` | `None` | 指定评测起始 Commit Hash |
| `-e` | `--end-commit` | `HEAD` | 指定评测结束 Commit Hash |
| `-o` | `--output-png` | `benchmark_workload.png` | 指定输出 PNG 图表路径及文件名 |
| `-t` | `--threads` | `4` | 指定多线程分析并发数 |
