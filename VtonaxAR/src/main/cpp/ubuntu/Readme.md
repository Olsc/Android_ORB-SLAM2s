# VtonaxAR Ubuntu SLAM

### 📦 安装依赖指令 (Ubuntu 20.04)

```bash
# 更新系统源并安装基础构建链
sudo apt-get update
sudo apt-get install -y build-essential cmake
```

> [!NOTE]
> **依赖说明**：
> 本项目已将 OpenCV (`Thirdparty/opencv`) 源码包含在仓库中，构建时会自动作为子库打包编译 (`core`, `imgproc`, `features2d`, `flann`, `calib3d` 静态库)，无需再通过系统 `apt-get` 安装 `libopencv-dev`。
> 系统启动时会自动在内存中预生成 ORB 描述子 LUT 查找表。

---

## 🔨 编译构建指南

进入当前 `ubuntu` 目录，执行以下标准构建流程：

```bash
# 1. 创建并进入编译缓存文件夹
mkdir build
cd build

# 2. 运行 CMake 生成 Makefile (自动匹配项目根目录依赖)
cmake ..

# 3. 开启多核并发编译
make -j$(nproc)
```

编译完成后，会在当前 `build` 目录下生成两个可执行文件：

| 目标 | 文件 | 用途 |
|------|------|------|
| **VtonaxAR_Ubuntu** | `build/VtonaxAR_Ubuntu` | 实时 SLAM 演示 (相机/视频输入 + 交互式 GUI) |
| **VtonaxAR_Benchmark** | `build/VtonaxAR_Benchmark` | 离线视频评测工具 (性能分析 + 报告输出) |

---

## 💻 VtonaxAR_Ubuntu — 实时 SLAM 演示

### 启动命令

```bash
# 方案 A: 使用默认系统相机 (设备索引 0) 启动
./VtonaxAR_Ubuntu

# 方案 B: 指定其他相机索引 (例如设备索引 1) 启动
./VtonaxAR_Ubuntu 1

# 方案 C: 读取本地 MP4 视频流（支持自动循环播放）
./VtonaxAR_Ubuntu /path/to/your/video.mp4
```

### 键盘热键控制

- **`ESC`** 或 **`q`**：安全退出主处理循环，通知底层线程安全关机并释放所有内存资源。

### GUI 交互

右侧面板提供按钮式菜单控制：
- **AR 互动操作**: 放置/清除 AR 虚拟物体
- **地图持久化操作**: 保存/加载 SLAM 地图
- **SLAM 引擎控制**: 重置/开关 SLAM
- **可视化显示设置**: 切换点云显示

---

## 📊 VtonaxAR_Benchmark — 离线视频评测

离线处理视频文件，逐帧运行完整 SLAM 流水线，输出结构化性能分析报告。

> ⚠ **固定 30 FPS 处理**：无论输入视频原始帧率是多少（24/30/60/120 FPS），
> Benchmark 会自动跳帧保证 SLAM 始终以 **30 FPS** 匀速处理，
> 时间戳固定按 33.33ms 步进，确保评测结果在不同帧率输入间可比。

### 基本用法

```bash
# 基本用法：处理视频，输出 benchmark_report.json / .csv
./VtonaxAR_Benchmark test_video.mp4

# 指定输出报告路径
./VtonaxAR_Benchmark test_video.mp4 my_report.json

# 启用 GUI 预览窗口 (点云/跟踪状态可视化)
./VtonaxAR_Benchmark test_video.mp4 --gui

# 关闭 GUI 中的点云显示 (仅保留关键点和状态面板)
./VtonaxAR_Benchmark test_video.mp4 --gui --no-pointcloud
```

### 命令行选项

| 选项 | 说明 |
|------|------|
| `--gui`, `-g` | 开启 OpenCV GUI 窗口，实时显示视频、点云和跟踪状态 |
| `--no-pointcloud` | 关闭 GUI 中的全局点云投影显示（减轻视觉杂乱） |

### GUI 预览功能

使用 `--gui` 启动后，窗口左上角显示状态仪表盘：

```
┌─ VtonaxAR Benchmark ──────────────────┐
│ State: TRACKING OK                    │  ← 绿/红/黄色表示跟踪状态
│ FPS: 30                               │  ← 实时帧率
│ Frame: 1234/4939 (25%)                │  ← 处理进度
│ OK: 600  LOST: 300                    │  ← 跟踪统计
│ Map: 16 KF  366 MP                    │  ← 地图规模
└────────────────────────────────────────┘
```

- **青色点**: 新建的地图点（当前帧检测到的特征）
- **绿色点**: 已加载地图中成功匹配的点
- **红色点**: 已加载地图中未匹配的点
- **底部进度条**: 视频处理进度，颜色随跟踪状态变化
- 按 **ESC** 或 **q** 键可提前结束并生成已有数据的报告

### 输出文件

| 文件 | 格式 | 内容 |
|------|------|------|
| `benchmark_report.json` | JSON | 结构化报告：跟踪统计、实时性百分位、跟踪分段、场景难度、帧采样数据 |
| `benchmark_report.csv` | CSV | 逐帧完整数据 (frame_id, timestamp, tracking_state, process_ms, tracked_points, etc.) |

### 分析报告内容

**终端输出** 包含:

1. **跟踪稳定性**: OK/LOST/INIT 帧数及占比、最长连续跟踪/丢失段、分段时间线
2. **实时性分析**: 平均/中位/P95/P99 帧耗时、帧预算达标率、耗时分布直方图
3. **建图质量**: 最终地图点数、关键帧数、地图增长曲线
4. **丢失分析**: OK→LOST 转换点、丢失前跟踪点数变化
5. **场景难度分类**: 良好/中等/较难/困难 场景占比

**JSON 报告** 可用于进一步的数据可视化和对比分析。**CSV 文件** 可直接导入 Excel 或 Python 做逐帧深度分析。

### 视频格式要求

> ⚠ **重要**：项目 OpenCV 为静态编译，未链接 FFMPEG/GStreamer 后端，无法直接解码 H.264/H.265 等编码的 MP4 文件。

**支持的格式**：OpenCV 内置 MJPEG 编码的 AVI 文件。

若输入为 H.264 编码的 MP4，需用 `ffmpeg` 转码：
```bash
ffmpeg -i input.mp4 -vcodec mjpeg -q:v 5 -an output.avi
./VtonaxAR_Benchmark output.avi
```

### 性能注意事项

- 无论输入视频原始帧率高低，工具始终以**固定 30 FPS** 处理，确保评测结果稳定可比
- 输入帧率过高（如 120 FPS）且跳帧率较大时，部分极端场景（短时快速运动）可能被跳帧错过
- GUI 模式会略微降低处理速度，但可视化有助于观察跟踪行为
