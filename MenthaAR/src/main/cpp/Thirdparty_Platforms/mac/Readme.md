# MenthaAR macOS SLAM

MenthaAR 的单目视觉 SLAM 桌面版本（macOS / Apple Silicon 或 Intel），支持实时摄像头和离线视频文件的 SLAM 跟踪与点云显示。

> 与 `ubuntu/`、`windows/` 目录共享同一套 SLAM 引擎源码（`src/` + `include/`），仅构建脚本与平台适配层不同。

---

## 📦 环境要求

| 组件 | 要求 |
|------|------|
| 操作系统 | macOS 11+（Apple Silicon 或 Intel x64） |
| 编译器 | Apple Clang（随 **Xcode Command Line Tools** 提供） |
| 构建工具 | CMake ≥ 3.10（`brew install cmake`） |
| Make | 系统自带（或 `brew install ninja` 使用 Ninja 加速） |

无需手动安装 OpenCV/Eigen/g2o —— 工程使用内置源码（`Thirdparty/`），CMake 会自动以静态库方式编译。
OpenCV 的相机采集走内置 **AVFoundation** 后端，GUI 走内置 **Cocoa** 后端；本机装有 FFmpeg 时 videoio 会自动探测并启用。

```bash
# 安装 Xcode Command Line Tools（如未安装）
xcode-select --install

# 安装 CMake
brew install cmake
```

---

## 🔨 编译构建

```bash
cd MenthaAR/src/main/cpp/Thirdparty_Platforms/mac
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(sysctl -n hw.ncpu)
```

编译完成后，产物在 `build/bin/` 目录下：

| 目标 | 文件 | 用途 |
|------|------|------|
| **MenthaAR_macOS** | `build/bin/MenthaAR_macOS` | 实时 SLAM 演示 (相机/视频 + 交互式 GUI) |
| **MenthaAR_Benchmark** | `build/bin/MenthaAR_Benchmark` | 离线视频评测 (输出性能报告) |

> 构建脚本会自动探测目标架构：
> - **Apple Silicon (arm64)**：NEON/ASIMD 是 ABI 基线，默认即启用，无需额外标志
> - **Intel (x86_64)**：自动探测并启用 AVX/AVX2，等价于 Windows 版的最高性能路径

---

## 💻 MenthaAR_macOS — 实时 SLAM 演示

### 启动

```bash
# 默认系统相机 (设备索引 0)
./MenthaAR_macOS

# 本地视频文件
./MenthaAR_macOS /path/to/video.mp4

# 指定相机索引
./MenthaAR_macOS 1
```

### 显示控制

- **分辨率上限**：显示画面最大 **1280×720（720P）**，超过的输入自动等比缩放，防止窗口过大
- **点云投影**：根据实际显示分辨率动态计算缩放系数，确保点云与画面精确对齐
- **点云颜色**：青色 = 新建地图点，绿色 = 已加载且匹配的地图点

### 交互面板

鼠标移入窗口右缘热区展开控制抽屉：

| 分组 | 功能 |
|------|------|
| **地图持久化操作** | 保存/加载 SLAM 地图 (mentha_map.bin) |
| **可视化显示设置** | 切换点云显示 |

### 键盘

- **ESC / q**：安全退出

---

## 📊 MenthaAR_Benchmark — 离线评测

逐帧运行完整 SLAM 流水线，输出结构化性能分析报告（含内存占用面板）。

> 固定 **30 FPS** 处理：无论输入视频原始帧率是多少，自动跳帧保证 SLAM 以 30 FPS 匀速处理。

### 用法

```bash
./MenthaAR_Benchmark /path/to/video.mp4 [output_report_name] [--loops N]
```

输出 `[name].json` 和 `[name].csv` 报告文件。

> 进程内存统计：macOS 下通过 Mach 内核接口 (`task_info / MACH_TASK_BASIC_INFO`) 读取，Linux 下读取 `/proc/self/statm`，Windows 下通过 PSAPI (`GetProcessMemoryInfo`) 读取，指标口径一致。
> 多音轨 AVI 自动修复功能依赖系统安装 FFmpeg 并加入 PATH（可选）。

---

## ⚡ 性能说明

- 与 Ubuntu/Windows 版本一样，SLAM 引擎以 **`-O3 -flto`** 全量优化编译
- Apple Silicon 上 OpenCV 与 Eigen 默认启用 **NEON 向量化**，ORB 特征提取/匹配获得硬件加速
- Intel Mac 上自动启用 **AVX2**（如 CPU 支持），充分发挥 SIMD 吞吐
- 多核并行：OpenCV 线程数自动设置为逻辑核心数，SLAM 三线程（跟踪/局部建图/回环）并发运行
