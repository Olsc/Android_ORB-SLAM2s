# MenthaAR Windows SLAM

MenthaAR 的单目视觉 SLAM 桌面版本（Windows / Visual Studio 2022），支持实时摄像头和离线视频文件的 SLAM 跟踪、点云显示、AR 物体放置。

> 与 `ubuntu/` 目录共享同一套 SLAM 引擎源码（`src/` + `include/`），仅构建脚本与平台适配层不同。

---

## 📦 环境要求

| 组件 | 要求 |
|------|------|
| 操作系统 | Windows 10 / 11 (x64) |
| IDE / 工具链 | **Visual Studio 2022**（勾选"使用 C++ 的桌面开发"工作负载） |
| 编译器 | MSVC v143（VS2022 自带） |
| CMake | VS2022 自带（或任意 ≥ 3.10 版本） |
| Windows SDK | VS2022 安装时默认包含 |

无需手动安装 OpenCV/Eigen/g2o —— 工程使用内置源码（`Thirdparty/`），CMake 会自动以静态库方式编译。
OpenCV 的相机采集走内置 MSMF/DirectShow 后端，GUI 走内置 Win32 UI 后端；本机装有 FFmpeg 时 videoio 会自动探测并启用。

---

## 🔨 编译构建

在 **Developer Command Prompt for VS 2022**（或已将 VS 的 CMake 加入 PATH 的终端）中执行：

```bat
cd MenthaAR/src/main/cpp/windows
cmake -S . -B build_vs -G "Visual Studio 17 2022" -A x64
cmake --build build_vs --config Release --parallel %NUMBER_OF_PROCESSORS%
```

也可以直接用 VS 打开 `build_vs/MenthaAR_Windows.sln`，选择 Release | x64 后生成。

编译完成后，产物位于 `build_vs/bin/Release/` 目录：

| 目标 | 文件 | 用途 |
|------|------|------|
| **MenthaAR_Windows** | `MenthaAR_Windows.exe` | 实时 SLAM 演示 (相机/视频 + 交互式 GUI) |
| **MenthaAR_Benchmark** | `MenthaAR_Benchmark.exe` | 视频评测工具 (输出性能报告) |

> Debug 构建：`cmake --build build_vs --config Debug`，产物位于 `build_vs/bin/Debug/`。

---

## 💻 MenthaAR_Windows — 实时 SLAM 演示

### 启动

```bat
:: 默认系统相机 (设备索引 0)
MenthaAR_Windows.exe

:: 本地视频文件
MenthaAR_Windows.exe D:\videos\test.mp4

:: 指定相机索引
MenthaAR_Windows.exe 1
```

### 显示控制

- **分辨率上限**：显示画面最大 **1280×720（720P）**，超过的输入自动等比缩放，防止窗口过大
- **点云投影**：根据实际显示分辨率动态计算缩放系数，确保点云与画面精确对齐
- **点云颜色**：青色 = 新建地图点，绿色 = 已加载且匹配的地图点

### 交互面板

鼠标移入窗口右缘热区展开控制抽屉：

| 分组 | 功能 |
|------|------|
| **AR 互动操作** | 放置/清除 AR 虚拟立方体 |
| **地图持久化操作** | 保存/加载 SLAM 地图 (mentha_map.bin) |
| **SLAM 引擎控制** | 重置/开关 SLAM |
| **可视化显示设置** | 切换点云显示 |

### 键盘

- **ESC / q**：安全退出

---

## 📊 MenthaAR_Benchmark — 离线评测

逐帧运行完整 SLAM 流水线，输出结构化性能分析报告（含内存占用面板）。

> 固定 **30 FPS** 处理：无论输入视频原始帧率是多少，自动跳帧保证 SLAM 以 30 FPS 匀速处理。

### 用法

```bat
MenthaAR_Benchmark.exe D:\videos\test.mp4 [output_report_name] [--loops N]
```

输出 `[name].json` 和 `[name].csv` 报告文件。

> 进程内存统计：Windows 下通过 PSAPI (`GetProcessMemoryInfo`) 读取，Linux 下读取 `/proc/self/statm`，指标口径一致。
> 多音轨 AVI 自动修复功能依赖系统安装 FFmpeg 并加入 PATH（可选）。

---

## 🛠️ 平台适配要点（相对 Ubuntu/GCC 版本的差异）

| 项目 | Ubuntu (GCC) | Windows (MSVC) |
|------|-------------|----------------|
| 优化参数 | `-O3 -march=native -flto` | `/O2 /Oi /Zc:__cplusplus` |
| 编码统一 | — | `/utf-8`（全部源码为 UTF-8） |
| 并行加速 | TBB | Concurrency（OpenCV 内置并行框架） |
| 相机/GUI 后端 | V4L2 / GTK | MSMF+DShow / Win32 UI |
| g2o 兼容宏 | UNIX | 定义 `WINDOWS`/`_WINDOWS` 激活其自带 gettimeofday/vasprintf 实现 |
| 词库资源嵌入 | GCC 汇编 `.incbin` | 本测试版未使用嵌入资源，逻辑自动跳过 |

常见问题：
- 若提示找不到 `cmake`，请改用 "x64 Native Tools Command Prompt for VS 2022"，或将 `%VS_INSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin` 加入 PATH；
- 无摄像头时运行演示程序会提示 `Cannot open camera or video file`，属正常现象，请传入视频文件路径。
