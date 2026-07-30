# MenthaAR Ubuntu SLAM

MenthaAR 的单目视觉 SLAM 桌面版本，支持实时摄像头和离线视频文件的 SLAM 跟踪、点云显示、AR 物体放置。

---

## 📦 依赖安装

```bash
# 更新系统源并安装基础构建链
sudo apt-get update
sudo apt-get install -y build-essential cmake libgtk-3-dev libtbb-dev libavcodec-dev libavformat-dev libavutil-dev libswscale-dev
sudo apt-get install -y ffmpeg
```

---

## 🔨 编译构建

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

编译完成后，产物在 `build/bin/` 目录下：

| 目标 | 文件 | 用途 |
|------|------|------|
| **MenthaAR_Ubuntu** | `build/bin/MenthaAR_Ubuntu` | 实时 SLAM 演示 (相机/视频 + 交互式 GUI) |
| **MenthaAR_Benchmark** | `build/bin/MenthaAR_Benchmark` | 离线视频评测 (无窗口，输出性能报告) |

---

## 💻 MenthaAR_Ubuntu — 实时 SLAM 演示

### 启动

```bash
# 默认系统相机 (设备索引 0)
./MenthaAR_Ubuntu

# 本地视频文件
./MenthaAR_Ubuntu /path/to/video.mp4

# 指定相机索引
./MenthaAR_Ubuntu 1
```

### 显示控制

- **分辨率上限**：显示画面最大 **1280×720（720P）**，超过的输入自动等比缩放，防止窗口过大
- **点云投影**：根据实际显示分辨率动态计算缩放系数，消除硬编码 ×2 假设，确保点云与画面精确对齐
- **点云颜色**：青色 = 新建地图点，绿色 = 已加载且匹配的地图点

### 交互面板

窗口右侧按钮式菜单：

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

逐帧运行完整 SLAM 流水线，输出结构化性能分析报告。**无窗口渲染**，纯 CPU 性能评测。

> 固定 **30 FPS** 处理：无论输入视频原始帧率是多少，自动跳帧保证 SLAM 以 30 FPS 匀速处理。

### 用法

```bash
./MenthaAR_Benchmark /path/to/video.mp4 [output_report_name]
```

输出 `[name].json` 和 `[name].csv` 报告文件。