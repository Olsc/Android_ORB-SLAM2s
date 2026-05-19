# VtonaxAR Ubuntu SLAM

### 📦 安装依赖指令 (Ubuntu 20.04)

```bash
# 更新系统源并安装基础构建链
sudo apt-get update
sudo apt-get install -y build-essential cmake

# 安装 OpenCV 开发依赖库
sudo apt-get install -y libopencv-dev
```

> [!IMPORTANT]
> **资源前置检查**：
> 在编译前，请确保项目仓库根目录下的资源文件 `VtonaxAR/src/main/other/ORBvoc.txt.arm.bin` 与 `ORB_LUT.bin` 存在，CMake 会自动加载它们并编译为静态对象。

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

编译完成后，会在当前 `build` 目录下生成名为 `VtonaxAR_Ubuntu` 的可执行文件。

---

## 💻 运行与控制说明

### 1. 启动命令

```bash
# 方案 A: 使用默认系统相机 (设备索引 0) 启动
./VtonaxAR_Ubuntu

# 方案 B: 指定其他相机索引 (例如设备索引 1) 启动
./VtonaxAR_Ubuntu 1

# 方案 C: 读取本地 MP4 视频流（支持自动循环播放）
./VtonaxAR_Ubuntu /path/to/your/video.mp4
```

### 2. 键盘热键控制

- **`ESC`** 或 **`q`**：安全退出主处理循环，通知底层线程安全关机并释放所有内存资源。
