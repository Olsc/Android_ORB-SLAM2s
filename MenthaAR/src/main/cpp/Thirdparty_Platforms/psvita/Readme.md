# MenthaAR PS Vita

MenthaAR 的 PlayStation Vita 版本（实验性）。

---

## ✨ 功能

| 功能 | 说明 |
|------|------|
| 单目 SLAM | 完整 ORB-SLAM2：ORB 特征、地图初始化、跟踪、局部建图、回环（HBST）、g2o 优化 |
| 实时相机 | 后置 / 前置相机，640×360 ABGR，30 FPS |
| 点云显示 | 将 `GetAllMapPoints()` 按 `Tcw` 投影绘制；青色=新建点，绿色=已加载点 |
| 特征点 | `GetTrackedKeyPointsUn()` 跟踪特征点叠加显示 |
| 地图持久化 | `SaveMap` / `LoadMap`，二进制地图 `ux0:data/MenthaAR/mentha_map.bin` |
| HUD | 跟踪状态、FPS、关键帧数、地图点数、跟踪点数 |

---

## 🎮 操作

| 按键 | 功能 |
|------|------|
| **O** | 保存地图到 `ux0:data/MenthaAR/mentha_map.bin` |
| **□** | 从该路径读取地图 |
| **△** | 重置 SLAM（清空当前地图） |
| **L1** | 显示 / 隐藏点云 |
| **R1** | 在“全部地图点” / “仅已加载地图点”之间切换 |
| **SELECT** | 切换前 / 后相机 |
| **START** | 退出（会先安全关闭 SLAM 线程） |

> 使用方式与桌面版一致：启动后**缓慢平移/旋转相机**让系统完成初始化，HUD 显示
> `TRACKING OK` 后即可到处扫描；移动过快会显示 `TRACKING LOST`。

---

## 🔨 编译

需要 [VitaSDK](https://vitasdk.org/)：

```bash
export VITASDK=/usr/local/vitasdk
export PATH=$VITASDK/bin:$PATH

cd MenthaAR/src/main/cpp/Thirdparty_Platforms/psvita
# 首次拉取：初始化 mini OpenCV 子模块
git submodule update --init --recursive
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

产物：

| 文件 | 说明 |
|------|------|
| `MenthaAR_PSVita.vpk` | 可安装到 Vita / VitaShell / Vita3K 的安装包 |
| `MenthaAR_PSVita.self` | 压缩后的 SELF |
| `MenthaAR_PSVita.velf` | 中间 ELF（未压缩，可 `nm` 查看符号） |

---

## 🧩 引擎如何在没有官方 OpenCV 的 Vita 上编译

VitaSDK 不提供 OpenCV。本移植通过以下方式让**原引擎源码零改动**地编译：

1. **`thirdparty/libopencv4/`（git 子模块）**
   社区维护的 “mini OpenCV 4 for VitaSDK”（core / imgproc / features2d / imgcodecs
   预编译静态库 + 头文件，基于 OpenCV 4.2，来源 `github.com/dejavulife/libopencv4`）。
   以 git submodule 方式引入，不再把库文件直接塞进本仓库。

   该子模块的 `saturate.hpp` 在现代 GCC 下会因 `int32_t` 与 `int` 重定义而报错。
   为保持子模块干净，构建脚本在配置阶段从子模块读取该头文件、删除重复的
   `int32_t`/`uint32_t` 重载，生成影子头文件到 `${CMAKE_BINARY_DIR}/opencv_shim/`
   并置于子模块 include 目录之前（`int`/`unsigned` 重载仍然覆盖）。

2. **`compat/opencv_calib3d_compat.{h,cpp}`**
   mini OpenCV 缺少 calib3d，而引擎只用到其中 4 个函数：
   `cv::undistortPoints`、`cv::Rodrigues`、`cv::solvePnP`、`cv::solvePnPRansac`。
   本文件用标准算法（Rodrigues 公式、迭代去畸变、DLT + Levenberg-Marquardt、
   RANSAC）实现它们，并通过 `-include` 强制注入到引擎编译单元，
   链接进最终 ELF —— **不修改引擎源码**。

3. **`compat/vasprintf_compat.{h,cpp}`**
   g2o 的 `string_tools.cpp` 使用 `vasprintf`，但 newlib 未提供；在这里补齐。

4. **构建脚本** 用 `-Wl,--start-group … --end-group` 解决 OpenCV / g2o 静态库
   之间的循环依赖。

---

## 📦 安装

1. 用 FTP（VitaShell 按 SELECT 开启）把 `MenthaAR_PSVita.vpk` 传到 `ux0:/`；
2. VitaShell 里选中它按 **X** 安装；
3. LiveArea 启动 “MenthaAR PSVita”。

地图文件：`ux0:/data/MenthaAR/mentha_map.bin`。

---

## ⚠️ 已知限制

- **性能**：PS Vita 是 4×Cortex-A9 @ ~444MHz，ORB-SLAM2 在此为低帧率运行（通常个位数
  FPS），这是硬件限制。建议缓慢移动相机。
- **内存**：`Config.h` 的默认上限（`MAX_KEYFRAMES=2000`、`MAX_MAPPOINTS=10000`）是
  为手机/桌面设定的。Vita 长时间扫描可能接近内存上限；原项目文件未做改动，
  如需更保守的预算可在其它平台统一调整 `Config.h`。
- **PnP 的 4–5 点分支**：兼容层基于 DLT，要求 ≥6 点。引擎在 ≥6 点走 RANSAC 主路径；
  少数 4–5 点后台重定位分支会返回 `false`（引擎会安全跳过，不影响主跟踪）。
- **未实机验证**：本次交付完成了完整的交叉编译、链接与 VPK 打包，并核对了 ELF 中
  包含 `System::TrackMonocular`、`SaveMap`、`LoadMap`、`ORBextractor` 及兼容层符号；
  但在真实硬件/模拟器上的运行表现（帧率、初始化稳定性、内存）仍需实机确认。
- `thirdparty/libopencv4` 为社区构建的子模块，仓库未附带许可证；OpenCV 本身为 Apache-2.0。
