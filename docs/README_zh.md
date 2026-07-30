![Banner](img/banner_1.png)
![Banner](img/banner_2.png)

<div align="center">

[简体中文](README_zh.md) · [English](../README.md) · [Русский](README_ru.md) · [한국어](README_ko.md) · [日本語](README_ja.md)

</div>

---

<div align="center">

# 🌿 薄荷AR

**兼容几乎所有安卓手机的视觉SLAM方案**

</div>

---

> **一句话定位**：基于 ORB-SLAM2 的 Android 增强空间计算工具，为移动设备提供稳定的 6DoF 空间定位。

<p align="center">
<img alt="Java" src="https://img.shields.io/badge/Java-ED8B00?style=flat-square&logo=openjdk&logoColor=white"/>
<img alt="C++11" src="https://img.shields.io/badge/C++-00599C?style=flat-square&logo=cplusplus&logoColor=white"/>
<img alt="Android" src="https://img.shields.io/badge/Android-3DDC84?style=flat-square&logo=android&logoColor=white"/>
<img alt="OpenCV" src="https://img.shields.io/badge/OpenCV-5C3EE8?style=flat-square&logo=opencv&logoColor=white"/>
<img alt="NDK" src="https://img.shields.io/badge/NDK-0C1E33?style=flat-square&logo=android&logoColor=white"/>
<img alt="GPL-3.0" src="https://img.shields.io/badge/License-GPL--3.0-blue?style=flat-square"/>
</p>

---

## 🙏 致谢

**【郑重感谢】原始 ORB-SLAM2 核心：**

https://github.com/raulmur/ORB_SLAM2

**【郑重感谢】原始 Android ORB-SLAM2 项目：**

https://github.com/Martin20150405/SLAM_AR_Android.git

---

# 🎯 ORB-SLAM2s：Android 空间计算与轻量级 AR 演示

*ORB-SLAM2s 中的小写 's' 代表 **智能 · 快速 · 小巧**，强调与原始 ORB-SLAM2 相比，具有增强的智能、更快的性能和更轻的体积。*

本项目使用的是小写 s，另外一篇大写 S 的论文名为 ORB-SLAM2S：

```
(Y. Diao, R. Cen, F. Xue and X. Su, "ORB-SLAM2S: A Fast ORB-SLAM2 System
with Sparse Optical Flow Tracking," 2021 13th International Conference
on Advanced Computational Intelligence (ICACI), Wanzhou, China, 2021,
pp. 160-165, doi: 10.1109/ICACI52617.2021.9435915.)
```

与本文在性能优化上的理念相同，但尚未集成此论文里的方法，此论文可作为未来优化方向的优质参考。衷心感谢学者们的付出和共享。

---

## 📋 项目概述

**ORB-SLAM2s** 是基于 ORB-SLAM2 的 Android 增强空间计算工具。它支持实时稀疏点云地图、地图保存/加载和重定位匹配。通过结合计算机视觉、惯性导航（IMU）和增强现实（AR）技术，本项目为移动设备提供稳定的 **6DoF**（六自由度）空间定位。

### 🤔 为什么选择 ORB-SLAM2 而不是 ORB-SLAM3？

- **轻量级设计**：更少的依赖和精简的代码，使其更容易在移动平台上编译和部署。
- **关于 IMU 与 VIO 支持**：
  - **ORB-SLAM3 的高门槛**：官方实现中的 VIO 是**紧耦合**的，要求高质量 IMU 和严格的 **Camera-IMU 时间戳同步**。这在机器人领域常见，但在 Android 手机上通过标准 API 很难做到"开箱即用"。
  - **Android 现状**：现有 Android 设备多为 Camera 与 IMU 异步采集，缺乏统一的硬件同步框架，且消费级 IMU 噪声/漂移较大。若无严格标定和同步，VIO 反而比纯视觉更易发散。安卓 11 以后的版本才有 SENSOR_TIMESTAMP 方法。
- **低资源消耗**：相比 ORB-SLAM3，CPU 和 RAM 需求更低。
- **实用效率**：在仅单目相机场景中的性能与 ORB-SLAM3 在标准移动用例中相当。

---

## ✨ 核心特性

- **🗺️ 点云 SLAM 映射** —— 基于单目相机输入的实时稀疏映射
- **💾 地图持久化** —— 支持将地图保存到本地存储并重新加载以供将来会话使用
- **🎯 重定位与匹配** —— 加载现有地图时的姿态估计和特征匹配
- **📊 置信度可视化** —— 可视化跟踪关键点以及当前帧与加载地图之间的匹配统计信息
- **📐 平面检测** —— 基于当前姿态和点云数据的智能地板/表面检测
- **🎨 原生 AR 渲染** —— 基于 **Google Filament**（GLB/glTF 模型）和 OpenGL ES 的 3D 渲染，支持在检测到的平面上放置和交互虚拟物体
- **🌑 暗帧检测** —— 自动跳过暗色或低质量帧以防止 SLAM 线程阻塞
- **🖱️ AR 对象管理** —— 支持 3D 物体的放置、双指缩放等交互操作
- **🧭 3DOF 姿态跟踪** —— 利用设备内置传感器（旋转矢量传感器 / 加速度计 + 磁力计）实现三自由度方向跟踪
- **🌐 浏览器端 SLAM 接入** —— 内置 SSL 加密的 HTTPS Web 服务器。浏览器可作为远程相机源，将图像发送至手机进行 SLAM 处理，并实时展示 3D 点云和位姿数据
- **📑 多地图支持** —— 支持多个地图文件的同时加载和匹配

---

## ⚡ 性能指标

目前主要在高通骁龙平台 CPU 上测试：

| SoC | 设备 | 性能 |
|---|---|---|
| Snapdragon 8 Elite | 小米 15 | 30 FPS |
| Snapdragon 8+ Gen1 | 红米 K60 | 30 FPS |
| Snapdragon 870 | 小米 10S | 30 FPS |
| Snapdragon 835 | 小米 6 | 15–30 FPS |
| Snapdragon AR1 Gen 1 | Rokid Glasses | 10–25 FPS |

### 🌑 暗帧检测

为确保流畅的用户体验，系统会监控曝光水平。当环境太暗时，SLAM 跟踪会暂停以避免计算延迟和"丢失"状态。

---

## ✅ 进度跟踪

- [x] 稀疏点云 SLAM 映射
- [x] 地图保存 / 加载功能
- [x] 重定位匹配
- [x] 基础 AR 渲染引擎
- [x] 暗帧跳过逻辑
- [x] 3D AR 对象管理
- [x] 多个地图文件的同时加载和匹配
- [x] 与 **Unity3D** 集成

## 🗺️ 未来路线图

- [ ] 提高映射速度和初始化
- [ ] 增强 AR 稳定性和 6DoF 鲁棒性
- [ ] 优化渲染管道以获得更高的帧率
- [ ] 加深传感器融合（VIO —— 视觉惯性测程）
- [ ] SLAM 频率降采样和自适应噪声处理
- [ ] 精细的传感器门控逻辑

---

## 📦 拉取源码

```
git clone --recursive https://github.com/Olsc/Android_ORB-SLAM2s.git
```

## 💝 致谢

本项目基于以下优秀的开源库构建：

- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
- [DBoW2](https://github.com/dorian3d/DBoW2)
- [g2o](https://github.com/RainerKuemmerle/g2o)
- [Eigen](http://eigen.tuxfamily.org/)
- [OpenCV](https://opencv.org/)

---

# 📚 ORB-SLAM2

**作者：** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/)、[Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/)、[J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) 和 [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

ORB-SLAM2 是一个实时 SLAM 库，用于**单目**、**立体**和 **RGB-D** 相机，计算相机轨迹和稀疏 3D 重建（在立体和 RGB-D 情况下具有真实比例）。它能够实时检测循环并重新定位相机。我们提供了在 [KITTI 数据集](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)上以立体或单目方式运行 SLAM 系统的示例，在 [TUM 数据集](http://vision.in.tum.de/data/datasets/rgbd-dataset)上以 RGB-D 或单目方式运行，以及在 [EuRoC 数据集](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)上以立体或单目方式运行。

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" alt="ORB-SLAM2" width="240" height="180" border="10" /></a>

### 📖 相关出版物

**[单目]** Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147–1163, 2015. (**2015 IEEE Transactions on Robotics 最佳论文奖**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**

**[立体和 RGB-D]** Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255–1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**

**[DBoW2 地点识别器]** Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp. 1188–1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

---

# 📜 许可证

## ORB-SLAM2 核心库

ORB-SLAM2 核心库以 [GPLv3 许可证](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt)发布。有关所有代码/库依赖项（及相关许可证）的列表，请参见 [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md)。

如需商业用途的闭源版本 ORB-SLAM2，请联系作者：orbslam (at) unizar (dot) es。

## 本项目 (ORB-SLAM2s)

此 Android 适配和增强项目（ORB-SLAM2s）也根据 **GPL-3.0 许可证**授权。详情请参见 [LICENSE.txt](../LICENSE.txt) 和 [License-gpl.txt](../License-gpl.txt) 文件。

项目合作或其他领域合作咨询，请联系：**OlscStudio@outlook.com**

## 第三方依赖库及开源许可声明

本项目的发展离不开众多优秀的开源项目。我们在本项目中集成并使用了以下第三方库，并严格遵守其开源协议：

- **[OpenCV](https://github.com/opencv/opencv)** —— **Apache 2.0 许可证**
- **[srrg_hbst](https://gitlab.com/srrg-software/srrg_hbst)** —— **BSD 3-Clause 许可证**。用于快速、增量式的可扩展图像匹配与重定位。
- **[DBoW2](https://github.com/dorian3d/DBoW2)** —— **BSD 许可证**。用于特征向量化及基础的局部特征匹配。
- **[g2o](https://github.com/RainerKuemmerle/g2o)** —— **BSD 许可证**（其核心部分）。用于图优化及非线性误差最小化。
- **[Eigen3](http://eigen.tuxfamily.org/)** —— **MPL2 (Mozilla Public License v2.0) 许可证**。用于矩阵及线性代数运算。

各第三方库的具体使用条款请参考其各自的官方仓库。

### 📝 学术引用

如果您在学术工作中使用 ORB-SLAM2（单目），请引用：

```bibtex
@article{murTRO2015,
  title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
  author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics},
  volume={31},
  number={5},
  pages={1147--1163},
  doi = {10.1109/TRO.2015.2463671},
  year={2015}
}
```

如果您在学术工作中使用 ORB-SLAM2（立体或 RGB-D），请引用：

```bibtex
@article{murORB2,
  title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular,
          Stereo and {RGB-D} Cameras},
  author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics},
  volume={33},
  number={5},
  pages={1255--1262},
  doi = {10.1109/TRO.2017.2705103},
  year={2017}
}
```

如果您在学术工作中使用此 Android 适配版（ORB-SLAM2s），请适当引用本开源地址 URL。

---

# ⚙️ 先决条件

## OpenCV

我们使用 [OpenCV](http://opencv.org) 来操作图像和特征。

## HBST（Hierarchical Bag of Scalable Trees）

项目集成了 [srrg_hbst](https://gitlab.com/srrg-software/srrg_hbst)，用于快速且可扩展的图像匹配。与传统的 DBoW2 相比，它在闭环检测和重定位性能上有了显著提升。

## Eigen3

g2o 所需（见下文）。下载和安装说明可在以下网址找到：http://eigen.tuxfamily.org。

## DBoW2 和 g2o（包含在 Thirdparty 文件夹中）

我们使用 [DBoW2](https://github.com/dorian3d/DBoW2) 和 [g2o](https://github.com/RainerKuemmerle/g2o) 库进行算法上的引用。这两个库（包含许可证）都包含在 *Thirdparty* 文件夹中。

---

# 🧭 关于惯性导航（IMU）

参考：https://github.com/Olsc/Android_3dof  

*本项目尚未研究整合完成。*

# 🐳吉祥物

<div align="center">

![Mascot](img/mascot-q.png)

*角色：薄荷*
</div>

---

<br>

# ♥ 贡献者

[![Contributors](https://contrib.rocks/image?repo=Olsc/Android_ORB-SLAM2s)](https://github.com/Olsc/Android_ORB-SLAM2s/graphs/contributors)
