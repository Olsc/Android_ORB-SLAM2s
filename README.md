![Banner](docs/img/banner_1.png)
![Banner](docs/img/banner_2.png)

<div align="center">

[简体中文](docs/README_zh.md) · [English](README.md) · [Русский](docs/README_ru.md) · [한국어](docs/README_ko.md) · [日本語](docs/README_ja.md)

</div>

---

<div align="center">

# 🌿 MenthaAR

**Visual SLAM solution compatible with almost all Android phones**

</div>

---

> **In a nutshell**: An enhanced spatial computing tool based on ORB-SLAM2 for Android, providing stable 6DoF spatial positioning for mobile devices.

<p align="center">
<img alt="Java" src="https://img.shields.io/badge/Java-ED8B00?style=flat-square&logo=openjdk&logoColor=white"/>
<img alt="C++11" src="https://img.shields.io/badge/C++-00599C?style=flat-square&logo=cplusplus&logoColor=white"/>
<img alt="Android" src="https://img.shields.io/badge/Android-3DDC84?style=flat-square&logo=android&logoColor=white"/>
<img alt="OpenCV" src="https://img.shields.io/badge/OpenCV-5C3EE8?style=flat-square&logo=opencv&logoColor=white"/>
<img alt="NDK" src="https://img.shields.io/badge/NDK-0C1E33?style=flat-square&logo=android&logoColor=white"/>
<img alt="GPL-3.0" src="https://img.shields.io/badge/License-GPL--3.0-blue?style=flat-square"/>
</p>

---

## 🙏 Acknowledgments

**【Solemn Thanks】 Original ORB-SLAM2 Core：**

https://github.com/raulmur/ORB_SLAM2

**【Solemn Thanks】 Original Android ORB-SLAM2 Project：**

https://github.com/Martin20150405/SLAM_AR_Android.git

---

# 🎯 ORB-SLAM2s: Android Spatial Computing & Lightweight AR Demo

*The lowercase 's' in ORB-SLAM2s stands for **Smart · Swift · Small**, emphasizing enhanced intelligence, faster performance, and lighter weight compared to the original ORB-SLAM2.*

This project uses lowercase 's'. Another paper with uppercase 'S' named ORB-SLAM2S:

```
(Y. Diao, R. Cen, F. Xue and X. Su, "ORB-SLAM2S: A Fast ORB-SLAM2 System
with Sparse Optical Flow Tracking," 2021 13th International Conference
on Advanced Computational Intelligence (ICACI), Wanzhou, China, 2021,
pp. 160-165, doi: 10.1109/ICACI52617.2021.9435915.)
```

It shares similar performance optimization concepts with this project, though its methods have not yet been integrated. This paper can serve as a valuable reference for future optimization directions. Sincere gratitude to the scholars for their contributions and sharing.

---

## 📋 Project Overview

**ORB-SLAM2s** is an enhanced spatial computing tool based on ORB-SLAM2 for Android. It supports real-time sparse point cloud mapping, map saving/loading, and relocation matching. By combining computer vision, inertial navigation (IMU), and Augmented Reality (AR) technologies, this project provides stable **6DoF** (Six Degrees of Freedom) spatial positioning for mobile devices.

### 🤔 Why ORB-SLAM2 instead of ORB-SLAM3?

- **Lightweight Design**: Fewer dependencies and streamlined code, making it easier to compile and deploy on mobile platforms.
- **About IMU & VIO Support**:
  - **High Barrier of ORB-SLAM3**: The VIO in the official implementation is **tightly coupled**, requiring high-quality IMU and strict **Camera-IMU timestamp synchronization**. This is common in robotics but difficult to achieve "out of the box" on Android phones via standard APIs.
  - **Android Reality**: Most existing Android devices use asynchronous acquisition for Camera and IMU, lacking a unified hardware synchronization framework, and consumer-grade IMUs have significant noise/drift. Without strict calibration and synchronization, VIO is more prone to divergence than pure visual SLAM. The SENSOR_TIMESTAMP method is only available in Android 11 and later.
- **Low Resource Consumption**: Lower CPU and RAM requirements compared to ORB-SLAM3.
- **Practical Efficiency**: Performance in monocular-only scenarios is comparable to ORB-SLAM3 for standard mobile use cases.

---

## ✨ Core Features

- **🗺️ Point Cloud SLAM Mapping** —— Real-time sparse mapping based on monocular camera input
- **💾 Map Persistence** —— Save maps to local storage and reload them for future sessions
- **🎯 Relocalization & Matching** —— Pose estimation and feature matching upon loading existing maps
- **📊 Confidence Visualization** —— Visual tracking of keypoints and matching statistics between current frames and the loaded map
- **📐 Plane Detection** —— Intelligent floor/surface detection based on current pose and point cloud data
- **🎨 Native AR Rendering** —— 3D rendering via **Google Filament** (GLB/glTF models) and OpenGL ES, supporting placement and interaction with virtual objects on detected planes
- **🌑 Dark Frame Detection** —— Automatically skips dark or low-quality frames to prevent SLAM thread blocking
- **🖱️ AR Object Management** —— Place, scale (pinch gesture), and interact with 3D objects
- **🧭 3DOF Orientation Tracking** —— Three-degrees-of-freedom orientation tracking using onboard device sensors (Rotation Vector / Accelerometer + Magnetometer)
- **🌐 Remote SLAM via Browser** —— Built-in SSL-encrypted HTTPS Web server. Use a browser as a remote camera source — send images to the device for SLAM processing and view real-time 3D point cloud and pose data
- **📑 Multi-Map Support** —— Simultaneous loading and matching of multiple map files

---

## ⚡ Performance Metrics

Currently tested primarily on Qualcomm Snapdragon platform CPUs:

| SoC | Device | Performance |
|---|---|---|
| Snapdragon 8 Elite | Xiaomi 15 | 30 FPS |
| Snapdragon 8+ Gen1 | Redmi K60 | 30 FPS |
| Snapdragon 870 | Xiaomi 10S | 30 FPS |
| Snapdragon 835 | Xiaomi 6 | 15–30 FPS |
| Snapdragon AR1 Gen 1 | Rokid Glasses | 10–25 FPS |

### 🌑 Dark Frame Detection

To ensure a smooth user experience, the system monitors exposure levels. When the environment is too dark, SLAM tracking is paused to avoid computational lag and "lost" states.

---

## ✅ Progress Tracker

- [x] Sparse Point Cloud SLAM Mapping
- [x] Map Save / Load functionality
- [x] Relocalization Matching
- [x] Basic AR Rendering Engine
- [x] Dark Frame Skip Logic
- [x] 3D AR Object Management
- [x] Simultaneous loading and matching of multiple map files
- [x] Integration with **Unity3D**

## 🗺️ Future Roadmap

- [ ] Improve mapping speed and initialization
- [ ] Enhance AR stability and 6DoF robustness
- [ ] Optimize rendering pipeline for higher frame rates
- [ ] Deepen sensor fusion (VIO —— Visual Inertial Odometry)
- [ ] SLAM frequency downsampling and adaptive noise handling
- [ ] Refined sensor gating logic

---

## 📦 Pull Source Code

```
git clone --recursive https://github.com/Olsc/Android_ORB-SLAM2s.git
```

## 💝 Acknowledgments

This project is built upon the following excellent open-source libraries:

- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
- [DBoW2](https://github.com/dorian3d/DBoW2)
- [g2o](https://github.com/RainerKuemmerle/g2o)
- [Eigen](http://eigen.tuxfamily.org/)
- [OpenCV](https://opencv.org/)

---

# 📚 ORB-SLAM2

**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

ORB-SLAM2 is a real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. We provide examples to run the SLAM system in the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) as stereo or monocular, in the [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset) as RGB-D or monocular, and in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as stereo or monocular.

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" alt="ORB-SLAM2" width="240" height="180" border="10" /></a>

### 📖 Related Publications

**[Monocular]** Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147–1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**

**[Stereo and RGB-D]** Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255–1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**

**[DBoW2 Place Recognizer]** Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp. 1188–1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

---

# 📜 License

## ORB-SLAM2 Core Library

The ORB-SLAM2 core library is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM2 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

## This Project (ORB-SLAM2s)

This Android adaptation and enhancement project (ORB-SLAM2s) is also licensed under the **GPL-3.0 License**. See the [LICENSE.txt](LICENSE.txt) and [License-gpl.txt](License-gpl.txt) files for details.

For project collaboration or other field cooperation inquiries, please contact: **OlscStudio@outlook.com**

## Third-Party Dependencies and Licenses

This project relies on several excellent open-source third-party libraries. We strictly adhere to their respective open-source licenses:

- **[OpenCV](https://github.com/opencv/opencv)** —— **Apache 2.0 License**
- **[srrg_hbst](https://gitlab.com/srrg-software/srrg_hbst)** —— **BSD 3-Clause License**. Used for fast and scalable image matching.
- **[DBoW2](https://github.com/dorian3d/DBoW2)** —— **BSD License**. Used for basic vocabulary vectors and feature matching structures.
- **[g2o](https://github.com/RainerKuemmerle/g2o)** —— **BSD License** (core components). Used for non-linear optimization.
- **[Eigen3](http://eigen.tuxfamily.org/)** —— **MPL2 (Mozilla Public License v2.0)**. Used for matrix operations and algebraic calculations.

For any issues related to third-party licenses, please refer to their respective official repositories.

### 📝 Academic Citations

If you use ORB-SLAM2 (Monocular) in an academic work, please cite:

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

If you use ORB-SLAM2 (Stereo or RGB-D) in an academic work, please cite:

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

If you use this Android adaptation (ORB-SLAM2s) in an academic work, please acknowledge this work appropriately.

---

# ⚙️ Prerequisites

## OpenCV

We use [OpenCV](http://opencv.org) to manipulate images and features.

## HBST (Hierarchical Bag of Scalable Trees)

We have integrated [srrg_hbst](https://gitlab.com/srrg-software/srrg_hbst) for fast and scalable image matching. It significantly improves loop closing and relocalization performance compared to DBoW2.

## Eigen3

Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org.

## DBoW2 and g2o (Included in Thirdparty folder)

We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

---

# 🧭 About Inertial Navigation (IMU)

Reference: https://github.com/Olsc/Android_3dof  

*This project has not yet completed research and integration.*

# 🐳 Mascot

<div align="center">

![Mascot](docs/img/mascot-q.png)

*Character: Mentha*
</div>

---

<br>

# ♥ Contributors

[![Contributors](https://contrib.rocks/image?repo=Olsc/Android_ORB-SLAM2s)](https://github.com/Olsc/Android_ORB-SLAM2s/graphs/contributors)
