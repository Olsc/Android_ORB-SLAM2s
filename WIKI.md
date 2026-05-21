# ORB-SLAM2s: Android 平台轻量化空间计算与增强现实系统

## 摘要

本文档详细阐述了 ORB-SLAM2s 的数学原理与核心算法实现，以及基于源码的实际系统架构。作为一个基于 Android 平台的轻量化空间计算系统，本项目在经典 ORB-SLAM2 框架的基础上，进一步集成了多地图管理、异步后台重定位、即时平面检测、地图持久化与 AR 场景恢复等功能。本文将严谨地从数学角度描述系统的各个模块，包括基于李代数的位姿估计、光束法平差（Bundle Adjustment）、词袋模型（Bag of Words）的回环检测、基于奇异值分解（SVD）的平面拟合算法，以及项目的核心创新——异步全局重定位机制。

## 1. 引言

同步定位与地图构建（SLAM）是移动机器人与增强现实（AR）领域的核心技术。ORB-SLAM2s 旨在移动端有限的计算资源下，通过单目相机实现鲁棒的 6DoF 轨迹跟踪与稀疏环境重建。系统采用多线程架构，并行处理跟踪、局部建图与回环检测任务，并在此基础上增加了异步全局重定位线程，支持多地图加载与坐标系无缝对齐。

**与原始 ORB-SLAM2 的关键差异**:
- 多地图容器管理（`mvpMaps`），支持创建子地图、切换地图、增量追加
- 独立的 GlobalReloc 后台线程，持续运行全局重定位
- 编译时参数配置（`Config.h`），替代 YAML 运行时加载
- 嵌入式资源加载（ORB 词汇表通过 `.incbin` 直接嵌入 `.so`）
- 动态分辨率适配（运行时缩放内参与投影矩阵）
- AR 平面检测与虚拟物体持久化

## 2. 符号约定

*   **世界坐标系** (World frame): $\mathcal{F}_w$
*   **相机坐标系** (Camera frame): $\mathcal{F}_c$
*   **相机内参矩阵**: $\mathbf{K}$
*   **李群与李代数**: 特殊欧几里得群 $SE(3)$ 与其对应的李代数 $\mathfrak{se}(3)$。
    *   旋转矩阵 $\mathbf{R} \in SO(3)$，平移向量 $\mathbf{t} \in \mathbb{R}^3$。
    *   变换矩阵 $\mathbf{T}_{cw} \in SE(3)$ 表示从世界坐标系到相机坐标系的变换。
*   **地图点**: $\mathbf{X}_w \in \mathbb{R}^3$ 表示世界坐标系下的 3D 点。
*   **相似变换**: $\mathbf{S} \in Sim(3)$ 包含尺度因子 $s$，用于闭环校正。

## 3. 视觉里程计与位姿跟踪 (Visual Odometry & Tracking)

### 3.1 针孔相机投影模型

对于世界坐标系中的一点 $\mathbf{P}_w = [X_w, Y_w, Z_w]^T$，其在当前相机坐标系下的坐标 $\mathbf{P}_c = [X_c, Y_c, Z_c]^T$ 由刚体变换给出：

$$
\mathbf{P}_c = \mathbf{R}_{cw} \mathbf{P}_w + \mathbf{t}_{cw}
$$

该点投影到像素平面的坐标 $\mathbf{u} = [u, v]^T$ 由投影函数 $\pi(\cdot)$ 描述：

$$
\mathbf{u} = \pi(\mathbf{P}_c) = \begin{bmatrix} f_x \frac{X_c}{Z_c} + c_x \\ f_y \frac{Y_c}{Z_c} + c_y \end{bmatrix}
$$

其中 $f_x, f_y$ 为焦距，$c_x, c_y$ 为主点坐标。实际内参通过 `Config.h` 编译时常量定义：

```cpp
const float CAMERA_FX = 640.0f;  // 640x360 基准分辨率下标定
const float CAMERA_FY = 640.0f;
const float CAMERA_CX = 320.0f;
const float CAMERA_CY = 180.0f;
```

**动态分辨率缩放**: 系统支持运行时更改相机分辨率。基准内参在 640x360 下标定，实际运行时按比例缩放：

```cpp
float scaleX = (float)slamWidth / BASE_SLAM_WIDTH;   // 640
float scaleY = (float)slamHeight / BASE_SLAM_HEIGHT;  // 360
gScaledFx = gBaseFx * scaleX;
```

同时通过 `slamSys->UpdateCalibration(fx, fy, cx, cy)` 同步更新 SLAM 核心模块的校准参数。

### 3.2 基于李代数的位姿优化

跟踪线程通过最小化重投影误差（Reprojection Error）来优化当前帧的位姿 $\mathbf{T}_{cw}$。定义误差项为观测像素坐标 $\mathbf{u}_{obs}$ 与投影坐标的差异：

$$
\mathbf{e} = \mathbf{u}_{obs} - \pi(\mathbf{T}_{cw} \mathbf{P}_w)
$$

为了求解最优位姿，我们对李代数 $\boldsymbol{\xi} \in \mathfrak{se}(3)$ 进行线性化。设扰动量为 $\delta \boldsymbol{\xi}$，则误差函数的一阶泰勒展开为：

$$
\mathbf{e}(\boldsymbol{\xi} \oplus \delta \boldsymbol{\xi}) \approx \mathbf{e}(\boldsymbol{\xi}) + \mathbf{J} \delta \boldsymbol{\xi}
$$

其中 $\mathbf{J} = \frac{\partial \mathbf{e}}{\partial \delta \boldsymbol{\xi}}$ 为雅可比矩阵。对于重投影误差，雅可比矩阵为 $2 \times 6$ 矩阵：

$$
\mathbf{J} = - \begin{bmatrix} \frac{f_x}{Z'} & 0 & -\frac{f_x X'}{Z'^2} & -\frac{f_x X' Y'}{Z'^2} & f_x (1 + \frac{X'^2}{Z'^2}) & -\frac{f_x Y'}{Z'} \\ 0 & \frac{f_y}{Z'} & -\frac{f_y Y'}{Z'^2} & -f_y (1 + \frac{Y'^2}{Z'^2}) & \frac{f_y X' Y'}{Z'^2} & \frac{f_y X'}{Z'} \end{bmatrix}
$$

系统使用高斯-牛顿（Gauss-Newton）或列文伯格-马夸特（Levenberg-Marquardt）算法迭代求解增量 $\delta \boldsymbol{\xi}$：

$$
(\mathbf{H} + \lambda \mathbf{I}) \delta \boldsymbol{\xi} = -\mathbf{b}
$$

其中 $\mathbf{H} = \sum \mathbf{J}^T \Omega \mathbf{J}$ 为海森矩阵（Hessian Matrix），$\mathbf{b} = \sum \mathbf{J}^T \Omega \mathbf{e}$，$\Omega$ 为信息矩阵（协方差矩阵的逆）。

### 3.3 恒速运动模型 (Constant Velocity Motion Model)

为加速跟踪收敛，系统引入恒速运动模型预测当前帧的初始位姿。假设从 $t_{k-1}$ 到 $t_k$ 的相对变换为 $\mathbf{V}_{k}$（即 $\mathbf{T}_{cw, k} = \mathbf{V}_k \mathbf{T}_{cw, k-1}$），则下一帧的先验位姿估计为：

$$
\tilde{\mathbf{T}}_{cw, k+1} = \mathbf{V}_k \mathbf{T}_{cw, k}
$$

若跟踪成功，则更新速度 $\mathbf{V}_{k+1} = \mathbf{T}_{cw, k+1} \mathbf{T}_{cw, k}^{-1}$。

### 3.4 全局重定位与 EPnP 算法

当跟踪丢失时，系统利用词袋模型（BoW）进行全局重定位，建立当前帧特征点与地图点的 3D-2D 对应关系。位姿初值通过 EPnP (Efficient Perspective-n-Point) 算法求解。

EPnP 将 $n$ 个 3D 点表示为 4 个控制点 $\mathbf{C}_j^w$ 的加权和：

$$
\mathbf{P}_i^w = \sum_{j=1}^4 \alpha_{ij} \mathbf{C}_j^w, \quad \sum_{j=1}^4 \alpha_{ij} = 1
$$

在相机坐标系下该关系保持不变：

$$
\mathbf{P}_i^c = \sum_{j=1}^4 \alpha_{ij} \mathbf{C}_j^c
$$

结合投影约束，构建线性方程组 $\mathbf{M}\mathbf{x} = \mathbf{0}$ 求解控制点在相机坐标系下的坐标，进而通过 Procrustes 分析恢复旋转 $\mathbf{R}$ 和平移 $\mathbf{t}$。随后使用 RANSAC 剔除外点。

**代码实现差异**: 项目中的 `PnPsolver` 额外实现了数据清洗（NaN/Inf 检查）、策略分级（$N<4$ 直接返回、$N<6$ 跳过 RANSAC、$N\ge6$ 启用 RANSAC）以及 OpenCV 断言错误的异常捕获。

### 3.5 单目初始化 (Monocular Initialization)

单目 SLAM 系统缺乏深度信息，需通过两帧运动恢复初始地图。系统并行计算单应性矩阵 (Homography) $\mathbf{H}_{21}$ 和基础矩阵 (Fundamental) $\mathbf{F}_{21}$，并通过评分模型选择最优模型。

#### 3.5.1 模型计算与评分

1.  **单应性矩阵 $\mathbf{H}$ (DLT 算法)**:
    适用于平面场景或纯旋转运动。对应点满足 $\mathbf{x}_2 = \mathbf{H}_{21} \mathbf{x}_1$。通过直接线性变换 (DLT) 构建方程 $\mathbf{A}\mathbf{h} = \mathbf{0}$，利用 SVD 求解。

2.  **基础矩阵 $\mathbf{F}$ (8点法)**:
    适用于一般场景。对应点满足 $\mathbf{x}_2^T \mathbf{F}_{21} \mathbf{x}_1 = 0$。同样通过 SVD 求解最小二乘解，并强制秩为 2 约束。

3.  **模型选择**:
    计算每个模型的对称转移误差 (Symmetric Transfer Error) 得分 $S_M$ ($M \in \{H, F\}$)：
    $$
    S_M = \sum_i (\rho(d^2(\mathbf{x}_{2i}, M \mathbf{x}_{1i})) + \rho(d^2(\mathbf{x}_{1i}, M^{-1} \mathbf{x}_{2i})))
    $$
    其中 $\rho(\cdot)$ 为卡方检验阈值截断函数。若 $S_H / (S_H + S_F) > 0.40$（Config.h: `INITIALIZER_H_SCORE_RATIO`），选择单应性模型；否则选择基础矩阵模型。初始化成功后要求视差 > 1.0° 且三角化点数 > 50。

### 3.6 优化雅可比矩阵 (Jacobians in Optimization)

在位姿优化与 BA 中，需计算残差关于状态量的雅可比矩阵。对于重投影误差 $\mathbf{e}_{ij} = \mathbf{u}_{ij} - \pi(\mathbf{R} \mathbf{X}_j + \mathbf{t})$：

1.  关于位姿的雅可比 $\mathbf{J}_{\boldsymbol{\xi}}$:
    $$
    \frac{\partial \mathbf{e}_{ij}}{\partial \delta \boldsymbol{\xi}} = - \begin{bmatrix} \frac{f_x}{Z'} & 0 & -\frac{f_x X'}{Z'^2} & -\frac{f_x X' Y'}{Z'^2} & f_x (1 + \frac{X'^2}{Z'^2}) & -\frac{f_x Y'}{Z'} \\ 0 & \frac{f_y}{Z'} & -\frac{f_y Y'}{Z'^2} & -f_y (1 + \frac{Y'^2}{Z'^2}) & \frac{f_y X' Y'}{Z'^2} & \frac{f_y X'}{Z'} \end{bmatrix}
    $$
    其中 $[X', Y', Z']^T = \mathbf{R} \mathbf{X}_j + \mathbf{t}$。

2.  关于地图点的雅可比 $\mathbf{J}_{\mathbf{P}}$:
    $$
    \frac{\partial \mathbf{e}_{ij}}{\partial \delta \mathbf{P}_j} = - \frac{\partial \pi}{\partial \mathbf{P}_c} \mathbf{R} = - \begin{bmatrix} \frac{f_x}{Z'} & 0 & -\frac{f_x X'}{Z'^2} \\ 0 & \frac{f_y}{Z'} & -\frac{f_y Y'}{Z'^2} \end{bmatrix} \mathbf{R}
    $$

## 4. 局部建图与 BA 优化 (Local Mapping & Bundle Adjustment)

### 4.1 局部光束法平差 (Local Bundle Adjustment)

局部建图线程维护一个局部关键帧集合 $\mathcal{K}_L$ 和局部地图点集合 $\mathcal{P}_L$。Local BA 联合优化这些关键帧的位姿和地图点坐标，最小化代价函数：

$$
E = \sum_{i \in \mathcal{K}_L} \sum_{j \in \mathcal{P}_L} \rho( E_{ij}^T \Omega_{ij} E_{ij} )
$$

其中 $E_{ij} = \mathbf{u}_{ij} - \pi(\mathbf{T}_{i} \mathbf{X}_{j})$ 为重投影误差，$\rho(\cdot)$ 为 Huber 核函数以增强鲁棒性：

$$
\rho(s) = \begin{cases} s & s \le \delta^2 \\ 2\delta \sqrt{s} - \delta^2 & s > \delta^2 \end{cases}
$$

**资源上限管理**（Config.h）:
- `MAX_KEYFRAMES = 2000`: 达到上限时批量剔除 5 个最旧的
- `MAX_MAPPOINTS = 10000`: 达到上限时批量剔除 500 个最早的
- `TRACKING_MAX_LOCAL_MAP_POINTS = 5000`: 局部地图点上限

## 5. 回环检测与 Sim3 优化

### 5.1 词袋模型与相似度评分

系统利用 DBoW2 库构建视觉词典。图像 $I$ 被转换为词袋向量 $\mathbf{v} \in \mathbb{R}^W$。两帧图像 $I_a, I_b$ 的相似度通过 $L_1$ 范数计算：

$$
s(I_a, I_b) = 1 - \frac{1}{2} \left| \frac{\mathbf{v}_a}{|\mathbf{v}_a|} - \frac{\mathbf{v}_b}{|\mathbf{v}_b|} \right|_{L_1}
$$

### 5.2 Sim3 变换与尺度校正

单目 SLAM 存在尺度漂移。检测到回环时，需计算当前关键帧与回环关键帧之间的相似变换 $\mathbf{S}_{loop} \in Sim(3)$。$Sim(3)$ 包含旋转 $\mathbf{R}$、平移 $\mathbf{t}$ 和尺度因子 $s$：

$$
\mathbf{S} = \begin{bmatrix} s\mathbf{R} & \mathbf{t} \\ \mathbf{0}^T & 1 \end{bmatrix}
$$

通过 Horn 方法求解 3D-3D 对应点的初始 $\mathbf{S}$，并在 Sim3 空间进行非线性优化，最小化双向重投影误差。

#### 5.2.1 Horn's 方法求解 Sim3

给定两组 3D 点 $\{\mathbf{P}_i\}$ 和 $\{\mathbf{Q}_i\}$，目标是找到一个 Sim3 变换 $\mathbf{S} = \{s, \mathbf{R}, \mathbf{t}\}$ 使得 $\mathbf{Q}_i \approx s\mathbf{R}\mathbf{P}_i + \mathbf{t}$。

1.  计算质心:
    $$
    \bar{\mathbf{P}} = \frac{1}{N} \sum_{i=1}^N \mathbf{P}_i, \quad \bar{\mathbf{Q}} = \frac{1}{N} \sum_{i=1}^N \mathbf{Q}_i
    $$

2.  去质心化:
    $$
    \mathbf{P}'_i = \mathbf{P}_i - \bar{\mathbf{P}}, \quad \mathbf{Q}'_i = \mathbf{Q}_i - \bar{\mathbf{Q}}
    $$

3.  构建协方差矩阵:
    $$
    \mathbf{H} = \sum_{i=1}^N \mathbf{P}'_i (\mathbf{Q}'_i)^T
    $$

4.  **SVD 分解**: 对 $\mathbf{H}$ 进行 SVD 分解 $\mathbf{H} = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^T$。

5.  计算旋转矩阵 $\mathbf{R}$:
    $$
    \mathbf{R} = \mathbf{V} \mathbf{U}^T
    $$
    若 $\det(\mathbf{R}) = -1$，则将 $\mathbf{V}$ 的最后一列乘以 -1 后重新计算 $\mathbf{R}$。

6.  计算尺度因子 $s$:
    $$
    s = \frac{\sum_{i=1}^N (\mathbf{Q}'_i)^T \mathbf{R} \mathbf{P}'_i}{\sum_{i=1}^N (\mathbf{P}'_i)^T \mathbf{P}'_i}
    $$

7.  计算平移向量 $\mathbf{t}$:
    $$
    \mathbf{t} = \bar{\mathbf{Q}} - s\mathbf{R}\bar{\mathbf{P}}
    $$

**代码实现差异**: 实际代码中手动展开了所有 $3\times3$ 矩阵运算和向量点积，完全绕过了 `cv::Mat` 的内存分配和函数调用开销。

## 6. 平面检测算法 (Plane Detection System)

本项目引入了环境平面检测功能，用于 AR 对象的放置。算法基于 RANSAC 初筛 + SVD 精炼的两阶段策略。

### 6.1 算法流程

1. **点云筛选**: 获取当前帧可见的地图点集合 `vMPs`
2. **RANSAC 拟合**: 随机采样 3 点确定平面候选，统计内点数，迭代 50 次（`PLANE_DETECT_RANSAC_ITERS`），选择内点最多的平面模型参数 $ax+by+cz+d=0$
3. **SVD 精炼**: 对内点集构建矩阵 $\mathbf{A} \in \mathbb{R}^{N \times 4}$，其中第 $i$ 行为 $[\mathbf{P}_i^T, 1]$，进行 SVD 分解 $\mathbf{A} = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^T$，最优平面参数 $\mathbf{n} = [a,b,c,d]^T$ 为 $\mathbf{V}$ 中最小奇异值对应的列向量
4. **方向修正**: 确保法向量指向相机（使用首次观测位姿 `mTcw`）
5. **坐标系构建**: 利用 Rodrigues 公式构建从世界坐标系到平面坐标系的变换矩阵 $\mathbf{T}_{pw}$

### 6.2 坐标系对齐与 SO(3) 指数映射

为了便于 AR 渲染，需构建从世界坐标系到平面坐标系的变换 $\mathbf{T}_{pw}$，使平面的法向量 $\vec{n}$ 对齐到 Y 轴（或规定的 Up 轴）$\vec{up} = [0, 1, 0]^T$。

首先归一化法向量 $\vec{n} = \vec{n} / \|\vec{n}\|$。然后计算旋转轴 $\mathbf{v}$ 和旋转角 $\theta$：

$$
\mathbf{v} = \vec{n} \times \vec{up}, \quad \theta = \arctan2(\|\mathbf{v}\|, \vec{n} \cdot \vec{up})
$$

归一化旋转轴得到单位向量 $\mathbf{k}$：

$$
\mathbf{k} = \frac{\mathbf{v}}{\|\mathbf{v}\|}
$$

利用罗德里格斯公式（Rodrigues' Rotation Formula），即 $\mathfrak{so}(3)$ 到 $SO(3)$ 的指数映射，计算旋转矩阵 $\mathbf{R}_{pw}$：

$$
\mathbf{R}_{pw} = \exp(\theta [\mathbf{k}]_{\times}) = \mathbf{I} + \sin\theta [\mathbf{k}]_{\times} + (1-\cos\theta) [\mathbf{k}]_{\times}^2
$$

其中 $[\mathbf{k}]_{\times}$ 为单位向量 $\mathbf{k}$ 的反对称矩阵：

$$
[\mathbf{k}]_{\times} = \begin{bmatrix} 0 & -k_z & k_y \\ k_z & 0 & -k_x \\ -k_y & k_x & 0 \end{bmatrix}
$$

**特殊情况处理**:
- 若 $\vec{n} \cdot \vec{up} \approx 1$：法向量已对齐，$\mathbf{R}_{pw} = \mathbf{I}$
- 若 $\vec{n} \cdot \vec{up} \approx -1$：法向量与目标方向相反，需选择任意垂直轴进行 $180^\circ$ 旋转

### 6.3 平面追踪与 AR 上下文

JNI 层维护了每个地图的独立平面和 AR 对象状态：

```cpp
std::map<int, Plane*> gMapPlanes;          // 地图 ID → 平面
std::map<int, std::vector<ArObjectInfo>> gMapArObjects;  // 地图 ID → AR 对象列表
```

平面来源有两种：
- **手动检测**: 用户点击触发 `detectPlane()`，`planeLoadedFromMap = false`，可直接显示 AR
- **地图加载**: 从 `.arinfo` 文件恢复，`planeLoadedFromMap = true`，需要对齐成功才能显示

## 7. 核心算法：改进的 ORB 特征提取

本系统采用了经过深度优化的 ORB (Oriented FAST and Rotated BRIEF) 算法。本节详细阐述特征提取、方向分配及描述子生成的数学模型。

### 7.1 图像金字塔构建

为了实现尺度不变性，系统构建了包含 $n_{levels}$ 层的图像金字塔。设 $I_0$ 为原始图像，第 $l$ 层图像 ($l \in \{0, \dots, n_{levels}-1\}$) 记为 $I_l$。层间尺度因子为 $s > 1$（Config.h: `ORB_EXTRACTOR_SCALE_FACTOR = 1.2f`，`ORB_EXTRACTOR_N_LEVELS = 8`）。

第 $l$ 层的尺度为：
$$
\sigma_l = s^l
$$

为了保证单位面积内的特征密度恒定，第 $l$ 层的目标特征提取数量 $N_l$ 按照面积尺度的倒数进行分配：

$$
N_l = N_{total} \frac{1 - s^{-2}}{1 - (s^{-2})^{n_{levels}}} (s^{-2})^l
$$

### 7.2 均匀 FAST 检测与双阈值网格搜索

标准的 FAST 算法容易产生特征点聚类的问题。本实现采用了 **基于网格的双阈值 (Grid-based Dual-Threshold)** 策略：

1. **网格划分**：将图像 $I_l$ 划分为 $W \times W$ 的网格单元 (通常 $30 \times 30$ 像素)
2. **自适应阈值**：对于每个网格，系统首先尝试使用高阈值 `iniThFAST = 20` 检测角点
3. **回退机制**：如果网格内角点数量 ≤ 3，系统自动降低阈值至 `minThFAST = 7` 并重试

FAST 角点判定准则为：以像素 $I_p$ 为中心，半径为 3 的圆周上 (Bresenham 圆，16个像素)，存在连续 $n$ 个像素的亮度均大于 $I_p + T$ 或均小于 $I_p - T$。

### 7.3 四叉树特征分布 (Quadtree Feature Distribution)

为了保证最大的信息熵和最优的空间分布，原始 FAST 点集需经过 **四叉树分布** 处理：

1. **初始化**：根节点覆盖包含所有候选点 $\mathcal{P}_{raw}$ 的图像区域
2. **递归分裂**：若节点内的点集 $|\mathcal{P}_n| > 1$，则将其分裂为 4 个子象限
3. **平衡策略**：系统迭代选择包含点数最多的节点进行分裂，扩展树结构直至叶节点数量等于目标特征数 $N_l$
4. **极大值抑制与选择**：从每个最终叶节点中，选择具有最大 FAST 响应值的唯一点

该算法在数学上近似实现了泊松盘采样 (Poisson Disk Sampling)，确保特征点间距满足：
$$
\forall i, j \in \mathcal{K}, \quad \|\mathbf{x}_i - \mathbf{x}_j\|_2 > d_{min}
$$

### 7.4 方向计算：灰度质心法 (Intensity Centroid)

为了提供旋转不变性，系统使用灰度质心法为每个关键点分配主方向。
对于以关键点为中心的图像块 $P$，定义矩 $m_{pq}$ 为：

$$
m_{pq} = \sum_{x,y \in P} x^p y^q I(x,y)
$$

图像块的质心 $\mathbf{C}$ 为：
$$
\mathbf{C} = \left( \frac{m_{10}}{m_{00}}, \frac{m_{01}}{m_{00}} \right)
$$

方向向量由几何中心 $\mathbf{O}$ 指向质心 $\mathbf{C}$。方向角 $\theta$ 计算如下：

$$
\theta = \text{atan2}(m_{01}, m_{10})
$$

**实现优化**: 代码中利用圆形的中心对称性将 $m_{10}$ 和 $m_{01}$ 的计算量减半，通过预先计算的 `umax` 向量避免 `sqrt` 边界判断。

### 7.5 描述子：Steered BRIEF

描述子采用二进制字符串描述局部图像块。我们使用预先训练好的 256 个点对模式 $(\mathbf{p}_i, \mathbf{q}_i)$。
为了实现旋转不变性，这些坐标通过关键点角度 $\theta$ 进行 "转向" (Steering)。

旋转矩阵 $\mathbf{R}_\theta$ 为：
$$
\mathbf{R}_\theta = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
$$

第 $i$ 对点的转向坐标为：
$$
\mathbf{S}_{\theta}(\mathbf{p}_i) = \mathbf{R}_\theta \mathbf{p}_i, \quad \mathbf{S}_{\theta}(\mathbf{q}_i) = \mathbf{R}_\theta \mathbf{q}_i
$$

定义的二进制测试 $\tau$ 为：
$$
\tau(\mathbf{p}, \mathbf{q}; \theta) = \begin{cases} 1 & \text{if } I(\mathbf{x} + \mathbf{S}_{\theta}(\mathbf{p})) < I(\mathbf{x} + \mathbf{S}_{\theta}(\mathbf{q})) \\ 0 & \text{otherwise} \end{cases}
$$

最终生成的 256 位 (32字节) 描述子 $D(\mathbf{x})$ 为：
$$
D(\mathbf{x}) = \sum_{i=0}^{255} 2^i \tau(\mathbf{p}_i, \mathbf{q}_i; \theta)
$$

**性能优化**: 三角函数查找表 (360 项量化) + 循环展开 (8 对比较/迭代) + 宏内联。

### 7.6 特征匹配：汉明距离 (Hamming Distance)

由于 ORB 描述子是二进制向量，特征匹配的相似度度量采用汉明距离，即两个二进制串中不同位的个数。
对于两个 256 位的描述子 $D_a$ 和 $D_b$，其汉明距离 $d_H$ 定义为：

$$
d_H(D_a, D_b) = \sum_{i=0}^{255} (D_a^{(i)} \oplus D_b^{(i)}) = \text{popcount}(D_a \oplus D_b)
$$

其中 $\oplus$ 表示按位异或 (XOR) 运算，$\text{popcount}(\cdot)$ 表示计算二进制中置位 (1) 的个数。代码中使用 OpenCV 的 `cv::norm(a, b, cv::NORM_HAMMING)`，自动利用 NEON SIMD 指令集加速。

## 8. 系统架构与执行流程 (System Architecture & Execution Flow)

### 8.1 模块化结构

项目由三个 Gradle 模块组成：

```
app/                          # Java UI + OpenGL 渲染 (零 C++ 代码)
VtonaxAR/                     # 所有 C++ 代码 + JNI 桥接
OpenCVLibrary/                # OpenCV Android SDK 封装
```

所有 C++ 代码集中在 `VtonaxAR` 模块中，编译产物为三个 `.so`：
- `libSLAM_AR.so` — SLAM 核心 + JNI + DBoW2 + 平面检测（单体库）
- `libg2o.a` — 图优化静态库
- `lib3dof.so` — 独立 3DoF 跟踪库

### 8.2 系统启动与初始化

1. **Java 层**: `ArCamUIActivity.onCreate()` → 异步调用 `NativeHelper.initSLAM()`
2. **JNI 层**: `initSLAM()` → 设置内参、预计算投影矩阵、初始化分析器、创建 `System` 对象
3. **资源加载**: ORB 词汇表优先从嵌入式二进制（`.incbin` 汇编指令嵌入）加载，无需文件 IO
4. **线程创建**:
    - **Tracking (主线程)**: 处理每一帧图像，实时位姿估计
    - **LocalMapping (后台)**: 关键帧处理与局部地图优化
    - **LoopClosing (后台)**: 回环检测与位姿图优化
    - **GlobalReloc (新增后台)**: 异步全局重定位，与加载地图对齐
5. **相机启动**: SLAM 初始化完成后才启动 CameraX 预览

### 8.3 主跟踪线程流程 (Tracking Thread Flow)

```
预处理 → 特征提取 → 状态判断 → 位姿跟踪 → 异步融合 → 关键帧决策
```

1. **预处理**: 半分辨率下采样（面积 1/4）、检查 SLAM 开关、检查重置请求
2. **特征提取**: 8 层金字塔 ORB 提取，每帧 1000 个特征点
3. **状态判断与初始化**:
    - `NOT_INITIALIZED` → 并行 H/F 初始化
    - `OK` → 正常跟踪
    - `LOST` → 尝试重定位，超时 3 秒后 `Reset(true)`
4. **位姿跟踪**:
    - **恒速模型跟踪**: 预测位姿 → 投影匹配 → Motion-only BA
    - **参考帧跟踪**: BoW 加速匹配 → PnP 求解
5. **异步重定位融合**:
    - 检查后台 `GlobalReloc` 是否计算出对齐变换 $\mathbf{T}_{map\_slam}$
    - 若存在则消耗结果、修正位姿、执行 `BindLoadedMapPoints`
6. **关键帧决策**: 根据特征点数量、帧间隔、LocalMapping 负载决定是否插入关键帧

### 8.4 状态机定义

| 状态 | 含义 | 处理动作 |
|------|------|---------|
| `SYSTEM_NOT_READY (-1)` | 词汇表未加载 | 等待 |
| `NO_IMAGES_YET (0)` | 等待图像 | 跳过 |
| `NOT_INITIALIZED (1)` | 未初始化 | 并行 H/F 初始化 |
| `OK (2)` | 正常跟踪 | 位姿估计 + 局部地图 |
| `LOST (3)` | 跟踪丢失 | 重定位 / Reset(true) |

## 9. 异步全局重定位与地图持久化

### 9.1 异步全局重定位算法 (Asynchronous Global Relocalization)

这是本项目区别于原始 ORB-SLAM2 的核心创新。通过独立的 `GlobalReloc` 后台线程实现当前 SLAM 系统与加载地图的**无缝对齐**。

#### 参考缓存构建

加载地图后，系统构建三种索引结构以支持高效匹配：

**倒排索引** (`mRefInverted`):
```
WordID → [MapPoint_Index_1, MapPoint_Index_2, ...]
```
基于 DBoW2 词汇的哈希表，加速候选检索。

**空间网格索引** (`mRefGrid`):
```cpp
struct LoadedMapGrid {
    float cellSize = 10.0f;  // 默认 10m 网格
    std::vector<std::vector<int>> cells;
    void GetCandidatesInSphere(center, radius, snaps, outIndices);
};
```
3D 空间划分，支持圆形范围精确过滤。

**不可变快照** (`mRefSnapshots`):
```cpp
struct RefMPSnapshot {
    cv::Point3f Pw;     // 世界坐标
    float minD, maxD;   // 深度范围
    int mapId;          // 地图 ID
};
```
避免后台线程与主线程的竞态条件。

#### 快照机制

主线程与后台线程通过**无锁原子版本号**协调：

```cpp
std::atomic<unsigned long long> mSnapSeqProduced{0ULL};  // 生产端
std::atomic<unsigned long long> mSnapSeqConsumed{0ULL};  // 消费端
```

- **生产**: 主线程每帧将描述子、关键点、位姿打包为快照，递增 `mSnapSeqProduced`
- **消费**: 后台线程通过条件变量实时唤醒，获取最新快照进行处理

#### 对齐流程

1. **BoW 检索**: 使用倒排索引在加载地图中检索 Top-K 候选（Config: `SYSTEM_RELOC_CONFIG_TOP_K = 20`）
2. **特征匹配**: KNN + Ratio Test 对当前帧与候选点进行描述子匹配
3. **PnP 求解**: EPnP + RANSAC 求解当前帧在加载地图坐标系下的位姿
4. **对齐发布**: 通过 `PublishRelocAlignment` 发布对齐结果 `T_map_from_slam`
5. **平滑更新** (主线程消费): EMA 平滑、跳帧（每 3 帧更新）、SVD 正交化修正

#### 智能调度

后台线程根据跟踪质量动态调整运行频率：
- **跟踪稳定** (内点 > 100): 低频运行，减少 CPU 争用
- **跟踪不稳定** / **未对齐**: 全速运行（80μs 睡眠间隔）
- 跟踪丢失时后台线程停止，避免浪费计算资源

### 9.2 安全 PnP 求解器 (Robust Safe PnP)

为了应对恶劣环境下的数据异常，`PnPsolver` 实现了额外的鲁棒性保障：

- **数据清洗**: 严格检查 3D-2D 点对的 NaN/Inf 及坐标范围限制（`PNP_LIMIT_2D = 1e5`, `PNP_LIMIT_3D = 1e6`）
- **策略分级**:
    - $N < 4$: 直接返回失败
    - $4 \le N < 6$: 跳过 RANSAC，仅做最小二乘
    - $N \ge 6$: 启用 RANSAC（200 次迭代、误差阈值 6.0、置信度 0.999）
- **异常捕获**: 封装 OpenCV 调用，捕获内部断言错误，防止程序崩溃

### 9.3 地图持久化格式 (Map Persistence Format)

系统采用自定义二进制格式存储地图：

**SLAM 地图文件 (`.bin`)**:
- 魔数: `0x4D415031` ("MAP1")
- 版本: 1
- 关键帧块: ID、时间戳、位姿 $\mathbf{T}_{cw}$、所有关键点（坐标、尺度、角度、描述子）
- 地图点块: 世界坐标 $\mathbf{P}_w$、法向量、代表性描述子、深度范围 $[d_{min}, d_{max}]$、`mbFromLoadedMap` 标记

**AR 信息文件 (`.arinfo`)**:
- 魔数: `0x4152494E` ("ARIN")
- 版本: 1
- 平面信息: 原点、法向量、旋转角
- AR 对象列表: 每个对象包含模型矩阵和 ID

**元数据文件 (`.json`)**（Java 层 `MapManager`）:
```json
{"name": "map_0521", "keyFrames": 42, "mapPoints": 1500,
 "createTime": 1716288000000, "hasPlane": true, "fileSize": 204800}
```

### 9.4 多地图管理

```cpp
std::vector<Map*> mvpMaps;           // 多地图容器
void CreateNewMap();                  // 创建子地图
void SwitchToMap(Map* pMap);          // 切换地图
void LoadMap(path, mapId, append);    // 加载(可追加)
```

| 模式 | 说明 |
|------|------|
| `LoadMap(path, 0, false)` | 默认覆盖模式 |
| `LoadMap(path, id, false)` | 指定 ID 加载 |
| `LoadMap(path, id, true)` | 追加模式，保留现有地图 |

子地图创建条件: 连续丢失 30 帧，冷却期 150 帧，最大 10 个子地图。

地图切换由 JNI 层检测 Map ID 变化，经 `MAP_SWITCH_THRESHOLD = 3` 帧确认后自动切换 AR 上下文。

## 10. 性能优化策略 (Performance Optimization Strategies)

### 10.1 ORB 特征提取优化

#### 10.1.1 三角函数查找表优化 (Trigonometric Lookup Table)

**问题**: 描述子计算中需要对每个关键点进行旋转变换，频繁调用 `sin()` 和 `cos()` 成为性能瓶颈。

**方案**: 预计算 `sineTable[361]` 和 `cosineTable[361]`，角度量化为整数索引。量化误差 $\epsilon < 0.5°$，对描述子鲁棒性影响可忽略。

**性能提升**: 消除了描述子计算中的所有超越函数调用。

#### 10.1.2 灰度质心方向计算优化 (IC_Angle Optimization)

1. **对称性利用**: 利用圆形补丁的中心对称性减少一半计算量：
    ```cpp
    // 利用中心线对称性 (v=0)
    for (int u = 1; u <= ORB_HALF_PATCH_SIZE; ++u)
        m_10 += u * (center[u] - center[-u]);
    ```

2. **四点同时处理**: 每个 $(u,v)$ 位置同时处理四个对称点 $(u,v), (-u,v), (u,-v), (-u,-v)$

3. **边界预计算 (`umax`)**: 预先计算圆形补丁每一行的最大横坐标，避免 `sqrt` 运算

#### 10.1.3 描述子计算向量化

使用位操作和循环展开加速二进制测试，将 8 对比较操作打包为单个字节：

```cpp
for (int i = 0; i < 32; ++i, pattern += 16) {
    int t0, t1, val;
    t0 = GET_VALUE(0); t1 = GET_VALUE(1);  val = t0 < t1;
    t0 = GET_VALUE(2); t1 = GET_VALUE(3);  val |= (t0 < t1) << 1;
    // ... 8对比较展开
    desc[i] = (uchar)val;
}
```

### 10.2 特征匹配优化

#### 10.2.1 汉明距离硬件加速

使用 OpenCV 的 `cv::norm(a, b, cv::NORM_HAMMING)`，在 ARM 上自动调用 NEON SIMD 指令集（`vcnt`）。

#### 10.2.2 延迟平方根计算

先进行平方距离比较，仅在通过筛选后计算实际距离。

#### 10.2.3 整数乘法替代浮点运算

```cpp
// 原始: if(max2 < 0.1f * max1)
// 优化: 整数乘法
if(max2 * 10 < max1) { ind2=-1; ind3=-1; }
```

### 10.3 内存与数学运算优化

- **内存预分配**: vector `reserve()` 避免扩容开销
- **图像金字塔边界填充**: 预分配带边界的完整图像，避免边界条件分支
- **逆内参缓存**: `invfx`, `invfy` 将除法转为乘法
- **中值描述子**: `std::nth_element` ($O(N)$) 替代全排序 ($O(N \log N)$)

### 10.4 嵌入式资源 (Embedded Resources)

ORB 词汇表和 LUT 通过 CMake 的 `.incbin` 汇编指令直接嵌入 `.so`：

```cmake
embed_resource(ORBvoc.txt.arm.bin ORBvoc_txt_arm_bin ${EMBEDDED_DIR}/ORBvoc.o)
embed_resource(ORB_LUT.bin ORB_LUT_bin ${EMBEDDED_DIR}/ORB_LUT.o)
```

链接时作为目标文件直接链接，运行时零文件 IO。

### 10.5 Top-K 地图点 UI 同步

使用 `std::partial_sort` 仅筛选 ID 最大的前 N 个地图点，最小化 JNI 传输和 OpenGL 渲染开销：

```cpp
std::partial_sort(v.begin(), v.begin() + maxPoints, v.end(), MapPointComparator());
// 只取前 maxPoints 个
```

### 10.6 线程安全的分层锁设计

| 互斥锁 | 保护数据 | 策略 |
|--------|---------|------|
| `gSlamStateMutex` | SLAM 系统指针、跟踪状态 | 跟踪线程必须获得，不可阻塞 |
| `gMapDataMutex` | 平面、AR 对象、地图数据 | 短暂锁定，细粒度更新 |
| `gMapPointsMutex` | 地图点/关键点缓存 | UI 线程使用 `try_lock` 无阻塞 |

### 10.7 SLAM 运行时开关

`setEnableSLAM(false)` 时完全跳过 TrackMonocular、特征提取等全部计算：
```cpp
if (!gEnableSLAM) {
    status = 0;  // NO_IMAGES_YET
    vMPs.clear(); vKeys.clear();
    gShouldDrawArObject = false;
}
```

### 10.8 Vtonax 性能分析器

可选工具，由 `VTONAX_DEVELOP_MODE` 控制（默认关闭）。开启后提供：
- RAII 自动计时 (`VT_PROFILE_FUNCTION()`)
- 纳秒级高精度时间戳
- Chrome Trace Event 格式输出（`chrome://tracing` 可视化）

关闭时所有宏展开为空，零运行时开销。

### 10.9 半分辨率处理

核心移动端优化。在 JNI 层使用 `cv::resize` 将图像长宽各缩小一半（面积 1/4），极大地减少了像素级操作的计算量。

### 10.10 编译期优化

```cmake
-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing -fno-lto
-DANDROID_ARM_NEON=TRUE
```

### 10.11 优化总结表

| 优化模块 | 优化手段 | 核心算法 | 影响 |
|---------|---------|---------|------|
| 特征提取 | 三角函数 LUT | 预计算查找表 | CPU 大幅减负 |
| 特征提取 | IC_Angle 对称性 | 圆形对称优化 | 减少 50% 乘法 |
| 特征匹配 | 词袋加速 | 倒排索引 | 极速匹配 |
| 特征匹配 | 旋转一致性 | 直方图统计 | 高效外点剔除 |
| 跟踪 | 轻量级投影 | 矩阵手动展开 | 减少内存分配 |
| 位姿求解 | 安全 PnP | EPnP + RANSAC | 数值鲁棒性 |
| 后台重定位 | 原子版本号 | 无锁快照 | 零阻塞 |
| 图像处理 | 半分辨率 | 面积 1/4 降采样 | 移动端关键优化 |
| 资源加载 | `.incbin` 嵌入 | 汇编指令嵌入 | 零文件 IO |
| UI 同步 | Top-K 偏序 | `partial_sort` | 最小化传输 |

---

*本文档基于 Android_ORB-SLAM2s 项目源码编写，反映了截至 2026 年 5 月的实际实现状态。*
*文档维护者: Olsc*
