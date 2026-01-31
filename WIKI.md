# ORB-SLAM2s: Android 平台轻量化空间计算与增强现实系统

（此文档仍在审查中）

## 摘要

本文档详细阐述了 ORB-SLAM2s 的数学原理与核心算法实现。作为一个基于 Android 平台的轻量化空间计算系统，本项目在经典 ORB-SLAM2 框架的基础上，进一步集成了即时平面检测、地图持久化与各向异性尺度下的重定位算法。本文将严谨地从数学角度描述系统的各个模块，包括基于李代数的位姿估计、光束法平差（Bundle Adjustment）、词袋模型（Bag of Words）的回环检测以及基于奇异值分解（SVD）的平面拟合算法。

## 1. 引言

同步定位与地图构建（SLAM）是移动机器人与增强现实（AR）领域的核心技术。ORB-SLAM2s 旨在移动端有限的计算资源下，通过单目相机实现鲁棒的 6DoF 轨迹跟踪与稀疏环境重建。系统采用多线程架构，并行处理跟踪、局部建图与回环检测任务。

## 2. 符号约定

*   **世界坐标系** (World frame): $\mathcal{F}_w$
*   **相机坐标系** (Camera frame): $\mathcal{F}_c$
*   **相机内参矩阵**: $\mathbf{K}$
*   **李群与李代数**: 特殊欧几里得群 $SE(3)$ 与其对应的李代数 $\mathfrak{se}(3)$。
    *   旋转矩阵 $\mathbf{R} \in SO(3)$，平移向量 $\mathbf{t} \in \mathbb{R}^3$。
    *   变换矩阵 $\mathbf{T}_{cw} \in SE(3)$ 表示从世界坐标系到相机坐标系的变换。
*   **地图点**: $\mathbf{X}_w \in \mathbb{R}^3$ 表示世界坐标系下的 3D 点。

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

其中 $f_x, f_y$ 为焦距，$c_x, c_y$ 为主点坐标。

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
    其中 $\rho(\cdot)$ 为卡方检验阈值截断函数。若 $S_H / (S_H + S_F) > 0.45$，选择单应性模型；否则选择基础矩阵模型。

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

#### 5.2.2 Sim3 非线性优化

在获得初始 Sim3 变换后，通过最小化双向重投影误差进行非线性优化。对于一对匹配的 3D 点 $\mathbf{X}_k$ (在关键帧 $K_k$ 坐标系下) 和 $\mathbf{X}_l$ (在关键帧 $K_l$ 坐标系下)，以及它们在各自关键帧中的观测 $\mathbf{u}_k, \mathbf{u}_l$，我们定义误差函数：

$$
E_{Sim3} = \sum_{k,l \in \mathcal{M}} \left( \rho( (\mathbf{u}_k - \pi(\mathbf{K} \mathbf{X}_k))^T \Omega_k (\mathbf{u}_k - \pi(\mathbf{K} \mathbf{X}_k)) ) + \rho( (\mathbf{u}_l - \pi(\mathbf{K} \mathbf{X}_l))^T \Omega_l (\mathbf{u}_l - \pi(\mathbf{K} \mathbf{X}_l)) ) \right)
$$

其中，$\mathbf{X}_l = s\mathbf{R}\mathbf{X}_k + \mathbf{t}$。优化变量为 Sim3 变换的李代数表示 $\boldsymbol{\xi}_{Sim3} \in \mathfrak{sim}(3)$。

$$
\mathbf{e}_{Sim3} = \mathbf{u}_{obs} - \pi( s\mathbf{R}\mathbf{P}_w + \mathbf{t} )
$$

## 6. 平面检测算法 (Plane Detection System)
### （注意！这一段在代码里的实现并不完全相同！）

本项目引入了环境平面检测功能，用于 AR 对象的放置。算法通过奇异值分解（SVD）拟合最优平面，并计算局部平面坐标系。

### 6.1 SVD 平面拟合

给定 $N$ 个地图点 $\mathbf{P}_i = [x_i, y_i, z_i]^T$，构建矩阵 $\mathbf{A} \in \mathbb{R}^{N \times 4}$，其中第 $i$ 行数据为 $[\mathbf{P}_i^T, 1]$。平面方程 $ax+by+cz+d=0$ 的参数 $\mathbf{n} = [a,b,c,d]^T$ 可通过求解 $\mathbf{A}\mathbf{n}=\mathbf{0}$ 获得。

对 $\mathbf{A}$ 进行奇异值分解：

$$
\mathbf{A} = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^T
$$

最优平面参数 $\mathbf{n}$ 即为 $\mathbf{V}$ 中对应于最小奇异值的列向量。归一化法向量 $\vec{n} = [a,b,c]^T$.

平面的几何中心（质心）$\mathbf{O}_{plane}$ 为所有输入点的均值：

$$
\mathbf{O}_{plane} = \frac{1}{N} \sum_{i=1}^N \mathbf{P}_i
$$

### 6.2 坐标系对齐与 SO(3) 指数映射

为了便于 AR 渲染，需构建从世界坐标系到平面坐标系的变换 $\mathbf{T}_{pw}$，使平面的法向量 $\vec{n}$ 对齐到 Y 轴（或规定的 Up 轴）$\vec{up} = [0, 1, 0]^T$。

首先归一化法向量 $\vec{n} = \vec{n} / \|\vec{n}\|$。然后计算旋转轴 $\mathbf{v}$ 和旋转角 $\theta$：

$$
\mathbf{v} = \vec{n} \times \vec{up}, \quad \theta = \arctan2(\|\mathbf{v}\|, \vec{n} \cdot \vec{up})
$$

**注意**：旋转轴的计算顺序为 $\vec{n} \times \vec{up}$（从 $\vec{n}$ 旋转到 $\vec{up}$），而非 $\vec{up} \times \vec{n}$。

归一化旋转轴得到单位向量 $\mathbf{k}$：

$$
\mathbf{k} = \frac{\mathbf{v}}{\|\mathbf{v}\|}
$$

利用罗德里格斯公式（Rodrigues' Rotation Formula），即 $\mathfrak{so}(3)$ 到 $SO(3)$ 的指数映射，计算旋转矩阵 $\mathbf{R}_{pw}$：

$$
\mathbf{R}_{pw} = \exp(\theta [\mathbf{k}]_{\times}) = \mathbf{I} + \sin\theta [\mathbf{k}]_{\times} + (1-\cos\theta) [\mathbf{k}]_{\times}^2
$$

其中 $[\mathbf{k}]_{\times}$ 为单位向量 $\mathbf{k}$ 的反对称矩阵（斜对称矩阵）：

$$
[\mathbf{k}]_{\times} = \begin{bmatrix} 0 & -k_z & k_y \\ k_z & 0 & -k_x \\ -k_y & k_x & 0 \end{bmatrix}
$$

**特殊情况处理**：

*   若 $\vec{n} \cdot \vec{up} \approx 1$：法向量已对齐，$\mathbf{R}_{pw} = \mathbf{I}$。
*   若 $\vec{n} \cdot \vec{up} \approx -1$：法向量与目标方向相反，需选择任意垂直轴（如 $[1,0,0]^T$）进行 $180^\circ$ 旋转。

最终变换矩阵 $\mathbf{T}_{pw}$ 结合了该旋转与质心平移，确立了以检测平面为基准的局部坐标系。

## 7. 核心算法：改进的 ORB 特征提取

本系统采用了经过深度优化的 ORB (Oriented FAST and Rotated BRIEF) 算法。本节详细阐述特征提取、方向分配及描述子生成的数学模型。

### 7.1 图像金字塔构建

为了实现尺度不变性，系统构建了包含 $n_{levels}$ 层的图像金字塔。设 $I_0$ 为原始图像，第 $l$ 层图像 ($l \in \{0, \dots, n_{levels}-1\}$) 记为 $I_l$。层间尺度因子为 $s > 1$ (通常取 1.2)。

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

1.  **网格划分**：将图像 $I_l$ 划分为 $W \times W$ 的网格单元 (通常 $30 \times 30$ 像素)。
2.  **自适应阈值**：对于每个网格，系统首先尝试使用高阈值 $T_{high}$ (如 `iniThFAST`) 检测角点。
3.  **回退机制**：如果在高阈值下未在网格内发现角点，系统自动降低阈值至 $T_{low}$ (如 `minThFAST`) 并重试。这确保了在弱纹理区域仍能提取到候选点，防止跟踪丢失。

FAST 角点判定准则为：以像素 $I_p$ 为中心，半径为 3 的圆周上 (Bresenham 圆，16个像素)，存在连续 $n$ 个像素的亮度均大于 $I_p + T$ 或均小于 $I_p - T$。

### 7.3 四叉树特征分布 (Quadtree Feature Distribution)

为了保证最大的信息熵和最优的空间分布，原始 FAST 点集需经过 **四叉树分布** 处理：

1.  **初始化**：根节点覆盖包含所有候选点 $\mathcal{P}_{raw}$ 的图像区域。
2.  **递归分裂**：若节点内的点集 $|\mathcal{P}_n| > 1$，则将其分裂为 4 个子象限。
3.  **平衡策略**：系统迭代选择包含点数最多的节点进行分裂，扩展树结构直至叶节点数量等于目标特征数 $N_l$。
4.  **极大值抑制与选择**：从每个最终叶节点中，选择具有最大 FAST 响应值 (Harris 评分或强度差之和) 的唯一点。

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

*注：实现中使用了圆形掩膜 (通过 `umax` 向量) 以确保旋转严格遵守图像块的几何形状。*

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

这种方法生成了鲁棒的二进制特征向量，支持极速的汉明距离匹配。

### 7.6 特征匹配：汉明距离 (Hamming Distance)

由于 ORB 描述子是二进制向量，特征匹配的相似度度量采用汉明距离，即两个二进制串中不同位的个数。
对于两个 256 位的描述子 $D_a$ 和 $D_b$，其汉明距离 $d_H$ 定义为：

$$
d_H(D_a, D_b) = \sum_{i=0}^{255} (D_a^{(i)} \oplus D_b^{(i)}) = \text{popcount}(D_a \oplus D_b)
$$

其中 $\oplus$ 表示按位异或 (XOR) 运算，$D^{(i)}$ 表示第 $i$ 位的值，$\text{popcount}(\cdot)$ 表示计算二进制中置位 (1) 的个数。

## 8. 系统架构与执行流程 (System Architecture & Execution Flow)

本项目在原 ORB-SLAM2 基础上进行了重大改进，引入了地图持久化、多地图管理及异步重定位机制。本节详细描述系统的核心执行流程。

### 8.1 系统启动与初始化

1.  **资源加载**: 系统首先加载 ORB 词汇表（支持文件加载与嵌入式二进制加载），这是闭环检测与全局重定位的基础。
2.  **线程创建**: 
    *   **Tracking (主线程)**: 负责处理每一帧图像，进行实时位姿估计。
    *   **LocalMapping**: 负责关键帧处理与局部地图优化。
    *   **LoopClosing**: 负责全局闭环检测与位姿图优化。
    *   **GlobalReloc (新增后台线程)**: 负责在后台异步运行全局重定位算法，实现当前 SLAM 系统与持久化地图的自动对齐。
3.  **地图加载 (Map Loading)**:
    *   系统支持加载预先构建的 `.map` 二进制文件。
    *   加载过程通过 `BuildLoadedRefCache` 构建参考地图的特征索引（倒排索引与空间网格索引），为高效匹配做准备。

### 8.2 主跟踪线程流程 (Tracking Thread Flow)

主线程 `System::TrackMonocular` 驱动整个系统的运转，其核心逻辑流程如下：

1.  **预处理**: 图像转灰度，检查模式切换（定位模式/SLAM模式）与重置请求。
2.  **特征提取**: 对当前帧提取 ORB 特征，构建图像金字塔。
3.  **状态判断与初始化**:
    *   若状态为 `NOT_INITIALIZED`，进入单目初始化流程（参见 3.5 节）。
    *   若初始化成功，创建初始地图与关键帧。
4.  **位姿跟踪 (Pose Tracking)**:
    *   **参考帧跟踪/恒速模型跟踪**: 根据上一帧状态，利用参考关键帧匹配或恒速运动模型预测当前位姿。
    *   **局部地图跟踪 (TrackLocalMap)**: 搜索当前视锥体内的局部地图点，通过投影匹配与位姿优化（Motion-only BA）精炼位姿。
5.  **异步重定位融合 (Async Reloc Fusion)**:
    *   系统检查后台 `GlobalReloc` 线程是否计算出了当前 SLAM 坐标系与加载地图坐标系的对齐变换 $\mathbf{T}_{map\_slam}$。
    *   若存在对齐变换，主线程将消耗该结果，修正当前位姿，并状态切换为 "已对齐"。
    *   **快照绑定 (BindLoadedMapPoints)**: 在已对齐状态下，系统将加载地图中的点（Loaded MapPoints）投影到当前帧进行匹配，实现当前序列与历史地图的融合。
6.  **关键帧决策**: 根据当前帧的特征点数量、距离上一关键帧的间隔及局部建图线程的负载，决定是否插入新的关键帧。

## 9. 异步全局重定位与地图持久化

### 9.1 异步全局重定位算法 (Asynchronous Global Relocalization)

传统的重定位通常在跟踪丢失（Lost）时阻塞式运行。本项目设计了并行运行的 `GlobalReloc` 策略，旨在实现“无缝”的地图合并与重定位。

**算法流程**:

1.  **快照生成 (Snapshot)**: 主线程将当前帧的描述子、关键点及临时位姿打包成“快照”，放入非阻塞队列。
2.  **智能调度**: 后台线程监控跟踪质量。若当前跟踪稳定（内点丰富），降低运行频率以节省计算资源；若跟踪不稳定或尚未对齐，则全速运行。
3.  **候选帧检索**:
    *   利用 BoW 倒排索引，在加载的参考地图中检索与当前帧相似度最高的 $N$ 个候选位置。
    *   **空间网格搜索**: 若 BoW 检索失败但存在位姿先验，利用空间网格 (Spatial Grid) 在先验位置附近直接搜索地图点。
4.  **几何验证与解算**:
    *   **特征匹配**: 对当前帧与候选地图点进行描述子匹配 (KNN + Ratio Test)。
    *   **PnP 求解**: 使用 EPnP + RANSAC 求解当前帧在参考地图坐标系下的位姿 $\mathbf{T}_{ref\_curr}$。
5.  **对齐变换计算**:
    *   若求解成功，计算对齐变换 $\mathbf{T}_{align} = \mathbf{T}_{ref\_curr} \times \mathbf{T}_{slam\_curr}^{-1}$。
    *   该变换描述了当前 SLAM 局部坐标系到全局加载地图坐标系的刚体变换关系。

### 9.2 安全 PnP 求解器 (Robust Safe PnP)

为了应对恶劣环境下的数据异常，系统实现了 `SolvePnPSafe` 算法：

*   **数据清洗**: 严格检查输入 3D-2D 点对的数值有效性（NaN/Inf 检查）及坐标范围限制，防止野值干扰。
*   **策略分级**:
    *   若点对数量 $N < 4$，直接返回失败，避免无解。
    *   若 $N < 6$，使用 P3P 或 EPnP 求解，跳过 RANSAC 迭代，仅做最小二乘优化。
    *   若 $N \ge 6$，启用 RANSAC 迭代机制，剔除误匹配外点（Outliers），确保位姿估计的鲁棒性。
*   **异常捕获**: 封装 OpenCV 调用，捕获内部断言错误（Assertion Fails），防止因数值不稳定导致的程序崩溃。

### 9.3 地图持久化格式 (Map Persistence Format)

系统采用自定义二进制格式 ("MAP1") 存储地图，实现高效的序列化与反序列化：

*   **文件头**: 魔数 (Magic Number) 与版本号，确保文件兼容性。
*   **关键帧块**:
    *   存储 ID、时间戳、位姿 $\mathbf{T}_{cw}$。
    *   **完整特征保留**: 存储所有关键点的坐标、尺度、角度及对应的 descriptors。这使得重加载的地图无需原始图像即可用于重定位匹配。
*   **地图点块**:
    *   存储世界坐标 $\mathbf{P}_w$、法向量 $\vec{n}$。
    *   存储代表性描述子 (Most Representative Descriptor) 及可视深度范围 $[d_{min}, d_{max}]$。
    *   保留 `mbFromLoadedMap` 标记，区分历史地图点与新生成的地图点。

## 10. 性能优化策略 (Performance Optimization Strategies)
### （注意！以下为代码实现部分）

本项目针对移动端有限的计算资源进行了系统性的性能优化。本节详细阐述在 ORB 特征提取、特征匹配及整体架构层面实施的优化方案。

### 10.1 ORB 特征提取优化 (ORB Extractor Optimizations)

#### 10.1.1 三角函数查找表优化 (Trigonometric Lookup Table)

**问题**: 描述子计算中需要对每个关键点进行旋转变换，频繁调用 `sin()` 和 `cos()` 函数成为性能瓶颈。

**解决方案**: 实现预计算的三角函数查找表：

```cpp
static float sineTable[361];
static float cosineTable[361];

static void InitTrigTable() {
    for(int i=0; i<=360; i++) {
        float rad = i * (float)CV_PI / 180.0f;
        sineTable[i] = sin(rad);
        cosineTable[i] = cos(rad);
    }
}
```

在描述子计算时，将角度量化为整数索引：

```cpp
int angleIdx = (int)(kpt.angle + 0.5f);
if(angleIdx < 0) angleIdx += 360;
if(angleIdx >= 360) angleIdx -= 360;

float a = cosineTable[angleIdx];
float b = sineTable[angleIdx];
```

**性能提升**: 消除了描述子计算中的超越函数调用。

**数学原理**: 利用角度的离散性，将连续函数 $\sin(\theta), \cos(\theta)$ 映射为离散查找表 $T_{sin}[k], T_{cos}[k]$，其中 $k = \lfloor \theta \cdot \frac{180}{\pi} + 0.5 \rfloor$。量化误差 $\epsilon < 0.5°$，对描述子鲁棒性影响可忽略。

#### 10.1.2 灰度质心方向计算优化 (IC_Angle Optimization)

**问题**: 原始实现中存在大量重复的内存访问和冗余计算。

**优化策略**:

1. **对称性利用**: 利用圆形补丁的中心对称性，减少一半的计算量：

```cpp
// 利用中心线对称性 (v=0)
for (int u = 1; u <= ORB_HALF_PATCH_SIZE; ++u)
    m_10 += u * (center[u] - center[-u]);
```

2. **循环展开与预计算**: 预计算行偏移量，减少地址计算：

```cpp
int offset = v * step;
const uchar* ptr_plus = center + offset;
const uchar* ptr_minus = center - offset;
```

3. **代数简化**: 利用对称性简化矩计算：

对于矩 $m_{01}$ 和 $m_{10}$：

$
m_{01} = \sum_{v=-r}^{r} v \sum_{u=-d_v}^{d_v} (I(u,v) - I(u,-v))
$

$
m_{10} = \sum_{u=-r}^{r} u \sum_{v=-d_u}^{d_u} (I(u,v) - I(-u,v))
$

其中 $d_v = \lfloor\sqrt{r^2 - v^2}\rfloor$ 为圆形边界约束。

4. **消除零项**: 在 $u=0$ 或 $v=0$ 时，某些项的贡献为零，直接跳过计算：

```cpp
// 中心列 (u=0)
v_sum += (ptr_plus[0] - ptr_minus[0]);
// m_10 在 u=0 时为 0，无需计算
```

5. **四点同时处理**: 对于每个 $(u,v)$ 位置，同时处理四个对称点 $(u,v), (-u,v), (u,-v), (-u,-v)$：

```cpp
// m_01 计算: v * sum(val_plus - val_minus)
v_sum += (val_plus_pos - val_minus_pos) + (val_plus_neg - val_minus_neg);

// m_10 计算: 利用对称性
// u * (val_pos_sum) + (-u) * (val_neg_sum) = u * (val_pos_sum - val_neg_sum)
int val_u_sum = (val_plus_pos + val_minus_pos);
int val_neg_u_sum = (val_plus_neg + val_minus_neg);
m_10 += u * (val_u_sum - val_neg_u_sum);
```

**性能提升**: 减少了内存访问和计算量。

#### 10.1.3 描述子计算向量化 (Descriptor Computation Vectorization)

**优化**: 使用位操作和循环展开加速二进制测试：

```cpp
for (int i = 0; i < 32; ++i, pattern += 16)
{
    int t0, t1, val;
    t0 = GET_VALUE(0); t1 = GET_VALUE(1);
    val = t0 < t1;
    t0 = GET_VALUE(2); t1 = GET_VALUE(3);
    val |= (t0 < t1) << 1;
    // ... 8对比较展开
    desc[i] = (uchar)val;
}
```

**原理**: 将 8 次比较操作打包为单个字节，减少分支预测失败和内存写入次数。每个字节的第 $i$ 位由二进制测试 $\tau_i$ 决定：

$
\text{desc}[k] = \sum_{i=0}^{7} 2^i \cdot \tau_{8k+i}
$

其中 $\tau_i = \mathbb{1}[I(\mathbf{p}_i) < I(\mathbf{q}_i)]$ 为指示函数。

**宏定义优化**: 使用宏消除函数调用开销：

```cpp
#define GET_VALUE(idx) \
    center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
           cvRound(pattern[idx].x*a - pattern[idx].y*b)]
```

这允许编译器在内层循环中进行更激进的优化，避免函数调用的栈操作开销。

### 10.2 特征匹配优化 (Feature Matching Optimizations)

#### 10.2.1 汉明距离计算优化 (Hamming Distance Optimization)

**原始实现问题**: 手工实现的位计数算法在现代处理器上效率低下。

**优化方案**: 使用 OpenCV 的硬件加速实现：

```cpp
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    // 使用 OpenCV 优化实现（SSE/AVX/NEON）
    return cv::norm(a, b, cv::NORM_HAMMING);
}
```

**性能提升**: 在支持 SIMD 指令的平台上显著提升性能。OpenCV 的 `NORM_HAMMING` 自动选择最优实现：
- x86/x64: 使用 POPCNT 指令或 SSE4.2
- ARM: 使用 NEON 的 `vcnt` 指令

#### 10.2.2 距离计算优化 (Distance Computation Optimization)

**问题**: 在投影匹配中需要频繁计算 3D 点到相机的距离，`sqrt()` 操作成为瓶颈。

**优化策略**: 延迟平方根计算，先进行平方距离比较：

```cpp
// 使用平方距离进行快速范围检查
const float dx = PO.at<float>(0);
const float dy = PO.at<float>(1);
const float dz = PO.at<float>(2);
const float distSq = dx*dx + dy*dy + dz*dz;
const float maxDistSq = maxDistance * maxDistance;
const float minDistSq = minDistance * minDistance;

if(distSq < minDistSq || distSq > maxDistSq)
    continue;

// 只在需要时计算实际距离
const float dist = sqrt(distSq);
```

**数学原理**: 对于距离约束 $d_{min} \le \|\mathbf{P}\| \le d_{max}$，等价于 $d_{min}^2 \le \|\mathbf{P}\|^2 \le d_{max}^2$。由于平方函数在正实数域单调递增，可先进行平方距离判断，仅在通过筛选后计算实际距离。

**性能提升**: 消除了大量的 `sqrt()` 调用。

#### 10.2.3 整数乘法替代浮点运算 (Integer Multiplication Optimization)

**优化**: 在旋转直方图筛选中，使用整数乘法替代浮点乘法：

```cpp
// 原始: if(max2 < 0.1f * max1)
// 优化: 使用整数乘法
if(max2*10 < max1)
{
    ind2=-1;
    ind3=-1;
}
else if(max3*10 < max1)
{
    ind3=-1;
}
```

**原理**: 对于比例判断 $x < k \cdot y$，可转换为 $x \cdot \frac{1}{k} < y$。当 $k$ 为简单分数时（如 $0.1 = \frac{1}{10}$），可转换为整数乘法 $10x < y$，避免浮点运算。

### 10.3 内存访问优化 (Memory Access Optimizations)

#### 10.3.1 缓存友好的数据布局 (Cache-Friendly Data Layout)

**优化**: 在图像金字塔构建中，使用连续内存布局和边界填充：

```cpp
Size wholeSize(sz.width + ORB_EDGE_THRESHOLD*2, sz.height + ORB_EDGE_THRESHOLD*2);
Mat temp(wholeSize, image.type());
mvImagePyramid[level] = temp(Rect(ORB_EDGE_THRESHOLD, ORB_EDGE_THRESHOLD, sz.width, sz.height));
```

**原理**: 通过预分配带边界的完整图像，避免边界检查和条件分支，提高缓存命中率。

#### 10.3.2 预分配与容量保留 (Pre-allocation and Capacity Reservation)

**优化**: 在关键路径上预分配容器容量：

```cpp
vector<int> rotHist[HISTO_LENGTH];
for(int i=0;i<HISTO_LENGTH;i++)
    rotHist[i].reserve(500);  // 预分配容量

vToDistributeKeys.reserve(nfeatures*10);  // 预留足够空间
```

**效果**: 减少动态内存分配次数，避免 `vector` 扩容时的数据拷贝。

### 10.4 算法层面优化 (Algorithmic Optimizations)

#### 10.4.1 四叉树特征分布 (Quadtree Distribution)

**优势**: 相比原始的网格分布方法，四叉树算法提供：

1. **自适应空间划分**: 根据特征密度动态调整分割粒度
2. **更优的空间均匀性**: 近似泊松盘采样，最大化特征点间距
3. **更好的尺度不变性**: 在不同尺度层级保持一致的分布质量

**时间复杂度**: $O(N \log N)$，其中 $N$ 为候选特征点数量。

#### 10.4.2 双阈值 FAST 检测 (Dual-Threshold FAST)

**策略**: 自适应阈值机制确保在各种纹理条件下都能提取足够特征：

```cpp
FAST(cellImage, cellKeyPoints, iniThFAST, true);
if(cellKeyPoints.size() <= 3)
{
    cellKeyPoints.clear();
    FAST(cellImage, cellKeyPoints, minThFAST, true);
}
```

**效果**: 在弱纹理区域降低阈值，防止跟踪丢失；在强纹理区域保持高阈值，确保特征质量。

### 10.5 编译器优化与平台适配 (Compiler Optimizations)

#### 10.5.1 内联函数与宏定义

**优化**: 使用宏定义消除函数调用开销（已在 10.1.3 节描述）。

#### 10.5.2 分支预测优化

**优化**: 将常见情况放在 `if` 分支，罕见情况放在 `else`：

```cpp
if(!pMP->mbTrackInView)
    continue;  // 快速路径

if(pMP->isBad())
    continue;  // 快速路径

// 主要逻辑
```

**原理**: 现代 CPU 的分支预测器倾向于预测 `if` 分支为真，将快速失败路径放在前面可提高预测准确率。

#### 10.5.3 常量折叠与编译时计算

**优化**: 将运行时计算转换为编译时常量：

```cpp
const float factorPI = (float)(CV_PI/180.f);  // 编译时计算
```

编译器可以在编译阶段完成此计算，避免运行时的浮点除法。

#### 10.5.4 循环不变量外提

**优化**: 将循环内的不变计算移到循环外：

```cpp
const int step = (int)image.step1();  // 循环外计算
for (int v = 1; v <= ORB_HALF_PATCH_SIZE; ++v)
{
    int offset = v * step;  // 使用预计算的 step
    // ...
}
```

这避免了在每次循环迭代中重复计算图像步长。

### 10.6 性能提升总结 (Performance Improvement Summary)

| 优化项 | 目标模块 | 实施难度 | 精度影响 |
|--------|---------|---------|---------|
| 三角函数查找表 | 描述子计算 | 低 | 无 |
| IC_Angle 对称性优化 | 方向计算 | 中 | 无 |
| 延迟平方根计算 | 投影匹配 | 低 | 无 |
| 汉明距离硬件加速 | 特征匹配 | 低 | 无 |
| 整数乘法优化 | 直方图筛选 | 低 | 无 |
| 四叉树分布 | 特征提取 | 中 | 正面 |
| 双阈值 FAST | 特征检测 | 低 | 正面 |

**综合效果**: 在保持精度不变的前提下，特征提取和特征匹配模块的性能得到显著提升。

### 10.7 优化原则与最佳实践 (Optimization Principles)

1. **精度优先**: 所有优化必须保证算法精度不受影响
2. **可测量**: 每项优化都通过性能测试验证实际效果
3. **可回退**: 使用编译开关控制优化，便于问题定位
4. **渐进式**: 逐步实施优化，避免引入复杂性
5. **平台兼容**: 避免使用平台特定指令，保持代码可移植性

**关键经验**:
- 优先优化热点路径（通过性能分析工具定位）
- 数学优化优于代码优化（如延迟平方根计算）
- 利用硬件特性但保持可移植性（如 OpenCV 的自适应 SIMD）
- 详细记录优化前后的性能数据

---
*文档最后更新时间: 2026-01-31*
