#ifndef CONFIG_H
#define CONFIG_H

namespace ORB_SLAM2
{

// 统一配置参数管理

// ==========================================
// 相机参数（需根据实际标定结果修改）
// ==========================================

// fx, fy：焦距（像素），越大深度估计越精确但匹配容差下降
const float CAMERA_FX = 640.0f;
const float CAMERA_FY = 640.0f;

// cx, cy：主点坐标，通常为图像中心附近
const float CAMERA_CX = 320.0f;
const float CAMERA_CY = 180.0f;

// k1-k3/p1-p2：Brown-Conrady 畸变系数，全0表示已去畸变
const float CAMERA_K1 = 0.0f;
const float CAMERA_K2 = 0.0f;
const float CAMERA_P1 = 0.0f;
const float CAMERA_P2 = 0.0f;
const float CAMERA_K3 = 0.0f;

// 相机帧率，用于运动模型速度推断
const float CAMERA_FPS = 30.0f;

// 颜色顺序（0=BGR, 1=RGB），仅影响显示，不影响SLAM核心
const int CAMERA_RGB = 1;

// ==========================================
// ORB特征提取参数
// ==========================================

// 每帧期望特征点数。增大增加鲁棒性但线性增加计算量
const int ORB_EXTRACTOR_N_FEATURES = 1000;

// 图像金字塔相邻层缩放因子，1.2f 为 ORB-SLAM 标准值
const float ORB_EXTRACTOR_SCALE_FACTOR = 1.2f;

// 金字塔总层数（含底层），nLevels=8 覆盖尺度范围约 1:4.3
const int ORB_EXTRACTOR_N_LEVELS = 8;

// FAST角点检测阈值（INI_TH=20, MIN_TH=7），纹理不足时自动降低
const int ORB_EXTRACTOR_INI_TH_FAST = 20;
const int ORB_EXTRACTOR_MIN_TH_FAST = 7;

// 描述子列数，固定32字节（256位BRIEF）
const int ORB_DESC_COLS = 32;

// ==========================================
// 性能/内存限制（限定地图规模上限）
// ==========================================

// 最大关键帧数。手机推荐500-1000，桌面2000-5000
const int MAX_KEYFRAMES = 2000;

// 最大地图点数。设小则稀疏精度下降，设大则增加内存和优化耗时
const int MAX_MAPPOINTS = 10000;

// 单次剔除操作的移除数量
const int KEYFRAME_CULL_BATCH_SIZE = 5;
const int MAPPOINT_CULL_BATCH_SIZE = 500;

// ==========================================
// 修剪阈值
// ==========================================

// 关键帧冗余判定阈值：超过 90% 的地图点被≥3个其他KF观测时视为冗余
const float KEYFRAME_REDUNDANCY_THRESHOLD = 0.93f;

// 地图点冗余判定的最少观测KF数（观测≥此值的点不参与判定）
const int KEYFRAME_REDUNDANCY_OBS_THRESHOLD = 3;

// 新关键帧稳定所需的最小共视关键帧数
const int KEYFRAME_MIN_STABLE_COVISM = 3;

// 地图点被视为优质所需的最小观测KF数（单目模式）
const int MAPPOINT_MIN_OBSERVATIONS_MONO = 2;

// ==========================================
// 跟踪参数
// ==========================================

// 局部地图点最大数量，超过时截断防止单帧投影匹配耗时过高
const int TRACKING_MAX_LOCAL_MAP_POINTS = 5000;

// 视锥可见性判定阈值（0~1），越小越严格
const float FRUSTUM_VISIBILITY_TH = 0.5f;

// PnP求解中2D/3D坐标的有效范围上限
const float PNP_LIMIT_2D = 1e5f;
const float PNP_LIMIT_3D = 1e6f;

// PnP RANSAC 求解参数：迭代次数、误差阈值、置信概率
const int PNP_RANSAC_ITERATIONS = 200;
const float PNP_RANSAC_ERROR = 6.0f;
const double PNP_RANSAC_CONFIDENCE = 0.999;

// 地图对齐后跟踪搜索半径（像素）：对齐态=12，未对齐态=8
const float TRACKING_SEARCH_RADIUS_ALIGNED = 12.0f;
const float TRACKING_SEARCH_RADIUS_UNALIGNED = 8.0f;

// 加载点投影绑定最大深度（米），超过则跳过
const float BIND_MAX_DEPTH = 50.0f;

// 全局重定位搜索半径（米）
const float TRACKING_RELOC_SEARCH_RADIUS = 50.0f;

// 重定位成功后高精度投影绑定的搜索半径（像素）
const float RELOC_PROJ_SEARCH_RADIUS = 15.0f;

// 跟踪对齐更新参数：最小内点数、最小置信度、跳过帧数
const int TRACKING_ALIGN_MIN_INLIERS_UPDATE = 20;
const float TRACKING_ALIGN_MIN_CONFIDENCE_UPDATE = 0.3f;
const int TRACKING_ALIGN_SMOOTH_SKIP_FRAMES = 3;

// 参考缓存重试上限和缓存条目限制
const int TRACKING_MAX_REF_CACHE_RETRIES = 5;
const int TRACKING_REF_CACHE_LIMIT = 30000;

// 主线程绑定加载点的网格搜索半径（米）
const float TRACKING_GRID_SEARCH_RADIUS = 40.0f;

// 绑定加载点候选数超过此值时启用步长采样，设为0禁用
const int TRACKING_CANDIDATE_STRIDE_THRESHOLD = 3000;

// 后台重定位：候选KF数超过此值才启用姿态粗筛
const int TRACKING_GATING_MIN_CANDIDATES = 50;

// 后台重定位 KNN 比率阈值（越小越严格）和描述子距离上限
const float TRACKING_KNN_RATIO = 0.75f;
const int TRACKING_KNN_DIST_MAX = 60;

// 后台重定位 PnP 最小内点数要求
const int TRACKING_RELOC_PNP_MIN_INLIERS = 10;

// 后台重定位冷却帧数
const int TRACKING_RELOC_COOLDOWN_FRAMES = 5;

// 后台重定位 PnP 求解最大采样数，移动端建议200
const int RELO_BG_PNP_MAX_SAMPLES = 200;

// 后台重定位匹配分数归一化分母，仅用于UI展示
const float RELO_MATCH_SCORE_DIVISOR = 20.0f;

// 单目初始化所需最小 ORB 匹配点数
const int TRACKING_INIT_MIN_MATCHES = 100;

// TrackWithMotionModel 搜索半径（像素）和最少匹配数
const int TRACKING_MOTION_SEARCH_TH = 15;
const int TRACKING_MOTION_MIN_MATCHES = 20;

// TrackReferenceKeyFrame 最少匹配数
const int TRACKING_REFKF_MIN_MATCHES = 12;

// 参考KF匹配判定时地图点所需的最小观测数
const int REFKF_MIN_OBSERVATIONS = 3;

// 参考KF跟踪匹配比率（正常/新地图/单目）
const float TRACKING_KF_REF_RATIO = 0.75f;
const float TRACKING_KF_NEWMAP_RATIO = 0.4f;
const float TRACKING_KF_MONO_RATIO = 0.9f;

// SearchLocalPoints 投影搜索半径（像素）。建议快速运动时增大
const int TRACKING_LOCAL_SEARCH_TH = 4;

// 跟踪丢失状态搜索半径（更大=更高找回概率）
const int TRACKING_LOCAL_SEARCH_TH_LOST = 8;

// 重定位后短期内搜索半径
const int TRACKING_LOCAL_SEARCH_TH_RELOC = 6;

// 局部地图点数不足时的补充上限，设为0禁用
const int LOCAL_MAP_SUPPLEMENT_COUNT = 200;

// 跟踪失败宽容次数和内点数基线
const int TRACKING_MAX_CONSECUTIVE_FAIL = 3;
const int CONSECUTIVE_FAIL_INLIERS_BASELINE = 20;

// 重定位 PoseOptimization 最小内点数
const int TRACKING_POSE_OPT_MIN_INLIERS = 10;

// TrackLocalMap 成功多级阈值（严格/宽松/加载态）
const int TRACKING_SUCCESS_STRICT = 30;
const int TRACKING_SUCCESS_LOOSE = 15;
const int TRACKING_SUCCESS_LOADED = 10;

// 对齐状态下 STRICT 阈值覆盖值
const int ALIGNED_STRICT_INLIERS_OVERRIDE = 20;

// 局部关键帧列表最大数量限制
const int MAX_LOCAL_KEYFRAMES = 80;

// 共视图邻居数
const int COVISIBILITY_NEIGHBOR_COUNT = 10;

// ==========================================
// 初始化器参数
// ==========================================

// 初始化最小平均视差（像素）
const float INITIALIZER_MIN_PARALLAX_PX = 8.0f;

// 初始化超时时间（秒），超过时强制尝试初始化，设为0禁用
const double INITIALIZER_TIMEOUT_SEC = 3.0;

// 初始化创建初始地图的最少跟踪点数
const int INITIALIZER_MIN_TRACKED_POINTS = 100;

// ==========================================
// ORB底层参数
// ==========================================

// 描述子计算 Patch 大小（31×31）
const int ORB_PATCH_SIZE = 31;
const int ORB_HALF_PATCH_SIZE = 15;

// 关键点检测图像边缘阈值，避免提取无效边界特征
const int ORB_EDGE_THRESHOLD = 19;

// ==========================================
// 闭环检测参数
// ==========================================

// 距离上次闭环检测的最小间隔帧数
const int LOOP_MIN_FRAMES_SINCE_LAST = 10;

// 闭环匹配最少匹配数（候选/投影后）
const int LOOP_MIN_MATCHES = 20;
const int LOOP_MIN_MATCHES_AFTER_PROJ = 40;

// 闭环共视一致性阈值
const int LOOP_COVISIBILITY_CONSISTENCY_TH = 3;

// Sim3 RANSAC 参数
const double LOOP_RANSAC_PROB = 0.99;
const int LOOP_RANSAC_MIN_INLIERS = 20;
const int LOOP_RANSAC_MAX_ITERS = 300;

// ==========================================
// g2o 图优化参数
// ==========================================

// Huber 核函数阈值：2DoF≈2.448, 3DoF≈2.796
// 注意：自2026-07-09起，Huber已被Cauchy核替代
const float OPTIMIZER_HUBER_TH_2D = 2.4476519f;  // 保留用于chi2阈值判断
const float OPTIMIZER_HUBER_TH_3D = 2.79553215f;
const float OPTIMIZER_CAUCHY_DELTA = 1.5f;        // Cauchy核delta参数(最优值)

// 卡方检验阈值：2DoF=5.991, 1DoF=3.841
const float OPTIMIZER_CHI2_TH_2D = 5.991f;
const float OPTIMIZER_CHI2_TH_1D = 3.841f;

// Sim3 求解器卡方阈值（2DoF 99%置信）
const float SIM3_CHI2_TH = 9.210f;

// Sim3 RANSAC 求解参数
const double SIM3_RANSAC_PROB = 0.99;
const int SIM3_RANSAC_MIN_INLIERS = 6;
const int SIM3_RANSAC_MAX_ITERS = 300;

// ORB 匹配器参数：高阈值/低阈值/直方图区间/视角阈值/极线阈值
const int ORB_MATCHER_TH_HIGH = 100;
const int ORB_MATCHER_TH_LOW = 50;
const int ORB_MATCHER_HISTO_LENGTH = 30;
const float ORB_MATCHER_VIEW_COS_TH = 0.998f;
const float ORB_MATCHER_EPILINE_TH = 3.84f;

// 初始化器 RANSAC 参数
const int INITIALIZER_RANSAC_MIN_SET = 8;
const float INITIALIZER_H_SCORE_RATIO = 0.40f;
const float INITIALIZER_MIN_PARALLAX = 1.0f;
const int INITIALIZER_MIN_TRIANGULATED = 50;

// PnP RANSAC 通用参数
const double PNP_RANSAC_PROB = 0.99;
const int PNP_RANSAC_MIN_INLIERS = 10;
const int PNP_RANSAC_MAX_ITERS = 300;
const int PNP_RANSAC_MIN_SET = 4;
const float PNP_RANSAC_EPSILON = 0.5f;
const float PNP_RANSAC_TH2 = 5.991f;

// 自适应RANSAC提前终止参数
const int PNP_ADAPTIVE_START_ITER = 30;   // 至少迭代30次后才检查提前终止
const float PNP_ADAPTIVE_MIN_RATIO = 0.3f;  // 内点率低于30%时不触发提前终止
const float PNP_ADAPTIVE_SAFETY_FACTOR = 1.5f;  // 安全系数：理论×1.5后提前终止

// 帧网格划分：48×64，约640×360时每格13.3×7.5像素
const int FRAME_GRID_ROWS = 48;
const int FRAME_GRID_COLS = 64;
const float FRAME_GRID_RESERVE_FACTOR = 0.5f;

// 关键帧连接所需的共视地图点最小数量
const int KEYFRAME_CONNECTION_TH = 15;

// 地图点深度不变性范围因子
const float MAPPOINT_MIN_DIST_INVARIANCE_FACTOR = 0.8f;
const float MAPPOINT_MAX_DIST_INVARIANCE_FACTOR = 1.2f;

// 地图点默认最近/最远观测距离
const float MAPPOINT_DEFAULT_MIN_DIST = 0.1f;
const float MAPPOINT_DEFAULT_MAX_DIST = 100.0f;

// 地图点标记为不良的最小观测数和最低找到比率
const int MAPPOINT_MIN_OBS_FOR_BAD = 2;
const float MAPPOINT_MIN_FOUND_RATIO = 0.25f;

// 系统加载地图的容量上限
const int SYSTEM_MAX_KFS_LOAD = 10000;
const int SYSTEM_MAX_MPS_LOAD = 500000;

// 地图文件格式魔数和版本号
const uint32_t SYSTEM_MAP_FILE_MAGIC = 0x4D415031;
const uint32_t SYSTEM_MAP_FILE_VERSION = 1;

// 重定位配置
const int SYSTEM_RELOC_CONFIG_TOP_K = 20;
const int SYSTEM_RELOC_CONFIG_MAX_CANDIDATES = 5000;
const int SYSTEM_RELOC_CONFIG_MATCH_CHUNK = 500;
const int SYSTEM_RELOC_CONFIG_BG_SLEEP_US = 80000;
const int SYSTEM_RELOC_CONFIG_MAX_BIND_INLIERS = 80;
const int SYSTEM_RELOC_CONFIG_MAX_PROJ_BINDS = 30;

// ==========================================
// 后台线程调度参数
// ==========================================

// 主线程每帧最多绑定的地图点数和触发阈值
const int MAIN_THREAD_MAX_BIND_PER_FRAME = 50;
const int MAIN_THREAD_BIND_INLIER_THRESHOLD = 100;

// ==========================================
// 重定位对齐参数
// ==========================================

// 重定位对齐的最小内点数要求和置信度
const int RELOC_MIN_INLIERS_FOR_ALIGN = 15;
const float RELOC_MIN_CONFIDENCE_FOR_ALIGN = 0.4f;

// 重定位后短期内投影搜索窗口半径（像素）
const int RELOC_POST_SEARCH_TH = 8;

// Reset 后重定位冷却帧数
const int RESET_COOLDOWN_FRAMES = 30;

// 连续丢失超过此帧数创建新子地图
const int TRACKING_LOST_FRAMES_FOR_NEW_MAP = 30;

// 新建子地图后的冷却帧数（150帧≈5秒@30fps）
const int TRACKING_NEW_MAP_COOLDOWN_FRAMES = 150;

// 子地图最大数量
const int MAX_SUBMAP_COUNT = 10;

// ==========================================
// 局部建图参数
// ==========================================

// 三角化选取的邻近关键帧数和基线/深度比
const int LOCAL_MAPPING_TRIANGULATION_NEIGHBORS = 20;
const float LOCAL_MAPPING_TRIANGULATION_BASELINE_RATIO = 0.01f;

// 三角化最小视差阈值和比率测试因子
const float LOCAL_MAPPING_TRIANGULATION_PARALLAX_TH = 0.9998f;
const float LOCAL_MAPPING_TRIANGULATION_RATIO_FACTOR = 1.5f;

// 一级/二级搜索的关键帧上限
const int LOCAL_MAPPING_NEIGHBOR_KFS = 20;
const int LOCAL_MAPPING_SECOND_NEIGHBOR_KFS = 5;

// 新关键帧的修剪保护帧数和输入队列最大积压数
const int LOCAL_MAPPING_CULL_PROTECT_FRAMES = 5;
const int LOCAL_MAPPING_MAX_QUEUED_KFS = 5;

// ==========================================
// Essential Graph BA
// ==========================================

// 构建 Essential Graph 时两KF间最少共视图点数
const int OPTIMIZER_ESSENTIAL_GRAPH_MIN_FEAT = 100;

// ==========================================
// 重定位优化
// ==========================================

// 重定位最小共享词数和最大候选帧数
const int RELOC_MIN_SHARED_WORDS = 10;
const int RELOC_MAX_CANDIDATES = 20;

// ==========================================
// 系统运行时参数
// ==========================================

// 等待线程停止的超时时间（毫秒）
const int LOOP_LOCALMAPPER_TIMEOUT_MS = 5000;

// 创建新子地图冷却时间（毫秒）
const int NEW_MAP_COOLDOWN_MS = 5000;

// ==========================================
// JNI 桥接层参数
// ==========================================

// SLAM 帧率和图像下采样因子
const float SYSTEM_FPS = 30.0f;
const float IMAGE_DOWNSCALE_FACTOR = 2.0f;

// 工作基准分辨率
const float BASE_SLAM_WIDTH = 640.0f;
const float BASE_SLAM_HEIGHT = 360.0f;

// OpenGL 投影裁剪面距离
const float PROJECTION_ZNEAR = 0.1f;
const float PROJECTION_ZFAR = 1000.0f;

// 丢失自动重置超时（秒）和地图切换确认帧数
const double LOST_RESET_TIMEOUT = 3.0;
const int MAP_SWITCH_THRESHOLD = 3;

// AR 模式最少新增点数和物体默认缩放
const int MIN_NEW_POINTS_BEFORE_AR = 50;
const float AR_OBJECT_SCALE_DEFAULT = 0.20f;

// 平面检测状态码和RANSAC迭代次数
const int PLANE_DETECTED = 233;
const int PLANE_NOT_DETECTED = 1234;
const int PLANE_DETECT_RANSAC_ITERS = 50;

}

#endif // CONFIG_H
