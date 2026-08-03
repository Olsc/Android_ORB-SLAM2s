#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

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

// 参考缓存（HBST 树）重建冷却时间（毫秒）：跟踪丢失→重建→重定位循环中，
// 避免频繁重建整棵 HBST 树导致帧率骤降；缓存可用且未超冷却时直接复用旧缓存
const int TRACKING_REF_CACHE_BUILD_COOLDOWN_MS = 2000;

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
const float OPTIMIZER_HUBER_TH_2D = 2.4476519f;  // Huber核函数delta参数
const float OPTIMIZER_HUBER_TH_3D = 2.79553215f;

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

// ==========================================
// 重定位后台线程调度
// ==========================================

// 后台重定位锁等待重试间隔（毫秒）
const int RELOC_RETRY_WAIT_MS = 1;

// 缓存重建失败/为空时的重试间隔（毫秒）
const int RELOC_CACHE_RETRY_WAIT_MS = 500;

// 地图无点时后台线程休眠间隔（秒）
const int RELOC_NO_MAP_WAIT_SEC = 2;

// ==========================================
// 对齐 EMA 平滑
// ==========================================

// 对齐质量分封顶（inliers 上限）
const float ALIGN_QUALITY_SCORE_CAP = 100.0f;

// EMA alpha 上下限（质量越高新值权重越大，限制在 0.1-0.5）
const float ALIGN_EMA_MAX_ALPHA = 0.5f;
const float ALIGN_EMA_MIN_ALPHA = 0.1f;

// ==========================================
// 安全 PnP 求解
// ==========================================

// ≥6 点走 RANSAC（避免 OpenCV 内部 DLT 断言）
const int PNP_MIN_SAMPLES_RANSAC = 6;

// 4-5 点走 P3P/EPNP 并做重投影验证
const int PNP_MIN_SAMPLES_P3P = 4;

// 4-5 点 PnP 重投影误差阈值（像素）
const float PNP_REPROJ_ERROR_TH = 5.0f;

// ==========================================
// 重定位置信度与地图投票
// ==========================================

// 后台重定位置信度归一化内点数
const float RELOC_CONF_NORM_INLIERS = 50.0f;

// 地图投票胜出需比第二名多出的比例（20%）
const float RELOC_MAP_VOTE_MARGIN = 1.2f;

// 重定位候选数硬上限
const int RELOC_MAX_CANDIDATES_HARD_CAP = 10000;

// ==========================================
// 加载地图空间网格
// ==========================================

// 加载地图空间网格单元尺寸（米）
const float LOADED_MAP_GRID_CELL_SIZE = 10.0f;

// 网格单元格数硬上限（防内存爆炸）
const long long GRID_CELLS_HARD_CAP = 10000000LL;

// 网格每维度最大格数（超限时按此缩小单元尺寸）
const int GRID_MAX_DIM_CELLS = 100;

// ==========================================
// Reset / 新地图保护
// ==========================================

// 初始化后 Reset 保护：地图 KF 数上限
const int RESET_PROTECT_MAX_KFS = 5;

// 初始化后距上一关键帧的最少帧数
const int RESET_PROTECT_AFTER_KF_FRAMES = 20;

// 触发新建子地图的最小 KF 数
const int TRACKING_NEW_MAP_MIN_KFS = 10;

// ==========================================
// 初始化（Tracking 侧）
// ==========================================

// 初始化 RANSAC 噪声标准差
const float INITIALIZER_SIGMA = 1.0f;

// 初始化 RANSAC 迭代次数
const int INITIALIZER_RANSAC_ITERS = 200;

// 初始化匹配搜索窗口（像素）
const int INITIALIZER_SEARCH_WINDOW = 100;

// 初始化提取器特征数倍数（2×正常特征数）
const int INITIALIZER_FEATURE_MULTIPLIER = 2;

// ==========================================
// 跟踪判定
// ==========================================

// 定位模式 VO 判定最小内点数
const int TRACKING_VO_MIN_MATCHES = 10;

// 局部地图补充触发阈值（点数不足时补充）
const int LOCAL_MAP_SUPPLEMENT_TRIGGER = 50;

// 跳过全局搜索的触发条件：地图点数上限 / KF 数上限
const int TRACKING_SKIP_GLOBAL_MAX_MPS = 5000;
const int TRACKING_SKIP_GLOBAL_MAX_KFS = 20;

// 自举关键帧最小有效匹配数
const int BOOTSTRAP_KF_MIN_MATCHES = 10;

// 初始地图全局 BA 迭代次数
const int GBA_INIT_ITERATIONS = 20;

// ==========================================
// 地图切换
// ==========================================

// 地图切换最小内点数 / 内点数增强比例（按上次内点数递增）
const int MAP_SWITCH_MIN_INLIERS = 60;
const float MAP_SWITCH_INLIERS_FACTOR = 1.25f;

// 地图切换冷却帧数
const int MAP_SWITCH_COOLDOWN_FRAMES = 20;

// ==========================================
// 地图对齐与加载点绑定
// ==========================================

// 高置信度投影绑定阈值
const float RELOC_STRONG_BIND_CONFIDENCE = 0.9f;

// 保持对齐的最小当前图加载点内点数
const int RELOC_KEEP_ALIGN_MIN_CURMAP_INLIERS = 5;

// 新地图（KF≤2）特殊阈值
const int NEW_MAP_KF_COUNT = 2;

// 局部建图队列积压接受上限
const int KEYFRAME_QUEUE_ACCEPT_LIMIT = 3;

// 重定位后短期窗口帧数
const int RELOC_POST_FRAMES_WINDOW = 10;

// 加载点近邻匹配数量上限（对齐态 / 普通态）
const int LOADED_MATCH_MAX_ALIGNED = 500;
const int LOADED_MATCH_MAX = 200;

// 对齐时搜索半径放大系数
const float LOADED_MATCH_RADIUS_SCALE_ALIGNED = 1.8f;

// ==========================================
// 主线程重定位
// ==========================================

// 主线程重定位候选 KF 上限
const int RELOC_MAX_CANDIDATE_KFS = 3;

// 重定位 PnP RANSAC 总超时（毫秒）
const int RELOC_PNP_TIMEOUT_MS = 1000;

// 每轮 RANSAC 迭代次数
const int RELOC_RANSAC_ROUND_ITERS = 5;

// 重定位成功内点数
const int RELOC_SUCCESS_INLIERS = 50;

// 触发窄窗口再搜索的内点数
const int RELOC_NARROW_TRIGGER_INLIERS = 30;

// 粗窗口投影搜索半径（像素）与最大匹配数
const int RELOC_PROJ_TH_COARSE = 10;
const int RELOC_PROJ_MAX_COARSE = 100;

// 窄窗口投影搜索半径（像素）与最大匹配数
const int RELOC_PROJ_TH_NARROW = 3;
const int RELOC_PROJ_MAX_NARROW = 64;

// 降级重定位成功置信度
const float RELOC_FALLBACK_CONFIDENCE = 1.0f;

// ==========================================
// HBST 树匹配
// ==========================================

// HBST 树匹配最大描述子距离
const int HBST_MATCH_MAX_DIST = 75;

// HBST 降级重定位最少匹配数
const int HBST_RELOC_MIN_MATCHES = 12;

// ==========================================
// ORB 匹配器
// ==========================================

// 256 位 BRIEF 描述子最大汉明距离（32 字节 × 8）
const int ORB_MAX_DISTANCE = 256;

// 视角方向决定的搜索半径（近视角 / 远视角，像素）
const float MATCH_RADIUS_NEAR = 2.5f;
const float MATCH_RADIUS_FAR = 4.0f;

// 金字塔层级差异阈值（触发等效距离惩罚）
const float MATCH_SCALE_DIFF_TH = 2.0f;

// 尺度差异等效距离惩罚系数
const float MATCH_SCALE_PENALTY = 0.15f;

// 大尺度差时的比率放宽系数
const float MATCH_SCALE_RATIO_RELAX = 0.1f;

// 观察角度过滤：cos²<0.25（60°）/ cos<0.5（60°）
const float MATCH_VIEW_COS_SQ_TH = 0.25f;
const float MATCH_VIEW_COS_TH = 0.5f;

// 三角化搜索极线点距离平方阈值
const int TRIANGULATION_EPIPOLE_DIST_SQ = 100;

// 旋转直方图主峰优势倍数与桶预分配容量
const int ROT_HIST_DOMINANT_FACTOR = 10;
const int ROT_HIST_RESERVE = 500;

// 各场景 ORBmatcher 最近邻比率（nnratio）
const float ORB_MATCHER_NNRATIO_MOTION = 0.9f;          // 运动模型/初始化/重定位二次搜索
const float ORB_MATCHER_NNRATIO_REFKF = 0.7f;           // TrackReferenceKeyFrame
const float ORB_MATCHER_NNRATIO_LOCAL = 0.8f;           // SearchLocalPoints
const float ORB_MATCHER_NNRATIO_RELOC = 0.75f;          // 重定位候选匹配
const float ORB_MATCHER_NNRATIO_TRIANGULATION = 0.6f;   // 三角化搜索
const float ORB_MATCHER_NNRATIO_FUSE = 0.8f;            // SearchAndFuse
const float ORB_MATCHER_NNRATIO_LOOP = 0.75f;           // 闭环 SearchBySim3

// ORBmatcher 构造与成员函数默认参数
const float ORB_MATCHER_DEFAULT_NN_RATIO = 0.6f;        // 默认最近邻比率
const int ORB_MATCHER_DEFAULT_PROJ_TH = 3;              // 默认投影搜索半径（像素）
const float ORB_MATCHER_DEFAULT_FUSE_TH = 3.0f;         // 默认 Fuse 搜索半径（像素）
const int ORB_MATCHER_INIT_WINDOW = 10;                 // 初始化匹配窗口半宽（像素）

// ==========================================
// ORB 提取器
// ==========================================

// BRIEF 采样点数（256 位 × 2 采样点/位）与采样点对数
const int ORB_BRIEF_NUM_POINTS = 512;
const int ORB_BRIEF_NUM_PAIRS = 256;

// FAST 检测网格单元宽度（像素）
const float ORB_FAST_GRID_CELL = 30.0f;

// FAST 边缘补偿像素数
const int ORB_FAST_BORDER_MARGIN = 3;

// 候选关键点预分配倍数（性能）
const int ORB_CANDIDATE_RESERVE_FACTOR = 10;

// ==========================================
// 局部建图
// ==========================================

// Stop 态等待 Release 的超时（毫秒），防止 LM 永久卡死
const int LOCAL_MAPPING_STOP_WAIT_TIMEOUT_MS = 5000;

// 事件循环轮询间隔（毫秒）
const int LOCAL_MAPPING_EVENT_WAIT_MS = 3;

// 局部 BA 前提：地图最少 KF 数
const int LOCAL_BA_MIN_KEYFRAMES = 3;

// 中位深度统计排序索引（ComputeSceneMedianDepth 参数）
const int TRIANGULATION_DEPTH_PERCENTILE = 2;

// 三角化 4×4 特征分解最大迭代次数
const int TRIANGULATION_JACOBI_MAX_ITERS = 30;

// MapPointCulling 的 KF 间隔时间窗：≥2 帧观测不足判死；≥3 帧移出候选列表
const int MAPPOINT_CULL_KF_GAP_CHECK = 2;
const int MAPPOINT_CULL_KF_GAP_REMOVE = 3;

// ==========================================
// 位姿优化 / BA
// ==========================================

// PoseOptimization 单次优化总超时（毫秒）
const int POSE_OPT_TIMEOUT_MS = 200;

// PoseOptimization 每轮 g2o 迭代次数与优化轮数
const int POSE_OPT_PASS_ITERS = 5;
const int POSE_OPT_PASSES = 4;

// PoseOptimization 最少初始对应点 / 最少边数
const int POSE_OPT_MIN_CORRESPONDENCES = 3;
const int POSE_OPT_MIN_EDGES = 10;

// BA 类函数默认迭代次数（Optimizer.h 默认参数）
const int OPTIMIZER_DEFAULT_BA_ITERS = 5;

// 局部 BA 窗口最大共视 KF 数 / 迭代次数
const int LOCAL_BA_MAX_KFS = 10;
const int LOCAL_BA_ITERATIONS = 5;

// Essential Graph BA 迭代次数
const int ESSENTIAL_GRAPH_BA_ITERS = 20;

// OptimizeSim3 迭代次数：首轮 / 存在外点时追加 / 最少内点数 / 卡方阈值
const int SIM3_OPT_ITERS = 5;
const int SIM3_OPT_EXTRA_ITERS = 10;
const int SIM3_OPT_MIN_INLIERS = 10;
const float SIM3_OPT_CHI2_TH = 10.0f;

// ==========================================
// 闭环检测
// ==========================================

// 线程事件轮询等待（毫秒，LoopClosing/System 共用）
const int THREAD_POLL_WAIT_MS = 5;

// SearchByHBST 匹配率阈值
const float LOOP_MATCHER_RATIO = 0.75f;

// Sim3 RANSAC 每轮批量迭代次数
const int SIM3_RANSAC_ITER_PER_PASS = 5;

// SearchBySim3 投影搜索半径（像素）
const float LOOP_SEARCH_RADIUS_SIM3 = 7.5f;

// 闭环投影搜索半径（像素）
const int LOOP_PROJ_SEARCH_TH = 10;

// SearchAndFuse 匹配率阈值 / Fuse 融合搜索半径（像素）
const float LOOP_FUSE_MATCHER_RATIO = 0.8f;
const int LOOP_FUSE_SEARCH_TH = 4;

// 全局 BA 迭代次数
const int GBA_ITERATIONS = 10;

// ==========================================
// 地图加载（System）
// ==========================================

// 加载地图点初始化可见性计数（有描述子 +10，无 +5；须远大于 0.25 剔除阈值）
const int LOADED_MP_INIT_VISIBLE = 10;
const int LOADED_MP_INIT_VISIBLE_NO_DESC = 5;

// ==========================================
// 初始化器（RANSAC 恢复）
// ==========================================

// 匹配数分界：<150 用好点比率 0.6，否则 0.9
const int INITIALIZER_GOOD_RATIO_SMALL_N = 150;
const float INITIALIZER_GOOD_RATIO_SMALL = 0.6f;
const float INITIALIZER_GOOD_RATIO_LARGE = 0.9f;

// 三角化点数硬性下限（max(30, N/2)）
const int INITIALIZER_MIN_TRI_HARD = 30;

// 重投影误差阈值系数（4.0×mSigma2）
const float INITIALIZER_REPROJ_TH_FACTOR = 4.0f;

// 4 个运动假设相似性判定比率
const float INITIALIZER_NGOOD_SIMILAR_RATIO = 0.7f;

// 平面场景奇异判定阈值（d1/d2≈1）
const float INITIALIZER_SINGULAR_RATIO = 1.00001f;

// 最优/次优内点比判定
const float INITIALIZER_BEST_RATIO = 0.75f;

// Jacobi 特征分解迭代上限
const int INITIALIZER_JACOBI_ITERS = 30;

// 负深度视差判定 cos 阈值（≈0.36°）与视差分位
const float INITIALIZER_PARALLAX_COS_TH = 0.99998f;
const int INITIALIZER_PARALLAX_PERCENTILE = 50;

// ==========================================
// Sim3 求解器 / PnP 求解器
// ==========================================

// Sim3 最小点集（RANSAC 采样）
const int SIM3_RANSAC_MIN_SET = 3;

// PnP Gauss-Newton 迭代次数
const int PNP_GN_ITERS = 5;

// PnP RANSAC 迭代次数公式中 epsilon 的幂指数（沿用原版 Sim3 逻辑）
const int PNP_RANSAC_POWER = 3;

// ==========================================
// 关键帧数据库
// ==========================================

// 累计擦除 KF 数触发树重建
const int KFD_REBUILD_ERASE_TH = 20;

// HBST 树匹配最大结果数
const int KFD_HBST_MATCH_LIMIT = 50;

// 闭环/重定位候选 KF 的最少词匹配数
const int KFD_LOOP_MIN_WORD_MATCHES = 15;
const int KFD_RELOC_MIN_WORD_MATCHES = 15;

// 候选 KF 累积得分保留比例（0.75×最高分）
const float KFD_SCORE_RETAIN_RATIO = 0.75f;

// ==========================================
// 帧 / 地图点
// ==========================================

// GetFeaturesInArea 预分配容量
const int FRAME_SEARCH_RESERVE = 16;

// 冗余观测判定的尺度层容差
const int MAPPOINT_SCALE_LEVEL_TOL = 1;

// 描述子计算观测数上限（栈矩阵 64×64）
const int MAPPOINT_DESC_MAX_OBS = 64;

// ==========================================
// AR / JNI 层
// ==========================================

// '.arinfo' 文件魔数 'ARIN' 与版本号
const uint32_t AR_INFO_FILE_MAGIC = 0x4152494E;
const uint32_t AR_INFO_FILE_VERSION = 1;

// 捏合缩放灵敏度分母 / AR 物体最小缩放
const float AR_SCALE_ZOOM_DIVISOR = 5.0f;
const float AR_SCALE_MIN = 0.03f;

// 3DOF 物体绕 Y 轴自转 / 绕 X 轴倾斜角度
const float AR_OBJECT_SPIN_Y_DEG = 45.0f;
const float AR_OBJECT_TILT_X_DEG = 30.0f;

// 3DOF 模式投影裁剪面：此模式下 frustumM 边界为 ±ratio/±1（不乘 near），
// near 直接参与透视缩放（ndc ∝ near/d）。near=1.0 为原设计值
const float AR_3DOF_ZNEAR = 1.0f;
const float AR_3DOF_ZFAR = 100.0f;

// ==========================================
// UI 绘制
// ==========================================

// 最大绘制点数（性能保护）
const int UI_MAX_DRAWN_POINTS = 5000;

// 新建点颜色 BGR(31,188,210) 与已加载点颜色 BGR(0,255,0)（cv::Scalar 参数顺序 B,G,R）
const int UI_COLOR_NEW_POINT_B = 31;
const int UI_COLOR_NEW_POINT_G = 188;
const int UI_COLOR_NEW_POINT_R = 210;
const int UI_COLOR_LOADED_POINT_B = 0;
const int UI_COLOR_LOADED_POINT_G = 255;
const int UI_COLOR_LOADED_POINT_R = 0;

// 特征点圆半径 / 点云圆半径（像素）
const int UI_POINT_RADIUS = 2;
const int UI_CLOUD_POINT_RADIUS = 1;

// 深度过近剔除阈值（米，相机后方/贴脸剔除）
const float PROJECT_MIN_DEPTH = 0.01f;

// ==========================================
// 平面检测（UIUtils）
// ==========================================

// 稳定点最少观测数 / 平面拟合最少点数
const int PLANE_MIN_OBSERVATIONS = 5;
const int PLANE_MIN_POINTS = 50;

// 距离边界尾部比例与中值计算最少样本
const float PLANE_MEDIAN_TAIL_RATIO = 0.2f;
const int PLANE_MEDIAN_MIN_SAMPLES = 20;

// 内点判定阈值比例（1.4×最佳中值距离）
const float PLANE_INLIER_TH_RATIO = 1.4f;

// ==========================================
// 性能分析器（profiler）
// ==========================================

// 分析文件魔数 'VPRO' 与版本号
const uint32_t PROFILER_MAGIC = 0x4F525056;
const uint32_t PROFILER_VERSION = 1;

// 事件记录 / 名称映射记录标记
const uint8_t PROFILER_EVENT_MARKER = 0xEE;
const uint8_t PROFILER_MAP_MARKER = 0xFF;

// 写入线程等待超时（毫秒）/ 批量写盘上限
const int PROFILER_WAIT_TIMEOUT_MS = 100;
const int PROFILER_BATCH_MAX = 1000;

// ==========================================
// OpenCV 桥接
// ==========================================

// RGBA 不透明 alpha
const int RGBA_ALPHA_OPAQUE = 255;

// ==========================================
// Ubuntu 桌面程序
// ==========================================

// 采集分辨率 / 显示分辨率上限
const int UBUNTU_CAPTURE_WIDTH = 1280;
const int UBUNTU_CAPTURE_HEIGHT = 720;
const int UBUNTU_MAX_DISPLAY_W = 1280;
const int UBUNTU_MAX_DISPLAY_H = 720;

// AR 立方体自适应尺寸系数 / 最小尺寸 / 回退尺寸
const float AR_CUBE_SCALE_FACTOR = 0.1f;
const float AR_CUBE_MIN_SIZE = 0.01f;
const float AR_CUBE_FALLBACK_SIZE = 0.05f;

// benchmark 记录预分配容量 / 进度输出间隔
const int BENCH_RESERVE_DEFAULT = 10000;
const int BENCH_PROGRESS_INTERVAL = 100;

}

#endif // CONFIG_H
