#ifndef CONFIG_H
#define CONFIG_H

namespace ORB_SLAM2
{

// 统一配置参数管理

// ==========================================
// 相机参数
// ==========================================

// 相机内参
const float CAMERA_FX = 640.0f;
const float CAMERA_FY = 640.0f;
const float CAMERA_CX = 320.0f;
const float CAMERA_CY = 180.0f;

// 相机畸变参数
const float CAMERA_K1 = 0.0f;
const float CAMERA_K2 = 0.0f;
const float CAMERA_P1 = 0.0f;
const float CAMERA_P2 = 0.0f;
const float CAMERA_K3 = 0.0f;

// 相机帧率
const float CAMERA_FPS = 30.0f;

// 图像颜色顺序 (0: BGR, 1: RGB)
const int CAMERA_RGB = 1;

// ==========================================
// ORB特征提取参数
// ==========================================

// 每帧图像提取的特征点数量
const int ORB_EXTRACTOR_N_FEATURES = 1000;

// 尺度金字塔相邻层之间的缩放因子
const float ORB_EXTRACTOR_SCALE_FACTOR = 1.2f;

// 尺度金字塔层数
const int ORB_EXTRACTOR_N_LEVELS = 8;

// FAST角点检测阈值
const int ORB_EXTRACTOR_INI_TH_FAST = 20;
const int ORB_EXTRACTOR_MIN_TH_FAST = 7;

// ==========================================
// 性能/内存限制
// ==========================================

// 地图中允许的最大关键帧数量。当超过此数量时，系统将修剪旧的/冗余的关键帧以保持性能。
const int MAX_KEYFRAMES = 2000; 

// 地图中允许的最大地图点数量。当超过此数量时，系统将从最早的点开始修剪。
const int MAX_MAPPOINTS = 10000;

// 当达到限制时要修剪的关键帧数量（批量删除）
const int KEYFRAME_CULL_BATCH_SIZE = 5;

// 当达到限制时要修剪的地图点数量
const int MAPPOINT_CULL_BATCH_SIZE = 500;


// ==========================================
// 修剪阈值
// ==========================================

// 关键帧修剪的冗余阈值（0.9表示90%的点被3个或更多其他关键帧观测到）
const float KEYFRAME_REDUNDANCY_THRESHOLD = 0.9f;

// 一个点被视为冗余所需的其他关键帧观测数量
const int KEYFRAME_REDUNDANCY_OBS_THRESHOLD = 3;

// 地图点被视为有效的最小观测数
const int MAPPOINT_MIN_OBSERVATIONS_MONO = 2;
const int MAPPOINT_MIN_OBSERVATIONS_STEREO = 3;


// ==========================================
// 跟踪参数
// ==========================================

// 局部地图点最大数量限制
const int TRACKING_MAX_LOCAL_MAP_POINTS = 5000;

// PnP 的 2D 像素坐标限制（检查合理范围）
const float PNP_LIMIT_2D = 1e5f;

// PnP 的 3D 坐标限制（检查合理范围）
const float PNP_LIMIT_3D = 1e6f;

// PnP RANSAC 参数
const int PNP_RANSAC_ITERATIONS = 200;
const float PNP_RANSAC_ERROR = 6.0f;
const double PNP_RANSAC_CONFIDENCE = 0.999;

// 地图对齐的搜索半径（已对齐/未对齐）
const float TRACKING_SEARCH_RADIUS_ALIGNED = 12.0f;
const float TRACKING_SEARCH_RADIUS_UNALIGNED = 8.0f;

// 全局重定位搜索半径（米）
const float TRACKING_RELOC_SEARCH_RADIUS = 50.0f;


// ==========================================
// ORB特征提取参数
// ==========================================

// 计算描述子的 Patch 大小
const int ORB_PATCH_SIZE = 31;

// Patch 大小的一半
const int ORB_HALF_PATCH_SIZE = 15;

// 关键点检测的边缘阈值
const int ORB_EDGE_THRESHOLD = 19;


// ==========================================
// 闭环检测参数
// ==========================================

// 距离上次闭环检测的最小帧数
const int LOOP_MIN_FRAMES_SINCE_LAST = 10;

// 有效闭环候选所需的最小匹配数
const int LOOP_MIN_MATCHES = 20;

// Sim3 投影后接受闭环所需的最小匹配数
const int LOOP_MIN_MATCHES_AFTER_PROJ = 40;

// 闭环检测的共视一致性阈值
const int LOOP_COVISIBILITY_CONSISTENCY_TH = 3;

// Sim3 计算的 RANSAC 参数
const double LOOP_RANSAC_PROB = 0.99;
const int LOOP_RANSAC_MIN_INLIERS = 20;
const int LOOP_RANSAC_MAX_ITERS = 300;


// ==========================================
// 优化器参数
// ==========================================

// Huber 核阈值（卡方值）
const float OPTIMIZER_HUBER_TH_2D = 2.4476519f; // sqrt(5.991)
const float OPTIMIZER_HUBER_TH_3D = 2.79553215f; // sqrt(7.815)

// 卡方阈值
const float OPTIMIZER_CHI2_TH_2D = 5.991f;
const float OPTIMIZER_CHI2_TH_3D = 7.815f;

// Sim3求解器参数
const float SIM3_CHI2_TH = 9.210f; // 卡方 2DoF 99%
const double SIM3_RANSAC_PROB = 0.99;
const int SIM3_RANSAC_MIN_INLIERS = 6;
const int SIM3_RANSAC_MAX_ITERS = 300;

// ORB匹配器参数
const int ORB_MATCHER_TH_HIGH = 100;
const int ORB_MATCHER_TH_LOW = 50;
const int ORB_MATCHER_HISTO_LENGTH = 30;
const float ORB_MATCHER_VIEW_COS_TH = 0.998f;
const float ORB_MATCHER_EPILINE_TH = 3.84f; // 卡方 1DoF

// 初始化器参数
const int INITIALIZER_RANSAC_MIN_SET = 8; // RANSAC所需点数
const float INITIALIZER_H_SCORE_RATIO = 0.40f; // 选择H而非F的比率
const float INITIALIZER_MIN_PARALLAX = 1.0f;
const int INITIALIZER_MIN_TRIANGULATED = 50;

// PnP求解器参数
const double PNP_RANSAC_PROB = 0.99;
const int PNP_RANSAC_MIN_INLIERS = 8;
const int PNP_RANSAC_MAX_ITERS = 300;
const int PNP_RANSAC_MIN_SET = 4;
const float PNP_RANSAC_EPSILON = 0.4f;
const float PNP_RANSAC_TH2 = 5.991f;

// 帧参数
const int FRAME_GRID_ROWS = 48;
const int FRAME_GRID_COLS = 64;
const float FRAME_GRID_RESERVE_FACTOR = 0.5f;

// 关键帧参数
const int KEYFRAME_CONNECTION_TH = 15;

// 地图点参数
const float MAPPOINT_MIN_DIST_INVARIANCE_FACTOR = 0.8f;
const float MAPPOINT_MAX_DIST_INVARIANCE_FACTOR = 1.2f;
const float MAPPOINT_DEFAULT_MIN_DIST = 0.1f;
const float MAPPOINT_DEFAULT_MAX_DIST = 100.0f;
const int MAPPOINT_MIN_OBS_FOR_BAD = 2;

// 系统参数
const int SYSTEM_MAX_KFS_LOAD = 10000;
const int SYSTEM_MAX_MPS_LOAD = 500000;

// 地图文件参数
const uint32_t SYSTEM_MAP_FILE_MAGIC = 0x4D415031; // 'MAP1'
const uint32_t SYSTEM_MAP_FILE_VERSION = 1;

// 系统重定位配置
const int SYSTEM_RELOC_CONFIG_TOP_K = 20;
const int SYSTEM_RELOC_CONFIG_MAX_CANDIDATES = 5000;
const int SYSTEM_RELOC_CONFIG_MATCH_CHUNK = 500;
const int SYSTEM_RELOC_CONFIG_BG_SLEEP_US = 80000;
const int SYSTEM_RELOC_CONFIG_MAX_BIND_INLIERS = 80;
const int SYSTEM_RELOC_CONFIG_MAX_PROJ_BINDS = 30;

// ==========================================
// 智能后台线程调度参数
// ==========================================

// 主线程每帧绑定的最大地图点数，限制主线程开销避免卡顿
const int MAIN_THREAD_MAX_BIND_PER_FRAME = 50;

// 主线程绑定的最小匹配内点数阈值，只有当内点少于此阈值时才进行额外绑定
const int MAIN_THREAD_BIND_INLIER_THRESHOLD = 100;

// ==========================================
// 重定位对齐参数
// ==========================================

// 重定位对齐的最小内点数要求
const int RELOC_MIN_INLIERS_FOR_ALIGN = 15;

// 重定位对齐的最小置信度要求
const float RELOC_MIN_CONFIDENCE_FOR_ALIGN = 0.4f;

// 连续丢失多少帧后创建新子地图（30帧 @ 30fps = 1秒）
const int TRACKING_LOST_FRAMES_FOR_NEW_MAP = 30;

}

#endif // CONFIG_H
