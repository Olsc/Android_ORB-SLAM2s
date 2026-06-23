#ifndef CONFIG_H
#define CONFIG_H

namespace ORB_SLAM2
{

// 统一配置参数管理

// ==========================================
// 相机参数（需根据实际标定结果修改）
// ==========================================

// fx, fy：相机焦距（像素单位）。值越大，相同物理距离对应的像素位移越大，
//   直接影响三角化深度估计的敏感度。若改大，同等视差下深度估计更精确，
//   但搜索窗口内匹配的容差下降。
const float CAMERA_FX = 640.0f;
const float CAMERA_FY = 640.0f;

// cx, cy：相机主点坐标（像素单位），即光轴与成像平面的交点。
//   通常为图像中心附近，若与实际标定值偏差过大会导致投影误差系统性偏移。
const float CAMERA_CX = 320.0f;
const float CAMERA_CY = 180.0f;

// k1, k2, p1, p2, k3：径向畸变(k1/k2/k3)和切向畸变(p1/p2)系数。
//   使用 Brown-Conrady 模型。全为0表示已事先去畸变或使用无畸变相机。
//   若实际相机畸变明显且未设正确值，边缘区域的匹配精度会显著下降。
const float CAMERA_K1 = 0.0f;
const float CAMERA_K2 = 0.0f;
const float CAMERA_P1 = 0.0f;
const float CAMERA_P2 = 0.0f;
const float CAMERA_K3 = 0.0f;

// 相机帧率（帧/秒）。用于时间戳累加、运动模型预测速度推断。
//   若设置偏高，速度估计会偏慢，运动模型搜索窗口可能偏小导致跟丢；
//   若设置偏低则反之。
const float CAMERA_FPS = 30.0f;

// 图像颜色顺序（仅影响显示/AR渲染层，不影响SLAM核心。
//   SLAM核心对灰度图操作，颜色顺序仅用于最终可视化）。
//   0 = BGR（OpenCV默认）, 1 = RGB。
const int CAMERA_RGB = 1;

// ==========================================
// ORB特征提取参数
// ==========================================

// 每帧图像期望提取的特征点数量。提取器会尽量接近此值，但在纹理贫乏区域可能不足。
//   增大此值会增加匹配鲁棒性，但会线性增加后续匹配/优化/BOW的计算量。
const int ORB_EXTRACTOR_N_FEATURES = 1000;

// 图像金字塔相邻层之间的缩放因子。每层图像尺寸为上一层的 1/scale_factor。
//   1.2f 是 ORB-SLAM 标准值，意味第 0 层到第 7 层分辨率逐层降低。
//   增大此值（如1.5f）会减少金字塔有效层数，降低尺度不变性；
//   减小则反之，但会增加提取耗时。
const float ORB_EXTRACTOR_SCALE_FACTOR = 1.2f;

// 金字塔总层数，包含最底层（原始分辨率层）。
//   每增加一层，算法对更大尺度变化的鲁棒性提高，但总特征数和提取时间增加。
//   实际有效尺度覆盖范围为 scale_factor^(n_levels-1)。
const int ORB_EXTRACTOR_N_LEVELS = 8;

// FAST角点检测的初始阈值（INI_TH）和最小阈值（MIN_TH）。
//   在纹理丰富的区域用 INI_TH 提取，不足时降至 MIN_TH 以补足特征数。
//   值越大，提取的角点越"尖锐"，数量越少，但重复性更好；
//   值越小，提取越多角点，但质量下降，且匹配时噪声增加。
const int ORB_EXTRACTOR_INI_TH_FAST = 20;
const int ORB_EXTRACTOR_MIN_TH_FAST = 7;

// ORB描述子列数，用于描述子矩阵的列数定义。
//   每个描述子为32字节（256位），对应 BRIEF 模式的32个字节对。
//   此值与 ORB 标准定义固定，如需修改须同步修改所有描述子操作相关的代码。
const int ORB_DESC_COLS = 32;

// ==========================================
// 性能/内存限制（限定地图规模上限）
// ==========================================

// 地图中允许的最大关键帧数量。
//   当达到此上限时，系统会调用 CullKeyFrame() 逐步剔除冗余关键帧。
//   设置越小内存占用越稳定，但可能剔除过多导致跟踪丢失后的重定位能力下降。
//   建议根据目标平台内存容量调节：
//     手机端推荐 500-1000，桌面端推荐 2000-5000。
const int MAX_KEYFRAMES = 2000;

// 地图中允许的最大地图点数量。
//   当达到上限时，系统从最早创建的地图点开始剔除。
//   设得过小会导致地图稀疏，跟踪精度下降；过大则增加内存和优化耗时。
const int MAX_MAPPOINTS = 10000;

// 当达到 KF 限制时，单次剔除操作移除的关键帧数量。
//   增大此值可更快释放内存但可能造成更剧烈的质量抖动。
const int KEYFRAME_CULL_BATCH_SIZE = 5;

// 当达到 MapPoint 限制时，单次剔除操作移除的地图点数量。
const int MAPPOINT_CULL_BATCH_SIZE = 500;


// ==========================================
// 修剪阈值（控制关键帧/地图点淘汰策略）
// ==========================================

// 关键帧冗余判定阈值。
//   当某关键帧中超过 90% 的地图点同时被至少 3 个其他关键帧观测到时，
//   该帧被视为冗余并被剔除，以保持后端 BA 规模可控。
//   调高此值（如0.95f）会保留更多关键帧，建图更密集但计算量增大。
const float KEYFRAME_REDUNDANCY_THRESHOLD = 0.9f;

// 判定地图点冗余时所需的"其他关键帧观测数"下限。
//   观测数 ≥ 此值的点不参与关键帧冗余性判定（因为它们足够可靠）。
//   增大此值使淘汰更激进（更多点被计入判定），减小则更保守。
const int KEYFRAME_REDUNDANCY_OBS_THRESHOLD = 3;

// 地图点被视为"优质"所需的最小观测关键帧数。
//   低于此观测数的点会在帧修剪中被优先剔除。
//   本项目为单目模式，仅使用单目阈值。
const int MAPPOINT_MIN_OBSERVATIONS_MONO = 2;


// ==========================================
// 跟踪参数（影响 Track() 全过程的行为）
// ==========================================

// 局部地图点最大数量限制。
//   在 TrackLocalMap() 中，投影搜索的局部地图点超过此数时截断，
//   防止单帧投影匹配耗时过高。
//   对低端设备可减小至 2000，高端设备可以不限制（放大至 10000）。
const int TRACKING_MAX_LOCAL_MAP_POINTS = 5000;

// 视锥可见性判定阈值（0~1）。
//   在 SearchLocalPoints() 中调用 isInFrustum() 时用于判断地图点是否在视野内。
//   0.5f 表示地图点投影位置偏离图像中心不超过图像尺寸 50% 时仍视为可见。
//   增大此值（如 0.7f）会使更多远处/边缘点参与匹配搜索，但增加误匹配风险；
//   减小（如 0.3f）则更严格，减少计算量但可能漏掉边缘特征。
const float FRUSTUM_VISIBILITY_TH = 0.5f;

// PnP求解中 2D 像素坐标的有效范围上限。
//   若匹配点投影后的像素坐标绝对值超过此值，视为投影异常剔除。
//   主要用于防止畸变校正失败导致坐标爆炸。
const float PNP_LIMIT_2D = 1e5f;

// PnP求解中 3D 地图点坐标的有效范围上限（米）。
//   若地图点坐标绝对值超过此值，视为深度异常点剔除。
//   主要用于防止错误的三角化产生"飞点"。
const float PNP_LIMIT_3D = 1e6f;

// PnP RANSAC 求解参数：
//   ITERATIONS：RANSAC最大迭代次数，越大找到正确解的概率越高，但耗时线性增加。
//   ERROR：RANSAC内点重投影误差阈值（像素），增大则内点更多但可能引入外点。
//   CONFIDENCE：期望达到的置信概率，0.999 表示 99.9% 概率至少有一次采样全为内点。
const int PNP_RANSAC_ITERATIONS = 200;
const float PNP_RANSAC_ERROR = 6.0f;
const double PNP_RANSAC_CONFIDENCE = 0.999;

// 地图对齐后跟踪搜索半径：
//   ALIGNED   = 12.0：已对齐状态下重投影搜索半径（像素）。
//              对齐后姿态较准，可用较大半径提高召回率。
//   UNALIGNED = 8.0：未对齐状态下搜索半径（像素）。
//              未对齐时位置不确定，用小半径避免大量误匹配。
const float TRACKING_SEARCH_RADIUS_ALIGNED = 12.0f;
const float TRACKING_SEARCH_RADIUS_UNALIGNED = 8.0f;

// 加载点投影绑定时允许的最大深度（米）。
//   在 BindLoadedMapPointsUsingSnapshots() 中将参考快照点投影到当前帧时，
//   若相机坐标系下深度 Z > 此值则跳过，避免匹配到极远处的低质量点。
//   设得过大（如 100）可匹配到更远的点但易引入深度不确定性大的误匹配；
//   设得过小（如 20）会丢弃远处真实点，减少可绑定点数量。
const float BIND_MAX_DEPTH = 50.0f;

// 全局重定位搜索半径（单位：米）。
//   在重定位阶段，将当前帧的 Bow 候选关键帧的地图点投影到当前帧时，
//   以此半径在图像上确定搜索范围。
//   值越大容差越好但计算量增大，值小则可能漏掉候选。
const float TRACKING_RELOC_SEARCH_RADIUS = 50.0f;

// 重定位成功后高精度投影绑定的搜索半径（像素）。
//   在 TrackLocalMap() 中消费后台对齐结果后，将已加载地图点投影到当前帧，
//   以此半径在图像上搜索最近邻特征点进行绑定。
//   值越大绑定召回率越高但计算量和误匹配增加；
//   值小则更保守，适合已对齐到位后做精细化补充匹配。
const float RELOC_PROJ_SEARCH_RADIUS = 15.0f;

// 跟踪对齐更新参数：
//   MIN_INLIERS_UPDATE      = 20：触发对齐更新的最小内点数下限。
//   MIN_CONFIDENCE_UPDATE   = 0.3f：触发更新所需的最小内点比例。
//   SMOOTH_SKIP_FRAMES      = 3：连续对齐更新之间的间隔帧数，防止抖动。
// 以上三个参数共同控制对齐更新的触发频率和条件。
const int TRACKING_ALIGN_MIN_INLIERS_UPDATE = 20;
const float TRACKING_ALIGN_MIN_CONFIDENCE_UPDATE = 0.3f;
const int TRACKING_ALIGN_SMOOTH_SKIP_FRAMES = 3;

// 参考缓存重试上限和缓存条目限制。
//   若从缓存加载参考帧失败，最多重试 MAX_RETRIES 次。
//   CACHE_LIMIT 控制缓存最大条目数，超过时淘汰最旧的条目。
const int TRACKING_MAX_REF_CACHE_RETRIES = 5;
const int TRACKING_REF_CACHE_LIMIT = 30000;

// 主线程绑定加载点时，参考快照在空间的网格搜索半径（米）。
//   从当前相机位置出发，在此半径内搜索已绑定的地图点。
//   增大可让主线程更早发现远处点，但搜索范围增大会增加CPU开销。
const float TRACKING_GRID_SEARCH_RADIUS = 40.0f;

// 绑定加载点时，若总候选点数量超过此阈值，启用步长采样机制，
//   避免单帧绑定遍历全部候选导致卡顿。
//   设为 0 可禁用步长采样。
const int TRACKING_CANDIDATE_STRIDE_THRESHOLD = 3000;

// 后台重定位姿态门控：候选关键帧数量超过此值时才启用姿态粗筛。
//   若候选数量太少直接进入精匹配而不做粗筛选，避免小样本下误筛。
const int TRACKING_GATING_MIN_CANDIDATES = 50;

// 后台重定位 KNN 匹配比率阈值（越小越严格）。
//   类似 Lowe's ratio test：若 最近距离/次近距离 > 此值，丢弃匹配。
//   0.75 是标准值；调低（如 0.6）会获得更少但更可靠的匹配。
const float TRACKING_KNN_RATIO = 0.75f;

// 后台重定位 KNN 描述子距离阈值（HAMMING 距离）。
//   超过此距离的匹配直接丢弃，用于过滤明显错误的描述子匹配。
//   BRIEF 描述子的典型有效距离范围是 0-128。
//   60 对应约 23% 的比特翻转容忍度。
const int TRACKING_KNN_DIST_MAX = 60;

// 后台重定位 PnP 姿态求解所需的最小内点数。
//   低于此数则认为 PnP 失败。
//   调高可防止错误姿态被接受，但也会增加重定位困难的场景下的失败率。
const int TRACKING_RELOC_PNP_MIN_INLIERS = 10;

// 后台重定位冷却帧数。
//   当一次 PnP 尝试失败后，等待此帧数再发起下一次重定位，避免无用功。
const int TRACKING_RELOC_COOLDOWN_FRAMES = 5;

// 后台重定位 PnP 求解的最大采样数。
//   在 GlobalRelocLoop() 中对 2D-3D 候选点对进行去重后，
//   按描述子距离升序截断到此上限，控制 RANSAC 求解的代价上限。
//   增大可在更多候选点上搜索更优姿态，但 RANSAC 耗时线性增加。
//   建议范围 100~500，移动端推荐 200。
const int RELO_BG_PNP_MAX_SAMPLES = 200;

// 后台重定位匹配分数的归一化分母。
//   在 GlobalRelocLoop() 中计算 matchScore = min(1.0, keptPairs / (N * 0.5))
//   时使用的分母基数。实际分母为此值（或 N*0.5 的较大者）。
//   此分数用于 UI 展示进入目标区域的可能性，不影响算法精度。
const float RELO_MATCH_SCORE_DIVISOR = 20.0f;

// 单目初始化阶段所需的最小 ORB 匹配点数。
//   若两帧间高质量匹配数低于此值，认为场景缺乏纹理或运动不足，无法初始化。
const int TRACKING_INIT_MIN_MATCHES = 100;

// TrackWithMotionModel 的搜索窗口半径（像素）。
//   在运动模型预测的位置附近，以此半径搜索特征点匹配。
//   增大可应对快速运动，但误匹配增加。
const int TRACKING_MOTION_SEARCH_TH = 15;

// TrackWithMotionModel 判定成功所需的最小匹配数。
//   若匹配数低于此值，认为运动模型跟踪失败，降级至参考关键帧跟踪。
const int TRACKING_MOTION_MIN_MATCHES = 20;

// TrackReferenceKeyFrame 判定成功所需的最小匹配数。
//   低于此值时降级到重定位或 TrackLocalMap 中更激进的搜索。
const int TRACKING_REFKF_MIN_MATCHES = 15;

// 参考关键帧匹配判定时，地图点所需的最小观测数。
//   在 NeedNewKeyFrame() 中调用 TrackedMapPoints(nMinObs) 时使用，
//   用于统计参考关键帧中"可靠"的地图点数量。
//   增大此值会使关键帧统计更严格（仅计高度观测的点），
//   减小则宽松但仍过滤掉零观测的孤立点。
const int REFKF_MIN_OBSERVATIONS = 3;

// 参考关键帧跟踪匹配比率阈值。
//   RATIO：正常建图阶段，匹配与预测比率的容忍度。
//   NEWMAP_RATIO：新地图（关键帧 < 2）时放宽阈值，因为初始帧匹配不稳定。
//   MONO_RATIO：单目模式下阈值，单目尺度不确定性更大故更严格。
// 三个参数配合控制 TrackReferenceKeyFrame 是否能通过比率检验。
const float TRACKING_KF_REF_RATIO = 0.75f;
const float TRACKING_KF_NEWMAP_RATIO = 0.4f;
const float TRACKING_KF_MONO_RATIO = 0.9f;

// SearchLocalPoints 投影搜索的窗口半径（像素）。
//   将局部地图点投影到当前帧图像时，在此半径内搜索匹配。
//   值越大越容易匹配到远处点但计算量增大；值小则更快但可能遗漏。
//   建议在快速运动场景适当增大。
const int TRACKING_LOCAL_SEARCH_TH = 1;

// 局部地图点数不足时的补充上限。
//   在 TrackLocalMap() 中若 mvpLocalMapPoints < 50，从全局地图点中
//   补充最多此数量的点进入局部地图，确保跟踪有足够候选。
//   增大可在建图初期提供更多匹配候选，但增加 SearchLocalPoints 的计算量。
//   设为 0 可禁用补充机制。
const int LOCAL_MAP_SUPPLEMENT_COUNT = 200;

// 连续跟踪失败宽容次数。
//   当已对齐状态下连续 TRACKING_MAX_CONSECUTIVE_FAIL 帧内点数均偏低时，
//   触发从对齐状态降级到未对齐或重定位。
const int TRACKING_MAX_CONSECUTIVE_FAIL = 3;

// 连续失败帧宽限时的内点数基线。
//   在 TrackLocalMap() 中，若 mbHaveMapAlign 且 mnMatchesInliers >= 此值，
//   则重置连续失败计数 mConsecutiveFail = 0；否则递增。
//   当内点数低于此值但未达 TRACKING_MAX_CONSECUTIVE_FAIL 上限时，
//   系统仍判定跟踪成功但会累积失败计数，避免单帧抖动导致立即丢失。
const int CONSECUTIVE_FAIL_INLIERS_BASELINE = 20;

// 重定位 PoseOptimization 后判定成功的最小内点数。
//   少于此时认为重定位失败，继续搜索下一组候选。
const int TRACKING_POSE_OPT_MIN_INLIERS = 10;

// TrackLocalMap 跟踪成功与否的多级阈值：
//   STRICT = 30：严格阈值，内点 >= 30 认为跟踪质量优秀。
//   LOOSE = 15：宽松阈值，用于判断跟踪是否存在但质量一般。
//   LOADED = 10：地图加载后的最低容忍阈值，刚加载时匹配可能不足。
const int TRACKING_SUCCESS_STRICT = 30;
const int TRACKING_SUCCESS_LOOSE = 15;
const int TRACKING_SUCCESS_LOADED = 10;

// 已对齐状态下 STRICT 阈值的覆盖值。
//   在 TrackLocalMap() 中，当 mbHaveMapAlign = true 时，
//   将 TRACKING_SUCCESS_STRICT 动态覆盖为此值（默认 20），
//   因为已对齐状态下加载地图点的内点统计规则不同，
//   过高的 STRICT 阈值会导致已对齐状态下频繁判定跟踪失败。
const int ALIGNED_STRICT_INLIERS_OVERRIDE = 20;

// 局部关键帧列表的最大数量限制。
//   在 UpdateLocalKeyFrames() 中，当 mvpLocalKeyFrames.size() 超过此值时
//   停止添加新的邻居/子/父关键帧。
//   增大使局部地图包含更多约束，BA 优化更准但耗时增加；
//   减小则效率优先，适合低端设备。
const int MAX_LOCAL_KEYFRAMES = 80;

// 共视图邻居关键帧的最佳共视数量。
//   在 UpdateLocalKeyFrames() 中，对每个局部关键帧调用
//   GetBestCovisibilityKeyFrames(COVISIBILITY_NEIGHBOR_COUNT)
//   将其共视关系最好的前 N 个关键帧也纳入局部地图。
//   增大使局部地图连接更紧密，但可能引入冗余帧。
const int COVISIBILITY_NEIGHBOR_COUNT = 10;

// ==========================================
// 初始化器参数（单目初始化阶段）
// ==========================================

// 初始化时所需的最小平均视差（单位：像素位移均值）。
//   低于此值时认为两帧间基线太短，推迟初始化等待更远基线。
//   增大此值使初始化更保守（避免纯旋转或极短基线导致的错误初始化），
//   减小可使系统在弱运动下更快启动，但可能因视差不足导致地图畸变。
const float INITIALIZER_MIN_PARALLAX_PX = 8.0f;

// 初始化超时时间（秒）。
//   即使视差一直不足，超过此时间也强制尝试初始化（基于当前最佳匹配）。
//   避免在静止或纯旋转场景下永远无法初始化。
//   设为 0 可禁用超时强制初始化。
const double INITIALIZER_TIMEOUT_SEC = 3.0;

// 初始化创建初始地图所需的最小跟踪点数。
//   低于此值即使视差充足也不创建初始地图，避免初始地图过于稀疏。
const int INITIALIZER_MIN_TRACKED_POINTS = 100;


// ==========================================
// ORB特征提取参数（底层算子参数）
// ==========================================

// 计算描述子时的 Patch 大小（像素）。
//   在关键点周围取 31×31 的像素块用于计算 BRIEF 描述子。
//   增大 patch 会提高描述子区分度但增加计算量和边缘效应。
const int ORB_PATCH_SIZE = 31;

// ORB Patch 有效半径（= patch_size/2 向下取整）。
//   用于描述子方向计算的圆形区域内半径。
const int ORB_HALF_PATCH_SIZE = 15;

// 关键点检测时的图像边缘阈值（像素）。
//   在图像边界 ORB_EDGE_THRESHOLD 像素范围内不提取特征点。
//   避免提取到边界外的无效特征，设为 19 相当于保证 Patch 在图像内。
const int ORB_EDGE_THRESHOLD = 19;


// ==========================================
// 闭环检测参数
// ==========================================

// 距离上一次闭环检测的最小间隔帧数。
//   防止在短时间内反复触发闭环校正。
//   设得太小可能导致频繁闭环造成抖动；太大则闭环响应变慢。
const int LOOP_MIN_FRAMES_SINCE_LAST = 10;

// 闭环候选关键帧与当前帧匹配所需的最少特征匹配数。
//   低于此值不认为存在闭环可能性。
const int LOOP_MIN_MATCHES = 20;

// Sim3 投影匹配后，判定闭环有效所需的最小匹配数。
//   比 LOOP_MIN_MATCHES 更严格，因为已经过 Sim3 几何验证。
const int LOOP_MIN_MATCHES_AFTER_PROJ = 40;

// 闭环检测的共视一致性阈值。
//   候选关键帧需要至少有此数量的连续共视组支持才能被确认为有效闭环。
//   调高减少误检但可能漏检真实闭环。
const int LOOP_COVISIBILITY_CONSISTENCY_TH = 3;

// Sim3 计算的 RANSAC 参数：
//   RANSAC_PROB = 0.99：期望置信概率。
//   RANSAC_MIN_INLIERS = 20：Sim3求解所需的最小内点数。
//   RANSAC_MAX_ITERS = 300：最大迭代次数。
const double LOOP_RANSAC_PROB = 0.99;
const int LOOP_RANSAC_MIN_INLIERS = 20;
const int LOOP_RANSAC_MAX_ITERS = 300;


// ==========================================
// 图优化参数（g2o 后端优化器）
// ==========================================

// Huber 鲁棒核函数的阈值（卡方值）。
//   2D（像素误差）：sqrt(5.991) ≈ 2.448，对应卡方 2DoF 95% 置信。
//   3D（空间误差）：sqrt(7.815) ≈ 2.796，对应卡方 3DoF 95% 置信。
//   误差超过此值的观测项会被降低权重以减小外点影响。
const float OPTIMIZER_HUBER_TH_2D = 2.4476519f; // sqrt(5.991)
const float OPTIMIZER_HUBER_TH_3D = 2.79553215f; // sqrt(7.815)

// 卡方检验的阈值，用于判断误差项是否为外点：
//   2DoF（2D投影）：5.991（95%置信）。
//   1DoF（1D误差）：3.841（95%置信）。
//   误差项的卡方值大于对应阈值则视为外点，在优化中被剔除。
const float OPTIMIZER_CHI2_TH_2D = 5.991f;
const float OPTIMIZER_CHI2_TH_1D = 3.841f;

// Sim3 求解器的卡方阈值（2DoF 99% 置信）。
//   用于 Sim3 优化中判断内点/外点。
const float SIM3_CHI2_TH = 9.210f;

// Sim3 RANSAC 求解参数
const double SIM3_RANSAC_PROB = 0.99;
const int SIM3_RANSAC_MIN_INLIERS = 6;
const int SIM3_RANSAC_MAX_ITERS = 300;

// ORB 匹配器的距离阈值：
//   TH_HIGH = 100：保守匹配的高阈值（HAMMING距离），用于高质量匹配。
//   TH_LOW  = 50：宽松匹配的低阈值，用于需要更多候选的场景。
//   HISTO_LENGTH = 30：旋转直方图的区间数，用于一致性检验。
//   VIEW_COS_TH = 0.998f：视图方向余弦阈值，超过此角度视为视角过大不匹配。
//   EPILINE_TH = 3.84f：极线约束阈值（卡方 1DoF），用于立体匹配验证。
const int ORB_MATCHER_TH_HIGH = 100;
const int ORB_MATCHER_TH_LOW = 50;
const int ORB_MATCHER_HISTO_LENGTH = 30;
const float ORB_MATCHER_VIEW_COS_TH = 0.998f;
const float ORB_MATCHER_EPILINE_TH = 3.84f;

// 初始化器 RANSAC 参数：
//   RANSAC_MIN_SET = 8：单应矩阵/基础矩阵估计所需的最小匹配点数。
//   H_SCORE_RATIO = 0.40f：当 score_H / (score_H + score_F) > 此值时选择 H。
//   MIN_PARALLAX = 1.0f：最小视差阈值（像素），用于判定是否具有足够的立体基线。
//   MIN_TRIANGULATED = 50：三角化成功的最小点数，低于此值认为初始化地图太小。
const int INITIALIZER_RANSAC_MIN_SET = 8;
const float INITIALIZER_H_SCORE_RATIO = 0.40f;
const float INITIALIZER_MIN_PARALLAX = 1.0f;
const int INITIALIZER_MIN_TRIANGULATED = 50;

// PnP RANSAC 求解参数（求解模块 & 重定位通用）：
//   PROB = 0.99：期望置信概率。
//   MIN_INLIERS = 10：接受求解结果的最小内点数。(注:原ORB-SLAM2默认为8,
//                   实际重定位场景要求更严格，已在PnPsolver默认参数中统一更新)
//   MAX_ITERS = 300：最大迭代次数。
//   MIN_SET = 4：P3P 算法最少需要 4 组匹配。
//   EPSILON = 0.5f：内点判定时的相对误差容限因子。
//   TH2 = 5.991f：重投影卡方阈值（2DoF 95% 置信）。
const double PNP_RANSAC_PROB = 0.99;
const int PNP_RANSAC_MIN_INLIERS = 10;
const int PNP_RANSAC_MAX_ITERS = 300;
const int PNP_RANSAC_MIN_SET = 4;
const float PNP_RANSAC_EPSILON = 0.5f;
const float PNP_RANSAC_TH2 = 5.991f;

// 帧网格划分参数：
//   GRID_ROWS = 48：图像在垂直方向划分的网格数。
//   GRID_COLS = 64：水平方向网格数。
//   网格用于加速特征匹配时的空间搜索（仅需搜索相邻网格内的特征点）。
//   总网格数 48*64=3072，若图像 640×360，每格约 13.3×7.5 像素。
//   RESERVE_FACTOR = 0.5f：网格预留空间系数，避免特征点溢出边界。
const int FRAME_GRID_ROWS = 48;
const int FRAME_GRID_COLS = 64;
const float FRAME_GRID_RESERVE_FACTOR = 0.5f;

// 关键帧成为"连接关键帧"所需的共视地图点最小数量。
//   两个关键帧之间共享至少 15 个地图点才建立连接边（用于共视图构建）。
//   增大则共视图更稀疏，BA 更快但信息连接不足；减小则图更稠密。
const int KEYFRAME_CONNECTION_TH = 15;

// 地图点深度不变性范围因子：
//   MIN_DIST_INVARIANCE_FACTOR = 0.8f：最小观测距离 = 基准距离 × 0.8。
//   MAX_DIST_INVARIANCE_FACTOR = 1.2f：最大观测距离 = 基准距离 × 1.2。
//   在此范围外观测该地图点，描述子尺度层可能不匹配导致匹配质量下降。
const float MAPPOINT_MIN_DIST_INVARIANCE_FACTOR = 0.8f;
const float MAPPOINT_MAX_DIST_INVARIANCE_FACTOR = 1.2f;

// 地图点的默认最近/最远观测距离（米），在深度信息缺失时使用。
const float MAPPOINT_DEFAULT_MIN_DIST = 0.1f;
const float MAPPOINT_DEFAULT_MAX_DIST = 100.0f;

// 标记地图点为"不良"所需的最小观测数。
//   当某地图点的实际观测帧数少于理论观测帧数 × MIN_FOUND_RATIO 时，
//   且观测数 < MIN_OBS_FOR_BAD，标记为不良以待剔除。
const int MAPPOINT_MIN_OBS_FOR_BAD = 2;

// 地图点被找到的最低比率。
//   若 被找到次数 / 期望观测次数 < 此值，标记为不良。
const float MAPPOINT_MIN_FOUND_RATIO = 0.25f;

// 系统加载地图文件时的容量上限：
//   MAX_KFS_LOAD = 10000：最多加载 10000 个关键帧。
//   MAX_MPS_LOAD = 500000：最多加载 500000 个地图点。
//   适用于离线构建的大地图加载到移动端时的裁剪。
const int SYSTEM_MAX_KFS_LOAD = 10000;
const int SYSTEM_MAX_MPS_LOAD = 500000;

// 地图文件的魔数（magic number）和版本号：
//   FILE_MAGIC = 0x4D415031，即 ASCII "MAP1"，用于文件格式校验。
//   FILE_VERSION = 1：当前格式版本，升级不兼容格式时需递增。
const uint32_t SYSTEM_MAP_FILE_MAGIC = 0x4D415031;
const uint32_t SYSTEM_MAP_FILE_VERSION = 1;

// 系统重定位配置：
//   TOP_K = 20：检索 Top-K 最佳候选关键帧。
//   MAX_CANDIDATES = 5000：全局检索的最大候选数。
//   MATCH_CHUNK = 500：分块匹配的批次大小，控制内存峰值。
//   BG_SLEEP_US = 80000：后台线程轮询间隔（微秒），控制 CPU 占用率。
//   MAX_BIND_INLIERS = 80：绑定加载时最大内点数限制。
//   MAX_PROJ_BINDS = 30：投影绑定阶段最大绑定数限制。
const int SYSTEM_RELOC_CONFIG_TOP_K = 20;
const int SYSTEM_RELOC_CONFIG_MAX_CANDIDATES = 5000;
const int SYSTEM_RELOC_CONFIG_MATCH_CHUNK = 500;
const int SYSTEM_RELOC_CONFIG_BG_SLEEP_US = 80000;
const int SYSTEM_RELOC_CONFIG_MAX_BIND_INLIERS = 80;
const int SYSTEM_RELOC_CONFIG_MAX_PROJ_BINDS = 30;

// ==========================================
// 智能后台线程调度参数
// ==========================================

// 主线程每帧最多绑定的地图点数量。
//   限制每帧的主线程绑定工作量，避免长时间占用 CPU 导致渲染卡顿。
//   增大在快速运动时可更快建立绑定但增加帧耗时。
const int MAIN_THREAD_MAX_BIND_PER_FRAME = 50;

// 主线程绑定的最小内点数阈值。
//   只有当当前帧的内点数低于此值时，才触发额外绑定操作。
//   若当前跟踪内点已充足（> 此值），跳过额外绑定以节省 CPU。
const int MAIN_THREAD_BIND_INLIER_THRESHOLD = 100;

// ==========================================
// 重定位对齐参数
// ==========================================

// 重定位对齐的最小内点数要求。
//   重定位成功后若内点不足此数，认为对齐质量不够，仍保持在丢失状态。
const int RELOC_MIN_INLIERS_FOR_ALIGN = 15;

// 重定位对齐的最小置信度（内点比例）要求。
//   内点数/总点数 < 此值时认为对齐不可靠。
const float RELOC_MIN_CONFIDENCE_FOR_ALIGN = 0.4f;

// 重定位后短期内 SearchLocalPoints 的投影搜索窗口半径（像素）。
//   在 SearchLocalPoints() 中，若当前帧在重定位后 2 帧内（mnId < mnLastRelocFrameId + 2），
//   使用此值替代 TRACKING_LOCAL_SEARCH_TH 进行更粗糙的搜索以提高召回。
//   增大可更快找回更多匹配，但误匹配增加；减小则更保守。
//   标准值 5，在稳定跟踪时可适当降低。
const int RELOC_POST_SEARCH_TH = 5;

// Reset() 后后台重定位的冷却帧数。
//   在 Reset() 完成后，设置 mRelocCooldownFrames 为此值，
//   在此期间后台重定位线程不发起 PnP 求解。
//   目的是等待足够的新扫描点积累后再尝试匹配，
//   避免 Reset 后立即匹配到错误区域导致错误对齐。
const int RESET_COOLDOWN_FRAMES = 30;

// 连续丢失超过此帧数后创建新的子地图（子地图策略）。
//   增大此值会延长等待时间但可能避免因短暂遮挡而错误创建新地图。
const int TRACKING_LOST_FRAMES_FOR_NEW_MAP = 30;

// 创建新子地图后的冷却帧数。
//   避免在连续丢失-找回的边界场景中高频触发子地图创建，
//   防止主跟踪线程被反复阻塞。150 帧 @ 30fps = 5 秒冷却。
const int TRACKING_NEW_MAP_COOLDOWN_FRAMES = 150;

// 子地图最大数量限制。
//   超出此数量时最旧的子地图被自动删除以释放内存。
//   每个子地图可能包含数千个关键帧和数万个地图点，建议根据内存定。
const int MAX_SUBMAP_COUNT = 10;


// ==========================================
// 局部建图参数（LocalMapping 线程）
// ==========================================

// 三角化时选取的邻近关键帧数量。
//   对每个新关键帧，在共视图中取 NEIGHBORS 个最佳共视帧来三角化新地图点。
//   增大可增加三角化的候选类型，但计算量线性增加。
const int LOCAL_MAPPING_TRIANGULATION_NEIGHBORS = 20;

// 三角化最小基线/中值深度比。
//   只有当两个关键帧的基线长度 > 当前场景中值深度 × 此比例时，才进行三角化。
//   太小的基线会导致三角化深度不确定性过大，产生"飞点"。
const float LOCAL_MAPPING_TRIANGULATION_BASELINE_RATIO = 0.01f;

// 三角化最小视差阈值（以视角夹角的 cos 值表示）。
//   0.9998f 对应约 1.14° 的最小视差角。
//   小于此视差的两个视图三角化深度不确定性过大，会直接放弃。
const float LOCAL_MAPPING_TRIANGULATION_PARALLAX_TH = 0.9998f;

// 三角化比率测试因子。
//   在描述子距离比率测试中，最佳匹配距离必须 < 次佳 × 此因子才接受。
//   降低此值（更严格）可减少误匹配但可能丢失许多有效匹配。
const float LOCAL_MAPPING_TRIANGULATION_RATIO_FACTOR = 1.5f;

// 邻近（一级）搜索的关键帧数上限。
//   在 LocalMapping 的处理中，建立的共视关系最多取前 NEIGHBOR_KFS 个。
const int LOCAL_MAPPING_NEIGHBOR_KFS = 20;

// 次级邻近搜索的关键帧数上限。
//   二级共视（共视的共视）搜索的上限，用于更广泛的约束。
const int LOCAL_MAPPING_SECOND_NEIGHBOR_KFS = 5;

// 新关键帧的修剪保护帧数。
//   新插入的关键帧在此帧数内不会被修剪，防止刚加入就因冗余性被剔除。
const int LOCAL_MAPPING_CULL_PROTECT_FRAMES = 5;

// LocalMapping 接受新关键帧时，输入队列中允许积压的最大数量。
//   若队列积压超过此值，主线程的 Track() 会阻塞等待 LocalMapping 消费。
//   增大可让主线程更流畅但增加建图延迟，设小则建图及时但主线程可能等待。
//   当 MAV 等高速场景建议增大，精度优先场景建议减小。
const int LOCAL_MAPPING_MAX_QUEUED_KFS = 3;

// ==========================================
// 基础图优化参数（Essential Graph BA）
// ==========================================

// 构建 Essential Graph 时，两关键帧间共享少于此数的地图点则跳过连线。
//   用于控制 Essential Graph 的密度：增大使图更稀疏（优化更快但约束少），
//   减小使图更稠密（优化更准但更慢）。
const int OPTIMIZER_ESSENTIAL_GRAPH_MIN_FEAT = 100;


// ==========================================
// 重定位优化参数
// ==========================================

// 关键帧早期剪枝时的最小共享词数（BoW 词汇树）。
//   候选关键帧与当前帧的 BoW 共享词数低于此值直接丢弃。
//   增大可减少候选数量加速检索，但可能丢掉真实闭环/重定位候选。
const int RELOC_MIN_SHARED_WORDS = 10;

// 重定位候选关键帧的 Top-K 最大数量。
//   最终参与 PnP 求解的候选帧不超过此数。
const int RELOC_MAX_CANDIDATES = 20;

// ==========================================
// 系统运行时参数
// ==========================================

// 等待 LocalMapping / LoopClosing 线程停止的超时时间（毫秒）。
//   在系统析构时发送停止信号后，最多等待此时间让线程自行退出。
//   超时后可能强制终止，可能导致内存泄漏或文件未写入完成。
const int LOOP_LOCALMAPPER_TIMEOUT_MS = 5000;

// 创建新子地图的冷却时间（毫秒）。
//   防止频繁触发子地图创建，避免 map 互斥锁反复竞争。
const int NEW_MAP_COOLDOWN_MS = 5000;

// ==========================================
// JNI 桥接层参数（与 native-lib.cpp 共享）
// ==========================================

// SLAM 工作帧率（帧/秒），用于 JNI 层的递进时间戳累加。
//   应与 CAMERA_FPS 一致，保持两套系统的时间同步。
const float SYSTEM_FPS = 30.0f;

// 原始图像下采样缩放因子（native-lib 侧使用）。
//   输入图像在送入 SLAM 前缩放至 1/DOWNSCALE_FACTOR 大小。
//   此值与 SLAM 核心的内参缩放计算配合，需与 Java 层保持同步。
const float IMAGE_DOWNSCALE_FACTOR = 2.0f;

// SLAM 工作基准分辨率（宽/高，像素），供 JNI 侧 AR 渲染布局计算使用。
//   配合下采样后的图像尺寸，用于投影矩阵和视口计算。
const float BASE_SLAM_WIDTH = 640.0f;
const float BASE_SLAM_HEIGHT = 360.0f;

// OpenGL 投影的裁剪面距离（近/远，单位与场景一致），供 JNI 侧 AR 渲染使用。
//   近裁剪面 ZNEAR 设得过大会裁掉近处的有效内容，
//   远裁剪面 ZFAR 设得过小会裁掉远处的物体。
const float PROJECTION_ZNEAR = 0.1f;
const float PROJECTION_ZFAR = 1000.0f;

// SLAM 丢失状态自动重置超时时间（秒），供 JNI 侧丢帧检测使用。
//   当跟踪丢失超过此时间且未恢复时，JNI 层触发系统重置。
const double LOST_RESET_TIMEOUT = 3.0;

// 地图切换确认所需的连续帧数，供 JNI 侧地图切换检测使用。
//   连续 MAP_SWITCH_THRESHOLD 帧请求地图切换后才真正执行切换。
const int MAP_SWITCH_THRESHOLD = 3;

// 启用 AR 模式所需的最少新增地图点数，供 JNI 侧 AR 启用判定使用。
//   确保 SLAM 已有足够建图稳定后才开启 AR，防止 AR 物体漂移。
const int MIN_NEW_POINTS_BEFORE_AR = 50;

// AR 物体的默认缩放系数，供 JNI 侧 AR 物体渲染使用。
const float AR_OBJECT_SCALE_DEFAULT = 0.20f;

// 平面检测成功/失败的状态码，供 JNI 侧渲染层判断使用。
const int PLANE_DETECTED = 233;
const int PLANE_NOT_DETECTED = 1234;

// 平面检测 RANSAC 迭代次数，供 JNI 侧平面检测调用使用。
//   增大提高平面拟合鲁棒性（尤其是有噪点云时），但增加耗时。
const int PLANE_DETECT_RANSAC_ITERS = 50;

}

#endif // CONFIG_H
