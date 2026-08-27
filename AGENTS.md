# AGENTS.md

本文件面向在 `Android_ORB-SLAM2s` 仓库中工作的 AI 编码代理。请先完整阅读，再开始修改代码。

## 项目是什么

**MenthaAR / ORB-SLAM2s** 是一个基于 ORB-SLAM2 的 Android 单目视觉 SLAM + AR 演示项目：

- 实时稀疏点云建图、地图保存/加载、重定位与多地图匹配。
- 平面检测、基于 Google Filament 的 GLB/glTF 3D 模型渲染、OpenGL ES 点云与 3DOF 跟踪。
- 通过 Android 独立进程 + Binder IPC + 共享内存（Ashmem/memfd）实现 **UI 进程** 与 **SLAM 引擎进程** 隔离。

## 模块与许可边界

| 路径 | 作用 | 许可证 |
|---|---|---|
| `app/` | Android 应用壳：UI、CameraX、GLSurfaceView、Filament、传感器、地图管理 | Apache-2.0 |
| `MenthaAR/` | SLAM 引擎：AIDL `SlamService`、JNI `NativeHelper`、C++ ORB-SLAM2 核心、Thirdparty 依赖 | 引擎核心 GPL-3.0；IPC Java/AIDL 按文件头标注 Apache-2.0/GPL-3.0 双许可 |
| `docs/` | 多语言 README、贡献者协议、基准/分析工具文档 | 见各文件 |
| `.github/` | CI、Issue/PR 模板、CLA | 仓库规范 |

> 许可边界是该项目的重要设计：`app` 与 `MenthaAR` 通过 Binder + SharedMemory 通信，避免在 Apache 模块中直接链接 GPL 引擎。修改时不要破坏这种进程隔离，也不要随意变更文件头许可声明。

## 技术栈与版本

- 语言：Java（App 层，无 Kotlin）、C++11（原生 SLAM 核心）。
- Android Gradle Plugin：`9.3.2`；Gradle Wrapper：`9.7.1`。
- `compileSdk`/`targetSdk`：`37`；`minSdk`：`app=23`、`MenthaAR=21`。
- NDK：`27.0.12077973`；CMake：`3.22.1`。
- 主要依赖：CameraX、AppCompat/Material、Filament `1.75.0`、Markwon、Guava。
- 原生依赖：OpenCV（git submodule）、g2o、srrg_hbst、Eigen、DBoW2 等位于 `MenthaAR/src/main/cpp/Thirdparty/`。

## 仓库结构速览

```
app/src/main/java/com/orb/slam2s/
  app/         隐私同意、启动与权限
  camera/      CameraX 采集、Y 转换、GL 预览、点云叠加
  graphics/    GL 直通、点云渲染、Filament 模型渲染、3DOF 立方体
  ui/          MainActivity、虚拟摇杆
  util/        FPS、地图管理、手势
MenthaAR/src/main/
  aidl/        ISlamService.aidl
  java/com/orb/slam2s/
    constant/   GlobalConstant（分辨率/旋转）
    ipc/        SlamIPCClient、SlamService、SharedMemoryBuffer
    sensors/    OrientationSensor
    slamar/     NativeHelper（JNI 入口）
  cpp/
    include/    ORB-SLAM2 核心头文件（System、Tracking、Map 等）
    src/        ORB-SLAM2 核心 .cc 实现
    native-lib.cpp   JNI + 共享内存帧处理 + AR 锚点逻辑
    Plane.cpp/UIUtils.cpp/Matrix.cpp 等扩展
    profiler/   性能分析器（默认关闭）
    ubuntu/, windows/  非 Android 调试/基准程序，不参与 Android 构建
  CMakeLists.txt  OpenCV/g2o/MenthaAR_Engine 构建
```

## 构建与验证

在项目根目录使用 Gradle Wrapper：

```powershell
# 完整 Debug APK（推荐首选验证方式）
.\gradlew.bat assembleDebug

# 只构建 App 模块（会连带构建 MenthaAR）
.\gradlew.bat :app:assembleDebug

# 清理后构建（原生 OpenCV 编译较慢，慎重使用 clean）
.\gradlew.bat clean assembleDebug

# 安装到已连接设备
.\gradlew.bat installDebug
```

CI 使用 JDK 17、Ubuntu、`gradlew assembleDebug`；另一个 workflow 执行 `./gradlew build -x lint`。

注意：

- `lint` 在根/App/MenthaAR 构建脚本中被全局禁用，不要试图“修复”这一配置。
- 原生构建会从 `MenthaAR/src/main/cpp/Thirdparty/opencv` 子模块源码编译 OpenCV，首次构建非常慢。
- 克隆仓库必须 `--recursive`；若 OpenCV 子模块缺失，原生构建会失败。
- 本机 `local.properties` 与 `keystore.properties` 存在但被 gitignore，不要提交。
- 目前没有项目自己的单元/仪器测试；修改后以 `assembleDebug` 通过作为最低验证。

## 核心架构与数据流

### 进程与 IPC

- `app` 进程中的 `SlamIPCClient` 绑定 `MenthaAR` 模块的 `SlamService`。
- `SlamService` 运行在独立进程 `:slam_process`，内部只有一个串行 `SlamFrameHandler` 线程处理所有 SLAM 任务。
- 高频路径使用 `oneway` AIDL：`processFrame` 只投递 seq/bufIndex/尺寸，不复制图像数据。
- 图像与结果通过 `SharedMemoryBuffer` 跨进程共享：Java 创建 `SharedMemory`/`MemoryFile`，将 fd 传给 native 层 `mmap`。
- 渲染线程直接读共享内存中的 MVP、drawFlag、点云，避免每帧 Binder 调用。

### 帧路径

1. `CameraPreviewView`（CameraX RGBA）将帧转成灰度 Y。
2. `DeviceCompat` 对 Rokid 眼镜等特殊设备做镜像翻转。
3. `SlamIPCClient.sendFrameData()` 将 Y 写入共享内存双缓冲，并通过 `oneway processFrame` 投递。
4. `SlamService` 将任务放入串行队列，native `processFrameSharedMem` 调用 `processImage()`。
5. `processImage()` 执行 ORB 特征、跟踪、局部建图、重定位/地图对齐、AR 锚点更新。
6. native 将 tracking/draw/MVP/点云/slamDoneSeq 写回共享内存。
7. UI 的 GL/Filament 渲染器读取共享内存并绘制。

### 共享内存布局（关键同步点）

`SharedMemoryBuffer.java` 与 `native-lib.cpp` 中的 `SH_*` 常量必须保持一致：

- Header 256 字节，magic `"MNTH"`，版本 2，little-endian。
- `frameW/frameH`、`uiWriteSeq`、`slamDoneSeq`、`trackingState`、`drawFlag`、`pointCloudBytes`、`MVP[48]`。
- 之后是两个 Y 帧缓冲（双缓冲）和一个点云区（`96 * 1024` 字节上限，每点 7 个 float）。
- UI 写 `buf[seq%2]` 前必须满足 `slamDoneSeq >= seq-2`，防止覆盖 SLAM 正在处理的缓冲。

修改共享内存布局、AIDL 方法或序列号协议时，必须同时修改 Java 与 native 两侧，并在同一提交中完成。

### Native 并发注意事项

- 不要绕过 `SlamService` 的串行队列直接在 Binder 线程执行耗时 SLAM 操作。
- 锁顺序为：`gSlamPtrLock -> gMapDataMutex -> gMapPointsMutex`，不得反向嵌套。
- 写操作（`loadMapWithId`、`nativeShutdown`）需等待 `gProcessingFrames == 0` 再访问/删除 `slamSys`。
- 新增 native 全局状态时，优先使用 `std::atomic` 或明确互斥锁，避免 SLAM 线程与 UI/Binder 线程竞争。

## 代码风格与约定

### 通用

- 保留每个文件顶部的许可证头；新增文件必须按所在模块选择正确许可头。
- 注释目前以中文为主，可继续使用中文；不要因为“翻译”而大规模改动已有注释。
- 避免无关的大规模格式化/重排，保持 diff 可审查。

### Java

- 包名统一 `com.orb.slam2s.*`。
- 现有 UI/渲染类使用 `m` 前缀字段（如 `mCameraPreviewView`）；沿用该风格。
- 使用 `private static final String TAG = "ClassName";` + `Log.d/e/w` 输出日志。
- 用户可见文案放入 `app/src/main/res/values*/strings.xml`，不要硬编码 UI 字符串；新增文案至少同步默认 `values/strings.xml`。
- 当前 UI 是 XML View + GLSurfaceView，默认不要迁移到 Jetpack Compose，除非用户明确要求。

### C++

- 核心 SLAM 类位于 `namespace ORB_SLAM2`。
- 实现文件使用 `.cc`，头文件在 `include/`；JNI/AR/IPC 扩展在 `cpp/*.cpp` 与 `native-lib.cpp`。
- 调参常量集中到 `Config.h`，不要散落魔法数字。
- 保持 C++11 兼容；允许 OpenMP/NEON/`-O3`/LTO，但避免引入需要新工具链的特性。
- 热路径中注意避免不必要的大对象拷贝、频繁 `std::vector` 分配和锁竞争。

### 渲染与坐标

- SLAM 使用 RDF（右-下-前），OpenGL/Filament 使用 RUB（右-上-后）；两者矩阵转换在 native 和 Java 中都有对应实现。
- 点云格式：每点 7 个 float `[x, y, z, r, g, b, size]`。
- 修改投影矩阵、视图矩阵或模型矩阵时，需要同步验证 native `writeResultToSharedMemory`、`GLPointCloudRenderer` 和 `FilamentModelRenderer` 的坐标约定。

## 不建议做的事

- 不要直接编辑 `Thirdparty/opencv` 生成物或大量修改其源码；OpenCV 是 submodule，应通过子模块方式管理。
- 不要提交 `build/`、`.gradle/`、`local.properties`、`keystore.properties`、`build_vs/`、`.claude/` 等被 gitignore 的目录/文件。
- 不要将 `MenthaAR/src/main/cpp/ubuntu/`、`windows/` 下的构建产物当作 Android 源码维护。
- 不要启用 lint 任务（项目有意关闭）。
- 不要为“优化”而破坏双缓冲背压协议或串行任务队列。
- 不要跨模块混用许可证：App 层保持 Apache-2.0，SLAM 引擎保持 GPL/双许可。

## 可用技能与参考

- 如果环境配置了 `android-skills` MCP，在涉及 CameraX、AGP、edge-to-edge、性能分析等 Android 专项任务时可检索对应 skill。
- 项目文档：`README.md` 及 `docs/README_*.md`。
- 贡献流程：`.github/CONTRIBUTING*.md`。
