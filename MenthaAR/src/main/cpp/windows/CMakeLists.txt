cmake_minimum_required(VERSION 3.10)
project(MenthaAR_Windows CXX C)
 
# 设置 C++ 标准为 C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------------------------------------------------------
# 1. 动态获取项目仓库的根目录绝对路径
# ----------------------------------------------------------------------------
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.." ABSOLUTE)
message(STATUS "[CMake] 检测到的项目根目录路径: ${REPO_ROOT}")

# ----------------------------------------------------------------------------
# 2. 内置 OpenCV 源码集成配置 (使用 Thirdparty/opencv 源码构建静态库)
# ----------------------------------------------------------------------------
find_package(Threads REQUIRED)

set(OPENCV_SRC_DIR "${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/opencv")
message(STATUS "[CMake] 使用内置 OpenCV 源码目录: ${OPENCV_SRC_DIR}")

# 强制配置 OpenCV 编译参数，仅构建 SLAM 所需的核心模块 (core, imgproc, features2d, flann, calib3d)
# Ubuntu 桌面版本额外需要 videoio, imgcodecs, highgui 模块用于相机/视频捕获和 GUI 显示
# Windows 平台由 videoio 内置 MSMF 后端与 highgui 内置 Win32 UI 提供同等能力，无需 GTK/FFmpeg
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)
set(BUILD_LIST "core,imgproc,features2d,flann,calib3d,videoio,imgcodecs,highgui" CACHE STRING "List of modules to build" FORCE)
set(BUILD_opencv_apps OFF CACHE BOOL "Build apps" FORCE)
set(BUILD_opencv_js OFF CACHE BOOL "Build js" FORCE)
set(BUILD_ANDROID_PROJECTS OFF CACHE BOOL "Build Android projects" FORCE)
set(BUILD_ANDROID_EXAMPLES OFF CACHE BOOL "Build Android examples" FORCE)
set(BUILD_DOCS OFF CACHE BOOL "Build docs" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build examples" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "Build tests" FORCE)
set(BUILD_PERF_TESTS OFF CACHE BOOL "Build perf tests" FORCE)
set(BUILD_JAVA OFF CACHE BOOL "Build Java wrapper" FORCE)
set(BUILD_FAT_JAVA_LIB OFF CACHE BOOL "Build fat Java library" FORCE)
set(BUILD_KOTLIN OFF CACHE BOOL "Build Kotlin wrapper" FORCE)
set(BUILD_OBJC OFF CACHE BOOL "Build ObjC wrapper" FORCE)
set(OPENCV_ENABLE_NONFREE OFF CACHE BOOL "Enable non-free" FORCE)
set(WITH_CUDA OFF CACHE BOOL "With CUDA" FORCE)
set(WITH_OPENCL OFF CACHE BOOL "With OpenCL" FORCE)
set(WITH_IPP OFF CACHE BOOL "With IPP" FORCE)
if(WIN32)
    # Windows 下关闭 TBB 探测（避免触发第三方依赖解析），OpenCV 使用内置并行框架
    set(WITH_TBB OFF CACHE BOOL "With TBB" FORCE)
else()
    set(WITH_TBB ON CACHE BOOL "With TBB" FORCE)
endif()
# 注意：OpenCV 不启用 Eigen —— 工程内置的是旧版扁平布局 Eigen，且 Windows 文件系统
# 大小写不敏感会误触发 Eigen/Array 兼容报错；SLAM 引擎自身直接使用该 Eigen，互不影响
set(WITH_EIGEN OFF CACHE BOOL "With Eigen" FORCE)
set(WITH_PNG ON CACHE BOOL "With PNG" FORCE)
set(WITH_JPEG ON CACHE BOOL "With JPEG" FORCE)
set(WITH_TIFF OFF CACHE BOOL "With TIFF" FORCE)
set(WITH_WEBP OFF CACHE BOOL "With WEBP" FORCE)
set(WITH_JASPER OFF CACHE BOOL "With Jasper" FORCE)
set(WITH_OPENEXR OFF CACHE BOOL "With OpenEXR" FORCE)
set(WITH_ITT OFF CACHE BOOL "With ITT" FORCE)
set(WITH_PROTOBUF OFF CACHE BOOL "With Protobuf" FORCE)
set(WITH_QUIRC OFF CACHE BOOL "With Quirc" FORCE)
if(WIN32)
    # Windows 下无需 JPEG2000/多线程加速依赖，进一步精简构建
    set(WITH_OPENJPEG OFF CACHE BOOL "With OpenJPEG" FORCE)
endif()

add_subdirectory(${OPENCV_SRC_DIR} opencv_build)

# ----------------------------------------------------------------------------
# 3. 编译器优化选项与编译定义 (Windows/VS2022 与 Linux/GCC 双平台支持)
# ----------------------------------------------------------------------------
add_compile_definitions(EIGEN_DONT_ALIGN_STATICALLY EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT EIGEN_MAX_ALIGN_BYTES=16)

if(MSVC)
    # VS2022 (cl.exe)：等价于 GCC 分支的 O3/native/LTO 组合
    # /MP 多核并行编译；/Zc:__cplusplus 修正 __cplusplus 宏；/utf-8 统一源码编码（工程内源码均为 UTF-8）
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /Zc:__cplusplus /utf-8")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oi")
    # 抑制 Windows 头文件 min/max 宏污染与 CRT 安全告警
    add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS _USE_MATH_DEFINES)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -flto -Wall -pthread")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto -s")
endif()

# 可执行文件统一输出到 build/bin 目录（多配置生成器会自动追加 Debug/Release 子目录）
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

# 头文件包含路径列表 (全部使用根目录绝对路径定位，确保无缝解析)
include_directories(
    ${OPENCV_SRC_DIR}/include
    ${OPENCV_SRC_DIR}/modules/core/include
    ${OPENCV_SRC_DIR}/modules/imgproc/include
    ${OPENCV_SRC_DIR}/modules/features2d/include
    ${OPENCV_SRC_DIR}/modules/flann/include
    ${OPENCV_SRC_DIR}/modules/calib3d/include
    ${OPENCV_SRC_DIR}/modules/videoio/include
    ${OPENCV_SRC_DIR}/modules/imgcodecs/include
    ${OPENCV_SRC_DIR}/modules/highgui/include
    ${CMAKE_CURRENT_BINARY_DIR}/opencv_build
    ${CMAKE_CURRENT_BINARY_DIR}
    ${REPO_ROOT}/MenthaAR/src/main/cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/srrg_hbst
    ${REPO_ROOT}/MenthaAR/src/main/cpp/profiler
)

# ----------------------------------------------------------------------------
# 4. 编译第三方优化的 g2o 静态库
# ----------------------------------------------------------------------------
add_library(g2o STATIC
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_sba.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_six_dof_expmap.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_sba.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/types/se3quat.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/types/se3_ops.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/types/se3_ops.hpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_edge.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_binary_edge.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph_action.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_binary_edge.hpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph_action.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_multi_edge.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_multi_edge.hpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_unary_edge.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/linear_solver.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_unary_edge.hpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_vertex.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_vertex.hpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/matrix_structure.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/batch_stats.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/matrix_structure.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/batch_stats.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/openmp_mutex.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/block_solver.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/block_solver.hpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/parameter.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/parameter.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/cache.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/cache.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimizable_graph.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimizable_graph.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/solver.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/solver.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/creators.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/estimate_propagator.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_factory.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/estimate_propagator.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/factory.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_property.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/factory.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_block_matrix.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_optimizer.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_optimizer.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_dijkstra.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/parameter_container.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/parameter_container.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/jacobian_workspace.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/jacobian_workspace.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_factory.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_impl.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/string_tools.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/color_macros.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/macros.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/timeutil.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/misc.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/timeutil.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/os_specific.c
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/os_specific.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/string_tools.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/property.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/property.h
)

target_compile_definitions(g2o PRIVATE
    EIGEN_DONT_ALIGN_STATICALLY
    EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
    EIGEN_MAX_ALIGN_BYTES=16
)

if(MSVC)
    # 激活 g2o 内置的 Windows 兼容分支 (os_specific/timeutil 提供 gettimeofday、vasprintf 等)
    # PUBLIC 传播给链接方，保证包含 g2o 头文件的 TU 使用一致的分支
    target_compile_definitions(g2o PUBLIC WINDOWS _WINDOWS)
endif()

# ----------------------------------------------------------------------------
# 5. 构建 SLAM AR 核心底层基础库 (MenthaAR_Engine)
# ----------------------------------------------------------------------------
add_library(MenthaAR_Engine STATIC
    # SLAM 核心组件
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/System.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/Tracking.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/LocalMapping.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/LoopClosing.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/ORBextractor.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/ORBmatcher.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/FrameDrawer.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/Converter.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/MapPoint.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/KeyFrame.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/Map.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/Optimizer.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/PnPsolver.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/Frame.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/KeyFrameDatabase.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/Sim3Solver.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/Initializer.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/include/Common.h
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/System.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/Tracking.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/LocalMapping.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/LoopClosing.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/ORBextractor.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/ORBmatcher.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/FrameDrawer.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/Converter.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/MapPoint.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/KeyFrame.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/Map.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/Optimizer.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/PnPsolver.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/Frame.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/KeyFrameDatabase.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/Sim3Solver.cc
    ${REPO_ROOT}/MenthaAR/src/main/cpp/src/Initializer.cc
    
    # DBoW2 组件已移除，改用 HBST
    
    # 底层通用实用组件
    ${REPO_ROOT}/MenthaAR/src/main/cpp/UIUtils.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Plane.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/Matrix.cpp
    ${REPO_ROOT}/MenthaAR/src/main/cpp/profiler/MenthaProfiler.cpp
)

# ----------------------------------------------------------------------------
# 6. 构建桌面可执行文件 (MenthaAR_Ubuntu)
# ----------------------------------------------------------------------------
add_executable(MenthaAR_Windows ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(MenthaAR_Windows
    MenthaAR_Engine
    g2o
    opencv_core
    opencv_imgproc
    opencv_features2d
    opencv_flann
    opencv_calib3d
    opencv_videoio
    opencv_imgcodecs
    opencv_highgui
    Threads::Threads
)

# ----------------------------------------------------------------------------
# 7. 构建 Benchmark 工具 (MenthaAR_Benchmark)
# ----------------------------------------------------------------------------
add_executable(MenthaAR_Benchmark ${CMAKE_CURRENT_SOURCE_DIR}/benchmark.cpp)

target_link_libraries(MenthaAR_Benchmark
    MenthaAR_Engine
    g2o
    opencv_core
    opencv_imgproc
    opencv_features2d
    opencv_flann
    opencv_calib3d
    opencv_videoio
    opencv_imgcodecs
    opencv_highgui
    Threads::Threads
)

message(STATUS "[CMake] 构建文件生成完毕。使用 'cmake --build . --config Release' 构建 MenthaAR_Windows 或 MenthaAR_Benchmark。")