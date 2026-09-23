cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_VERBOSE_MAKEFILE on)

# Find OpenMP to enable multi-threading acceleration
project(MenthaAR C CXX)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP CXX found!")
else()
    message(WARNING "OpenMP CXX NOT found! OpenCV will compile without OpenMP acceleration.")
endif()

# OpenCV 构建
# 使用官方 OpenCV 源码库，作为子库构建，并激活 NEON/OpenMP 硬件加速
set(OPENCV_SRC_DIR "${CMAKE_SOURCE_DIR}/src/main/cpp/Thirdparty/opencv")

# 强制开启 C++ 编译器全局优化参数 (O3) 和 LTO (Link Time Optimization)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -flto=thin" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -flto=thin" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG -flto=thin" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -flto=thin" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_DEBUG "-O2 -g" CACHE STRING "" FORCE)

set(ARCH_SPECIFIC_OPTIMIZATION_FLAGS "")

if(ANDROID_ABI STREQUAL "arm64-v8a" OR CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)")
    # ARM64: 保持 Android 标准 arm64-v8a 基线兼容性 (ARMv8.0-A)，兼容所有大小核架构
    list(APPEND ARCH_SPECIFIC_OPTIMIZATION_FLAGS "-march=armv8-a")
    message(STATUS "[MenthaAR] 针对目标架构 ARM64 (arm64-v8a) 启用基础优化: ${ARCH_SPECIFIC_OPTIMIZATION_FLAGS}")

elseif(ANDROID_ABI STREQUAL "armeabi-v7a" OR CMAKE_SYSTEM_PROCESSOR MATCHES "^(armv7|arm)")
    # ARMv7-A: 启用 NEON-VFPv4 浮点向量计算单元与 softfp 浮点调用约定
    list(APPEND ARCH_SPECIFIC_OPTIMIZATION_FLAGS "-mfpu=neon-vfpv4" "-mfloat-abi=softfp")
    message(STATUS "[MenthaAR] 针对目标架构 ARMv7-A (armeabi-v7a) 启用专属优化: ${ARCH_SPECIFIC_OPTIMIZATION_FLAGS}")

elseif(ANDROID_ABI STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)")
    # x86_64: 启用 x86-64-v2 微架构基线（包含 SSE3/SSSE3/SSE4.1/SSE4.2/POPCNT 等），
    # 彻底激活 ORB 描述子汉明距离硬件 popcount 指令 (__builtin_popcountll -> POPCNT 单指令) 及 Eigen/OpenCV 向量化
    list(APPEND ARCH_SPECIFIC_OPTIMIZATION_FLAGS "-march=x86-64-v2" "-mssse3" "-msse4.1" "-msse4.2" "-mpopcnt")
    set(CPU_BASELINE "SSE4_2;POPCNT" CACHE STRING "OpenCV baseline optimizations" FORCE)
    message(STATUS "[MenthaAR] 针对目标架构 x86_64 启用专属优化: ${ARCH_SPECIFIC_OPTIMIZATION_FLAGS}")

elseif(ANDROID_ABI STREQUAL "x86" OR CMAKE_SYSTEM_PROCESSOR MATCHES "^(i[3-6]86|x86)")
    # x86 32位: 启用 SSSE3/SSE4.1/SSE4.2 与硬件 POPCNT 指令加速，强制使用 SSE 浮点计算单元
    list(APPEND ARCH_SPECIFIC_OPTIMIZATION_FLAGS "-mssse3" "-msse4.1" "-msse4.2" "-mpopcnt" "-mfpmath=sse")
    message(STATUS "[MenthaAR] 针对目标架构 x86 (32-bit) 启用专属优化: ${ARCH_SPECIFIC_OPTIMIZATION_FLAGS}")

elseif(ANDROID_ABI STREQUAL "riscv64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "^(riscv64|riscv)")
    # RISC-V 64位: 启用 RV64GCV（RVV 1.0 矢量 SIMD 扩展）与位操作扩展（Zba/Zbb/Zbs），
    # 其中 Zbb 扩展原生提供 cpop 硬件指令，将 ORB 汉明距离从数十周期软件模拟优化为单周期硬件执行
    list(APPEND ARCH_SPECIFIC_OPTIMIZATION_FLAGS "-march=rv64gcv_zba_zbb_zbs")
    set(ENABLE_RVV ON CACHE BOOL "Enable RVV for RISC-V" FORCE)
    message(STATUS "[MenthaAR] 针对目标架构 RISC-V 64 (riscv64) 启用专属优化: ${ARCH_SPECIFIC_OPTIMIZATION_FLAGS}")

else()
    message(STATUS "[MenthaAR] 当前架构 (ABI: ${ANDROID_ABI}, ARCH: ${CMAKE_SYSTEM_PROCESSOR}) 使用默认优化配置")
endif()

# 强制设置编译参数以排除非必要模块与测试，缩短编译时间
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)
set(BUILD_LIST "core,imgproc,features2d,flann,calib3d" CACHE STRING "List of modules to build" FORCE)
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
set(WITH_TBB OFF CACHE BOOL "With TBB" FORCE)
set(WITH_OPENMP OFF CACHE BOOL "With OpenMP" FORCE)
set(WITH_EIGEN ON CACHE BOOL "With Eigen" FORCE)
set(WITH_PNG OFF CACHE BOOL "With PNG" FORCE)
set(WITH_JPEG OFF CACHE BOOL "With JPEG" FORCE)
set(WITH_TIFF OFF CACHE BOOL "With TIFF" FORCE)
set(WITH_WEBP OFF CACHE BOOL "With WEBP" FORCE)
set(WITH_JASPER OFF CACHE BOOL "With Jasper" FORCE)
set(WITH_OPENEXR OFF CACHE BOOL "With OpenEXR" FORCE)
set(WITH_ITT OFF CACHE BOOL "With ITT" FORCE)
set(WITH_PROTOBUF OFF CACHE BOOL "With Protobuf" FORCE)
set(WITH_QUIRC OFF CACHE BOOL "With Quirc" FORCE)
set(WITH_CAROTENE ON CACHE BOOL "With Carotene" FORCE)

include_directories(${OPENCV_SRC_DIR}/include)
include_directories(${OPENCV_SRC_DIR}/modules/core/include)
include_directories(${OPENCV_SRC_DIR}/modules/imgproc/include)
include_directories(${OPENCV_SRC_DIR}/modules/features2d/include)
include_directories(${OPENCV_SRC_DIR}/modules/flann/include)
include_directories(${OPENCV_SRC_DIR}/modules/calib3d/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory(${OPENCV_SRC_DIR} opencv_build)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fexceptions -frtti")

ADD_LIBRARY(g2o
        src/main/cpp/Thirdparty/g2o/g2o/types/types_sba.h
        src/main/cpp/Thirdparty/g2o/g2o/types/types_six_dof_expmap.h
        src/main/cpp/Thirdparty/g2o/g2o/types/types_sba.cpp
        src/main/cpp/Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp
        src/main/cpp/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp
        src/main/cpp/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h
        src/main/cpp/Thirdparty/g2o/g2o/types/se3quat.h
        src/main/cpp/Thirdparty/g2o/g2o/types/se3_ops.h
        src/main/cpp/Thirdparty/g2o/g2o/types/se3_ops.hpp
        src/main/cpp/Thirdparty/g2o/g2o/core/base_edge.h
        src/main/cpp/Thirdparty/g2o/g2o/core/base_binary_edge.h
        src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph_action.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/base_binary_edge.hpp
        src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph_action.h
        src/main/cpp/Thirdparty/g2o/g2o/core/base_multi_edge.h
        src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/base_multi_edge.hpp
        src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph.h
        src/main/cpp/Thirdparty/g2o/g2o/core/base_unary_edge.h
        src/main/cpp/Thirdparty/g2o/g2o/core/linear_solver.h
        src/main/cpp/Thirdparty/g2o/g2o/core/base_unary_edge.hpp
        src/main/cpp/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/base_vertex.h
        src/main/cpp/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.h
        src/main/cpp/Thirdparty/g2o/g2o/core/base_vertex.hpp
        src/main/cpp/Thirdparty/g2o/g2o/core/matrix_structure.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/batch_stats.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/matrix_structure.h
        src/main/cpp/Thirdparty/g2o/g2o/core/batch_stats.h
        src/main/cpp/Thirdparty/g2o/g2o/core/openmp_mutex.h
        src/main/cpp/Thirdparty/g2o/g2o/core/block_solver.h
        src/main/cpp/Thirdparty/g2o/g2o/core/block_solver.hpp
        src/main/cpp/Thirdparty/g2o/g2o/core/parameter.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/parameter.h
        src/main/cpp/Thirdparty/g2o/g2o/core/cache.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/cache.h
        src/main/cpp/Thirdparty/g2o/g2o/core/optimizable_graph.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/optimizable_graph.h
        src/main/cpp/Thirdparty/g2o/g2o/core/solver.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/solver.h
        src/main/cpp/Thirdparty/g2o/g2o/core/creators.h
        src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/estimate_propagator.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_factory.h
        src/main/cpp/Thirdparty/g2o/g2o/core/estimate_propagator.h
        src/main/cpp/Thirdparty/g2o/g2o/core/factory.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_property.h
        src/main/cpp/Thirdparty/g2o/g2o/core/factory.h
        src/main/cpp/Thirdparty/g2o/g2o/core/sparse_block_matrix.h
        src/main/cpp/Thirdparty/g2o/g2o/core/sparse_optimizer.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp
        src/main/cpp/Thirdparty/g2o/g2o/core/sparse_optimizer.h
        src/main/cpp/Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/hyper_dijkstra.h
        src/main/cpp/Thirdparty/g2o/g2o/core/parameter_container.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/parameter_container.h
        src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm.h
        src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h
        src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h
        src/main/cpp/Thirdparty/g2o/g2o/core/jacobian_workspace.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/jacobian_workspace.h
        src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel.h
        src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_factory.h
        src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp
        src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_impl.h
        src/main/cpp/Thirdparty/g2o/g2o/stuff/string_tools.h
        src/main/cpp/Thirdparty/g2o/g2o/stuff/color_macros.h
        src/main/cpp/Thirdparty/g2o/g2o/stuff/macros.h
        src/main/cpp/Thirdparty/g2o/g2o/stuff/timeutil.cpp
        src/main/cpp/Thirdparty/g2o/g2o/stuff/misc.h
        src/main/cpp/Thirdparty/g2o/g2o/stuff/timeutil.h
        src/main/cpp/Thirdparty/g2o/g2o/stuff/os_specific.c
        src/main/cpp/Thirdparty/g2o/g2o/stuff/os_specific.h
        src/main/cpp/Thirdparty/g2o/g2o/stuff/string_tools.cpp
        src/main/cpp/Thirdparty/g2o/g2o/stuff/property.cpp
        src/main/cpp/Thirdparty/g2o/g2o/stuff/property.h
        )

include_directories(
        src/main/cpp
        src/main/cpp/include
        src/main/cpp/Thirdparty
        src/main/cpp/Thirdparty/fast
        src/main/cpp/Thirdparty/g2o
        src/main/cpp/Thirdparty/srrg_hbst
)

target_compile_definitions(g2o PRIVATE
        EIGEN_DONT_ALIGN_STATICALLY
        EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
        EIGEN_MAX_ALIGN_BYTES=16)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(g2o PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-O3 -g -fno-omit-frame-pointer -fno-strict-aliasing -flto=thin>
            $<$<CONFIG:Debug>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing>)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(g2o PUBLIC OpenMP::OpenMP_CXX)
    endif()
    target_link_options(g2o PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-flto=thin -O3>)
else()
    target_compile_options(g2o PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-O3 -g -fno-omit-frame-pointer -fno-strict-aliasing -flto>
            $<$<CONFIG:Debug>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing>)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(g2o PUBLIC OpenMP::OpenMP_CXX)
    endif()
    target_link_options(g2o PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-flto -O3>)
endif()

# 应用架构专属优化参数到 g2o (在 Release 和 RelWithDebInfo 下生效)
if(ARCH_SPECIFIC_OPTIMIZATION_FLAGS)
    foreach(opt_flag IN LISTS ARCH_SPECIFIC_OPTIMIZATION_FLAGS)
        target_compile_options(g2o PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:${opt_flag}>)
    endforeach()
endif()

# 整合所有组件到一个 SO：libMenthaAR_Engine.so
add_library(MenthaAR_Engine SHARED
        # SLAM Core 组件
        src/main/cpp/include/System.h
        src/main/cpp/include/Tracking.h
        src/main/cpp/include/LocalMapping.h
        src/main/cpp/include/LoopClosing.h
        src/main/cpp/include/ORBextractor.h
        src/main/cpp/include/ORBmatcher.h
        src/main/cpp/include/FrameDrawer.h
        src/main/cpp/include/Converter.h
        src/main/cpp/include/MapPoint.h
        src/main/cpp/include/KeyFrame.h
        src/main/cpp/include/Map.h
        src/main/cpp/include/Optimizer.h
        src/main/cpp/include/PnPsolver.h
        src/main/cpp/include/Frame.h
        src/main/cpp/include/KeyFrameDatabase.h
        src/main/cpp/include/Sim3Solver.h
        src/main/cpp/include/Initializer.h
        src/main/cpp/include/Common.h
        src/main/cpp/src/System.cc
        src/main/cpp/src/Tracking.cc
        src/main/cpp/src/LocalMapping.cc
        src/main/cpp/src/LoopClosing.cc
        src/main/cpp/src/ORBextractor.cc
        src/main/cpp/src/ORBmatcher.cc
        src/main/cpp/src/FrameDrawer.cc
        src/main/cpp/src/Converter.cc
        src/main/cpp/src/MapPoint.cc
        src/main/cpp/src/KeyFrame.cc
        src/main/cpp/src/Map.cc
        src/main/cpp/src/Optimizer.cc
        src/main/cpp/src/PnPsolver.cc
        src/main/cpp/src/Frame.cc
        src/main/cpp/src/KeyFrameDatabase.cc
        src/main/cpp/src/Sim3Solver.cc
        src/main/cpp/src/Initializer.cc

        # JNI 接口层与 IPC 通讯层
        src/main/cpp/native-lib.cpp
        src/main/cpp/UIUtils.cpp
        src/main/cpp/Plane.cpp
        src/main/cpp/Matrix.cpp

        # FAST 角点检测组件 (Edward Rosten C FAST-9)
        src/main/cpp/Thirdparty/fast/fast.h
        src/main/cpp/Thirdparty/fast/fast_9.c
        src/main/cpp/Thirdparty/fast/nonmax.c
        src/main/cpp/Thirdparty/fast/fast.c
        )

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(MenthaAR_Engine PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-O3 -g -fno-omit-frame-pointer -fno-strict-aliasing -flto=thin>
            $<$<CONFIG:Debug>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing>)
else()
    target_compile_options(MenthaAR_Engine PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-O3 -g -fno-omit-frame-pointer -fno-strict-aliasing -flto>
            $<$<CONFIG:Debug>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing>)
endif()

# LTO 链接标志（仅 Release 和 RelWithDebInfo）
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_link_options(MenthaAR_Engine PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-flto=thin -O3>)
else()
    target_link_options(MenthaAR_Engine PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-flto -O3>)
endif()

# 应用架构专属优化参数到 MenthaAR_Engine (在 Release 和 RelWithDebInfo 下生效)
if(ARCH_SPECIFIC_OPTIMIZATION_FLAGS)
    foreach(opt_flag IN LISTS ARCH_SPECIFIC_OPTIMIZATION_FLAGS)
        target_compile_options(MenthaAR_Engine PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:${opt_flag}>)
    endforeach()
endif()

find_library(log-lib log)

target_link_libraries(MenthaAR_Engine
        -Wl,--whole-archive
        opencv_core
        opencv_imgproc
        opencv_features2d
        opencv_flann
        opencv_calib3d
        -Wl,--no-whole-archive
        g2o
        ${log-lib}
        jnigraphics                     # For AndroidBitmap_* APIs (bitmap.h)
        EGL
        GLESv1_CM
        )

if(OpenMP_CXX_FOUND)
    target_link_libraries(MenthaAR_Engine OpenMP::OpenMP_CXX -static-openmp)
endif()

# Mentha 性能分析器
set(MENTHA_DEVELOP_MODE ON) # <--- 修改这里：ON 开启，OFF 关闭

message(STATUS "Mentha 分析器状态: ${MENTHA_DEVELOP_MODE}")

# 始终包含此目录，以便编译器在 OFF 模式下也能找到头文件并展开空宏
include_directories(src/main/cpp/profiler)

if(MENTHA_DEVELOP_MODE)
    # 仅在开启时添加分析器库
    add_library(Mentha_Profiler SHARED
            src/main/cpp/profiler/MenthaProfiler.cpp
            src/main/cpp/profiler/MenthaProfiler.h
    )

    # 使用 PUBLIC 确保宏定义传播给所有链接到此库的目标（如 MenthaAR_Engine）
    target_compile_definitions(Mentha_Profiler PUBLIC MENTHA_DEVELOP_MODE)

    # 应用架构专属优化参数
    if(ARCH_SPECIFIC_OPTIMIZATION_FLAGS)
        foreach(opt_flag IN LISTS ARCH_SPECIFIC_OPTIMIZATION_FLAGS)
            target_compile_options(Mentha_Profiler PRIVATE
                $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:${opt_flag}>)
        endforeach()
    endif()

    target_link_libraries(Mentha_Profiler ${log-lib} -static-openmp)

    if(OpenMP_CXX_FOUND)
        target_link_libraries(Mentha_Profiler OpenMP::OpenMP_CXX -static-openmp)
    endif()

    # 链接分析器库
    target_link_libraries(MenthaAR_Engine Mentha_Profiler)
endif()