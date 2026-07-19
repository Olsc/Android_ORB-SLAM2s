cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_VERBOSE_MAKEFILE on)
set(libs "${CMAKE_SOURCE_DIR}/src/main/jniLibs")

# Find OpenMP to enable multi-threading acceleration
project(VtonaxAR C CXX)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP CXX found!")
else()
    message(WARNING "OpenMP CXX NOT found! OpenCV will compile without OpenMP acceleration.")
endif()

#--------------------------------------------------- OpenCV 构建 ---------------------------------------------------#
# 使用 TEMP/opencv 的纯 C/C++ 源码构建，替代旧的 libopencv_java4.so + OpenCVLibrary
set(OPENCV_SRC_DIR "${CMAKE_SOURCE_DIR}/src/main/cpp/opencv_src")
include_directories(${OPENCV_SRC_DIR}/include)
include_directories(${OPENCV_SRC_DIR}/modules/core/include)
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

# ARM64 架构特定优化
target_compile_options(g2o PRIVATE
    $<$<AND:$<CONFIG:Release>,$<STREQUAL:${ANDROID_ABI},arm64-v8a>>:-march=armv8.2-a+fp16+rcpc+dotprod>)

include_directories(
        src/main/cpp
        src/main/cpp/include
        src/main/cpp/Thirdparty
        src/main/cpp/Thirdparty/g2o
        src/main/cpp/Thirdparty/srrg_hbst
        src/main/ncnn-android-vulkan-shared/${ANDROID_ABI}/include
)

#------------------------------------------------ 嵌入资源 ------------------------------------------------#
# 使用汇编 .incbin 指令将二进制文件直接嵌入到目标文件（比生成C数组快得多）
set(RESOURCE_DIR ${CMAKE_SOURCE_DIR}/src/main/other)
set(EMBEDDED_DIR ${CMAKE_CURRENT_BINARY_DIR}/embedded)
file(MAKE_DIRECTORY ${EMBEDDED_DIR})

# 函数：将二进制文件转换为目标文件
# 使用汇编包装器来创建明确命名的符号
function(embed_resource input_file symbol_base output_obj)
    set(bin_file ${EMBEDDED_DIR}/${symbol_base}.bin)
    set(asm_file ${EMBEDDED_DIR}/${symbol_base}.S)
    
    # 步骤1: 复制二进制文件
    add_custom_command(
        OUTPUT ${bin_file}
        COMMAND ${CMAKE_COMMAND} -E copy ${input_file} ${bin_file}
        DEPENDS ${input_file}
        COMMENT "Copying ${input_file}"
        VERBATIM
    )
    
    # 步骤2: 生成汇编包装器文件
    add_custom_command(
        OUTPUT ${asm_file}
        COMMAND ${CMAKE_COMMAND} -E echo ".section .rodata" > ${asm_file}
        COMMAND ${CMAKE_COMMAND} -E echo ".global _binary_${symbol_base}_start" >> ${asm_file}
        COMMAND ${CMAKE_COMMAND} -E echo ".global _binary_${symbol_base}_end" >> ${asm_file}
        COMMAND ${CMAKE_COMMAND} -E echo ".balign 16" >> ${asm_file}
        COMMAND ${CMAKE_COMMAND} -E echo "_binary_${symbol_base}_start:" >> ${asm_file}
        COMMAND ${CMAKE_COMMAND} -E echo ".incbin \"${bin_file}\"" >> ${asm_file}
        COMMAND ${CMAKE_COMMAND} -E echo "_binary_${symbol_base}_end:" >> ${asm_file}
        DEPENDS ${bin_file}
        COMMENT "Generating assembly wrapper for ${symbol_base}"
        VERBATIM
    )
    
    # 步骤3: 汇编生成目标文件 (使用正确的Android目标架构)
    add_custom_command(
        OUTPUT ${output_obj}
        COMMAND ${CMAKE_C_COMPILER}
            --target=${CMAKE_C_COMPILER_TARGET}
            --sysroot=${CMAKE_SYSROOT}
            -c ${asm_file} -o ${output_obj}
        DEPENDS ${asm_file}
        COMMENT "Assembling ${symbol_base} (fast method)"
        VERBATIM
    )
endfunction()

# 嵌入资源文件
set(EMBEDDED_OBJS)

# 创建自定义目标以确保在链接前生成所有对象文件
add_custom_target(generate_embedded_resources DEPENDS "${EMBEDDED_OBJS}")


# ========== 整合所有组件到一个 SO：libVtonaxAR_Engine.so ==========
add_library(VtonaxAR_Engine SHARED
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
        

        
        # JNI 接口层
        src/main/cpp/native-lib.cpp
        src/main/cpp/UIUtils.cpp
        src/main/cpp/Plane.cpp
        src/main/cpp/Matrix.cpp
        src/main/cpp/bridge/OpenCVBridge.cpp
        src/main/cpp/bridge/opencv_stubs.cpp


        # 6DoF 传感融合功能已移除
        
        # 嵌入资源
        ${EMBEDDED_OBJS}
        )

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(VtonaxAR_Engine PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-O3 -g -fno-omit-frame-pointer -fno-strict-aliasing -flto=thin>
            $<$<CONFIG:Debug>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing>)
else()
    target_compile_options(VtonaxAR_Engine PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-O3 -g -fno-omit-frame-pointer -fno-strict-aliasing -flto>
            $<$<CONFIG:Debug>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing>)
endif()

# LTO 链接标志（仅 Release 和 RelWithDebInfo）
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_link_options(VtonaxAR_Engine PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-flto=thin -O3>)
else()
    target_link_options(VtonaxAR_Engine PRIVATE
            $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>>:-flto -O3>)
endif()

find_library(log-lib log)

# 确保在编译前生成嵌入的资源对象文件
add_dependencies(VtonaxAR_Engine generate_embedded_resources)


target_link_libraries(VtonaxAR_Engine
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
    target_link_libraries(VtonaxAR_Engine OpenMP::OpenMP_CXX -static-openmp)
endif()

# ========== 3DOF 独立库 ==========
add_library(Vtonax_3DOF SHARED src/main/cpp/3dof.cpp src/main/cpp/Matrix.cpp)
target_include_directories(Vtonax_3DOF PRIVATE
    ${OPENCV_SRC_DIR}/include
    ${OPENCV_SRC_DIR}/modules/core/include
)
target_link_libraries(Vtonax_3DOF ${log-lib} opencv_core)

if(OpenMP_CXX_FOUND)
    target_link_libraries(Vtonax_3DOF OpenMP::OpenMP_CXX -static-openmp)
endif()

# ========== Vtonax 性能分析器 ==========
set(VTONAX_DEVELOP_MODE ON) # <--- 修改这里：ON 开启，OFF 关闭

message(STATUS "Vtonax 分析器状态: ${VTONAX_DEVELOP_MODE}")

# 始终包含此目录，以便编译器在 OFF 模式下也能找到头文件并展开空宏
include_directories(src/main/cpp/profiler)

if(VTONAX_DEVELOP_MODE)
    # 仅在开启时添加分析器库
    add_library(Vtonax_Profiler SHARED
            src/main/cpp/profiler/VtonaxProfiler.cpp
            src/main/cpp/profiler/VtonaxProfiler.h
    )
    
    # 使用 PUBLIC 确保宏定义传播给所有链接到此库的目标（如 VtonaxAR_Engine）
    target_compile_definitions(Vtonax_Profiler PUBLIC VTONAX_DEVELOP_MODE)
    
    target_link_libraries(Vtonax_Profiler ${log-lib} -static-openmp)
    
    if(OpenMP_CXX_FOUND)
        target_link_libraries(Vtonax_Profiler OpenMP::OpenMP_CXX -static-openmp)
    endif()
    
    # 链接分析器库
    target_link_libraries(VtonaxAR_Engine Vtonax_Profiler)
endif()
