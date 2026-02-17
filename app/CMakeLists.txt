cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_VERBOSE_MAKEFILE on)
set(libs "${CMAKE_SOURCE_DIR}/src/main/jniLibs")
include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/opencv/include)

#--------------------------------------------------- 导入 ---------------------------------------------------#
add_library(libopencv_java4 SHARED IMPORTED )
set_target_properties(libopencv_java4 PROPERTIES
        IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_java4.so")



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
)

target_compile_definitions(g2o PRIVATE
        EIGEN_DONT_ALIGN_STATICALLY
        EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
        EIGEN_MAX_ALIGN_BYTES=16)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(g2o PRIVATE
            $<$<CONFIG:Release>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing -fno-lto>
            $<$<CONFIG:Debug>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing -fno-lto>)
else()
    target_compile_options(g2o PRIVATE
            $<$<CONFIG:Release>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing -fno-lto -fno-aggressive-loop-optimizations -fno-delete-null-pointer-checks>
            $<$<CONFIG:Debug>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing -fno-lto -fno-aggressive-loop-optimizations -fno-delete-null-pointer-checks>)
endif()

include_directories(
        src/main/cpp
        src/main/cpp/include
        src/main/cpp/Thirdparty
        src/main/cpp/Thirdparty/g2o
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
set(EMBEDDED_OBJS
    ${EMBEDDED_DIR}/ORBvoc.o
    ${EMBEDDED_DIR}/ORB_LUT.o
)

# 使用符号基础名称，函数会生成 _binary_<symbol_base>_start/end 符号
# 这些符号名称必须与 EmbeddedResources.cpp 中声明的符号匹配
embed_resource(${RESOURCE_DIR}/ORBvoc.txt.arm.bin ORBvoc_txt_arm_bin ${EMBEDDED_DIR}/ORBvoc.o)
embed_resource(${RESOURCE_DIR}/ORB_LUT.bin ORB_LUT_bin ${EMBEDDED_DIR}/ORB_LUT.o)

# 创建自定义目标以确保在链接前生成所有对象文件
add_custom_target(generate_embedded_resources DEPENDS ${EMBEDDED_OBJS})


# ========== 整合所有组件到一个 SO：libSLAM_AR.so ==========
add_library(SLAM_AR SHARED
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
        
        # DBoW2 组件
        src/main/cpp/Thirdparty/DBoW2/DBoW2/BowVector.h
        src/main/cpp/Thirdparty/DBoW2/DBoW2/FORB.h
        src/main/cpp/Thirdparty/DBoW2/DBoW2/FClass.h
        src/main/cpp/Thirdparty/DBoW2/DBoW2/FeatureVector.h
        src/main/cpp/Thirdparty/DBoW2/DBoW2/ScoringObject.h
        src/main/cpp/Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h
        src/main/cpp/Thirdparty/DBoW2/DUtils/Random.h
        src/main/cpp/Thirdparty/DBoW2/DBoW2/BowVector.cpp
        src/main/cpp/Thirdparty/DBoW2/DBoW2/FORB.cpp
        src/main/cpp/Thirdparty/DBoW2/DBoW2/FeatureVector.cpp
        src/main/cpp/Thirdparty/DBoW2/DBoW2/ScoringObject.cpp
        src/main/cpp/Thirdparty/DBoW2/DUtils/Random.cpp
        
        # 工具类
        src/main/cpp/Utils.cpp
        src/main/cpp/EmbeddedResources.cpp
        
        # JNI 接口层
        src/main/cpp/native-lib.cpp
        src/main/cpp/UIUtils.cpp
        src/main/cpp/Plane.cpp
        src/main/cpp/Matrix.cpp


        # 6DoF 传感融合功能已移除
        
        # 嵌入资源
        ${EMBEDDED_OBJS}
        )

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(SLAM_AR PRIVATE
            $<$<CONFIG:Release>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing -fno-lto>
            $<$<CONFIG:Debug>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing -fno-lto>)
else()
    target_compile_options(SLAM_AR PRIVATE
            $<$<CONFIG:Release>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing -fno-lto -fno-aggressive-loop-optimizations -fno-delete-null-pointer-checks>
            $<$<CONFIG:Debug>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing -fno-lto -fno-aggressive-loop-optimizations -fno-delete-null-pointer-checks>)
endif()

target_compile_options(SLAM_AR PRIVATE
        $<$<CONFIG:Release>:-O2 -g -fno-omit-frame-pointer -fno-strict-aliasing -fno-lto -fno-aggressive-loop-optimizations -fno-delete-null-pointer-checks>)

find_library(log-lib log)

# 确保在编译前生成嵌入的资源对象文件
add_dependencies(SLAM_AR generate_embedded_resources)


target_link_libraries(SLAM_AR
        libopencv_java4
        g2o
        ${log-lib}
        EGL
        GLESv1_CM
        )

# ========== 3DOF 独立库 ==========
add_library(3dof SHARED src/main/cpp/3dof.cpp)
target_link_libraries(3dof ${log-lib})
