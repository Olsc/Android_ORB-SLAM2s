cmake_minimum_required(VERSION 3.10)
project(VtonaxAR_Ubuntu CXX C)

# 设置 C++ 标准为 C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------------------------------------------------------
# 1. 动态获取项目仓库的根目录绝对路径
# ----------------------------------------------------------------------------
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.." ABSOLUTE)
message(STATUS "[CMake] 检测到的项目根目录路径: ${REPO_ROOT}")

# ----------------------------------------------------------------------------
# 2. 检索系统依赖项 (仅包含 OpenCV 和 线程库，其余库均使用项目内置源码)
# ----------------------------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

message(STATUS "[CMake] OpenCV 库状态:")
message(STATUS "  - 版本: ${OpenCV_VERSION}")
message(STATUS "  - 库路径: ${OpenCV_LIBS}")
message(STATUS "  - 头文件路径: ${OpenCV_INCLUDE_DIRS}")

# ----------------------------------------------------------------------------
# 3. 编译器优化选项与编译定义
# ----------------------------------------------------------------------------
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -Wall -pthread")
add_compile_definitions(EIGEN_DONT_ALIGN_STATICALLY EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT EIGEN_MAX_ALIGN_BYTES=16)

# 头文件包含路径列表 (全部使用根目录绝对路径定位，确保无缝解析)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${REPO_ROOT}/VtonaxAR/src/main/cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/srrg_hbst
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/profiler
)

# ----------------------------------------------------------------------------
# 4. 编译第三方优化的 g2o 静态库
# ----------------------------------------------------------------------------
add_library(g2o STATIC
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_sba.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_six_dof_expmap.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_sba.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/types/se3quat.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/types/se3_ops.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/types/se3_ops.hpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_edge.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_binary_edge.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph_action.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_binary_edge.hpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph_action.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_multi_edge.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_multi_edge.hpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_unary_edge.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/linear_solver.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_unary_edge.hpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_vertex.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/base_vertex.hpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/matrix_structure.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/batch_stats.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/matrix_structure.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/batch_stats.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/openmp_mutex.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/block_solver.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/block_solver.hpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/parameter.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/parameter.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/cache.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/cache.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimizable_graph.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimizable_graph.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/solver.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/solver.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/creators.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/estimate_propagator.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_factory.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/estimate_propagator.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/factory.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_property.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/factory.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_block_matrix.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_optimizer.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_optimizer.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_dijkstra.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/parameter_container.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/parameter_container.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/jacobian_workspace.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/jacobian_workspace.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_factory.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_impl.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/string_tools.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/color_macros.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/macros.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/timeutil.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/misc.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/timeutil.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/os_specific.c
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/os_specific.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/string_tools.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/property.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Thirdparty/g2o/g2o/stuff/property.h
)

target_compile_definitions(g2o PRIVATE
    EIGEN_DONT_ALIGN_STATICALLY
    EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
    EIGEN_MAX_ALIGN_BYTES=16
)

# ----------------------------------------------------------------------------
# 5. 二进制资源嵌入逻辑 (采用高性能汇编链接模式)
# ----------------------------------------------------------------------------
set(RESOURCE_DIR ${REPO_ROOT}/VtonaxAR/src/main/other)
set(EMBEDDED_DIR ${CMAKE_CURRENT_BINARY_DIR}/embedded)
file(MAKE_DIRECTORY ${EMBEDDED_DIR})

function(embed_resource input_file symbol_base output_obj)
    set(bin_file ${EMBEDDED_DIR}/${symbol_base}.bin)
    set(asm_file ${EMBEDDED_DIR}/${symbol_base}.S)
    
    # 步骤 1: 拷贝原始二进制资源至临时编译缓存区
    add_custom_command(
        OUTPUT ${bin_file}
        COMMAND ${CMAKE_COMMAND} -E copy ${input_file} ${bin_file}
        DEPENDS ${input_file}
        COMMENT "正在拷贝二进制资源: ${input_file}"
        VERBATIM
    )
    
    # 步骤 2: 生成跨平台通用汇编代码包装器
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
        COMMENT "正在生成汇编链接包装器: ${symbol_base}"
        VERBATIM
    )
    
    # 步骤 3: 采用 host 系统的 C 编译器进行编译，生成对应的目标链接文件 (.o)
    add_custom_command(
        OUTPUT ${output_obj}
        COMMAND ${CMAKE_C_COMPILER} -c ${asm_file} -o ${output_obj}
        DEPENDS ${asm_file}
        COMMENT "正在将汇编文件包装构建为目标对象: ${symbol_base}"
        VERBATIM
    )
endfunction()

set(EMBEDDED_OBJS
    ${EMBEDDED_DIR}/ORB_LUT.o
)

embed_resource(${RESOURCE_DIR}/ORB_LUT.bin ORB_LUT_bin ${EMBEDDED_DIR}/ORB_LUT.o)

add_custom_target(generate_embedded_resources DEPENDS ${EMBEDDED_OBJS})

# ----------------------------------------------------------------------------
# 6. 构建 SLAM AR 核心底层基础库 (VtonaxAR_Engine)
# ----------------------------------------------------------------------------
add_library(VtonaxAR_Engine STATIC
    # SLAM 核心组件
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/System.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/Tracking.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/LocalMapping.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/LoopClosing.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/ORBextractor.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/ORBmatcher.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/FrameDrawer.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/Converter.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/MapPoint.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/KeyFrame.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/Map.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/Optimizer.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/PnPsolver.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/Frame.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/KeyFrameDatabase.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/Sim3Solver.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/Initializer.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/include/Common.h
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/System.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/Tracking.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/LocalMapping.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/LoopClosing.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/ORBextractor.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/ORBmatcher.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/FrameDrawer.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/Converter.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/MapPoint.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/KeyFrame.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/Map.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/Optimizer.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/PnPsolver.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/Frame.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/KeyFrameDatabase.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/Sim3Solver.cc
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/src/Initializer.cc
    
    # DBoW2 组件已移除，改用 HBST
    
    # 底层通用实用组件
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/EmbeddedResources.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/UIUtils.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Plane.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/Matrix.cpp
    ${REPO_ROOT}/VtonaxAR/src/main/cpp/profiler/VtonaxProfiler.cpp
    
    # 包装生成的二进制符号目标文件
    ${EMBEDDED_OBJS}
)

add_dependencies(VtonaxAR_Engine generate_embedded_resources)

# ----------------------------------------------------------------------------
# 7. 构建桌面可执行文件 (VtonaxAR_Ubuntu)
# ----------------------------------------------------------------------------
add_executable(VtonaxAR_Ubuntu ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(VtonaxAR_Ubuntu
    VtonaxAR_Engine
    g2o
    ${OpenCV_LIBS}
    Threads::Threads
)

# ----------------------------------------------------------------------------
# 8. 构建 Benchmark 工具 (VtonaxAR_Benchmark)
# ----------------------------------------------------------------------------
add_executable(VtonaxAR_Benchmark ${CMAKE_CURRENT_SOURCE_DIR}/benchmark.cpp)

target_link_libraries(VtonaxAR_Benchmark
    VtonaxAR_Engine
    g2o
    ${OpenCV_LIBS}
    Threads::Threads
)

message(STATUS "[CMake] 构建文件生成完毕。您现在可以在编译目录运行 'make' 命令来构建 VtonaxAR_Ubuntu 或 VtonaxAR_Benchmark。")
