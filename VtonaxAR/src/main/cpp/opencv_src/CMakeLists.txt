cmake_minimum_required(VERSION 3.10)
project(opencv_minimal VERSION 4.5.0 LANGUAGES CXX C)

# --- 选项 ---
option(BUILD_SHARED_LIBS "是否构建为共享库" OFF)

# --- C++ 标准 ---
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# --- OpenCV 自身编译所需的宏定义 ---
set(OPENCV_BUILD_DEFS
    __OPENCV_BUILD
    CVAPI_EXPORTS
    CV_CPU_COMPILE_NEON        # 启用所有 NEON 专用代码路径
    CV_ENABLE_INTRINSICS
)

# --- 基础头文件搜索路径 ---
set(BASE_INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# --- 需要排除的源文件（架构相关或平台相关） ---
set(CORE_EXCLUDE
    cuda_gpu_mat.cpp
    cuda_gpu_mat_nd.cpp
    cuda_host_mem.cpp
    cuda_info.cpp
    cuda_stream.cpp
    opengl.cpp
    gl_core_3_1.cpp
    ovx.cpp
    directx.cpp
    va_intel.cpp
    ocl.cpp
    bindings_utils.cpp
    # dxt.cpp 已包含（有 OpenCL 运行时桩代码）
    logger.cpp                   # 由 _missing.cpp 中的桩代码替代
    # system.cpp 已包含
    # arithm.cpp（高层 API）和 arithm.dispatch.cpp（HAL 实现）都需要
    # persistence 已包含（zlib 桩代码在 _missing.cpp 中）
)

# x86 SIMD 文件（在 ARM64 上无法编译）
set(X86_SIMD_FILES
    corner.avx.cpp
    imgwarp.avx2.cpp
    imgwarp.lasx.cpp
    imgwarp.sse4_1.cpp
    resize.avx2.cpp
    resize.lasx.cpp
    resize.sse4_1.cpp
    fast.avx2.cpp
)

# --- 需要构建的模块 ---
set(OPENCV_MODULES
    core
    imgproc
    features2d
    flann
    calib3d
)

# ============================================================================
# 辅助函数：递归收集 .cpp 文件，排除不需要的文件
# ============================================================================
function(collect_module_sources MODULE EXCLUDE_LIST OUT_VAR)
    file(GLOB_RECURSE ALL_CPP
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/${MODULE}/*.cpp
    )
    set(FILTERED "")
    foreach(src ${ALL_CPP})
        get_filename_component(fname ${src} NAME)
        list(FIND EXCLUDE_LIST ${fname} EXCLUDED)
        if(EXCLUDED EQUAL -1)
            list(APPEND FILTERED ${src})
        endif()
    endforeach()
    set(${OUT_VAR} ${FILTERED} PARENT_SCOPE)
endfunction()

# ============================================================================
# 构建各模块
# ============================================================================
foreach(mod ${OPENCV_MODULES})
    # 收集源文件并应用排除列表
    if(mod STREQUAL "core")
        set(EXCLUDE_LIST ${CORE_EXCLUDE} ${X86_SIMD_FILES})
    elseif(mod STREQUAL "features2d")
        set(EXCLUDE_LIST ${X86_SIMD_FILES} mser.cpp)    # 需要 3rdparty/mscr
    elseif(mod STREQUAL "imgproc")
        set(EXCLUDE_LIST ${X86_SIMD_FILES})
    else()
        set(EXCLUDE_LIST "")
    endif()
    collect_module_sources(${mod} "${EXCLUDE_LIST}" mod_sources)

    # 模块头文件搜索路径（包含模块根目录用于 precomp.hpp 和内部头文件）
    set(mod_includes
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/${mod}
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/${mod}/include
        ${BASE_INCLUDE}
    )

    # 创建库
    add_library(opencv_${mod} ${mod_sources})

    if(BUILD_SHARED_LIBS)
        set_target_properties(opencv_${mod} PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        )
    else()
        set_target_properties(opencv_${mod} PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        )
    endif()

    target_compile_definitions(opencv_${mod} PRIVATE ${OPENCV_BUILD_DEFS})
    # parallel/parallel.cpp 需要此宏来定义 parallel_for_()
    if(mod STREQUAL "core")
        target_compile_definitions(opencv_${mod} PRIVATE PARALLEL_ENABLE_PLUGINS)
    endif()
    target_include_directories(opencv_${mod} PUBLIC ${mod_includes})

    target_compile_options(opencv_${mod} PRIVATE -O3 -DNDEBUG)
    target_compile_definitions(opencv_${mod} PRIVATE NDEBUG)

    if("${ANDROID_ABI}" STREQUAL "arm64-v8a")
        target_compile_options(opencv_${mod} PRIVATE -march=armv8.2-a+fp16+rcpc+dotprod)
    endif()
    target_compile_options(opencv_${mod} PRIVATE
        $<$<AND:$<STREQUAL:${ANDROID_ABI},arm64-v8a>,$<CONFIG:Release>>:-flto=thin>
    )

    if(ANDROID)
        target_link_libraries(opencv_${mod} PUBLIC log)
    endif()

    set_target_properties(opencv_${mod} PROPERTIES
        OUTPUT_NAME opencv_${mod}
    )
endforeach()

# ============================================================================
# 模块依赖关系
# ============================================================================
# flann → core
target_include_directories(opencv_flann PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/core/include
)
target_link_libraries(opencv_flann PUBLIC opencv_core)

# imgproc → core
target_include_directories(opencv_imgproc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/core/include
)
target_link_libraries(opencv_imgproc PUBLIC opencv_core)

# features2d → core, imgproc, flann
target_include_directories(opencv_features2d PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/imgproc/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/flann/include
)
target_link_libraries(opencv_features2d PUBLIC opencv_core opencv_imgproc opencv_flann)

# calib3d → core, imgproc, features2d
target_include_directories(opencv_calib3d PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/imgproc/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/features2d/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/flann/include
)
target_link_libraries(opencv_calib3d PUBLIC opencv_core opencv_imgproc opencv_features2d)

# 安装规则
include(GNUInstallDirs)
foreach(mod ${OPENCV_MODULES})
    install(TARGETS opencv_${mod}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endforeach()
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

message(STATUS "精简版 OpenCV 配置完成：")
message(STATUS "  模块：${OPENCV_MODULES}")
message(STATUS "  共享库：${BUILD_SHARED_LIBS}")
message(STATUS "  平台：$<IF:$<BOOL:${ANDROID}>,Android,Desktop>")
