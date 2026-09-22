cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  if(DEFINED ENV{VITASDK})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VITASDK}/share/vita.toolchain.cmake" CACHE PATH "toolchain file")
  else()
    message(FATAL_ERROR "Please define VITASDK to point to your SDK path!")
  endif()
endif()

project(MenthaAR_PSVita CXX C)
include("${VITASDK}/share/vita.cmake" REQUIRED)

# 路径。
set(CPP_ROOT   "${CMAKE_CURRENT_SOURCE_DIR}/../..")                 # .../src/main/cpp
set(VITA_OCV   "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libopencv4/vitasdk/arm-vita-eabi")
set(COMPAT     "${CMAKE_CURRENT_SOURCE_DIR}/compat")

# OpenCV 头文件影子副本：保持子模块干净。
if(NOT EXISTS "${VITA_OCV}/include/opencv2/core/saturate.hpp")
  message(FATAL_ERROR
    "libopencv4 submodule is missing. Run:\n"
    "  git submodule update --init --recursive")
endif()
set(OCV_SHIM_DIR "${CMAKE_CURRENT_BINARY_DIR}/opencv_shim")
file(MAKE_DIRECTORY "${OCV_SHIM_DIR}/opencv2/core")
file(READ "${VITA_OCV}/include/opencv2/core/saturate.hpp" VITA_OCV_SATURATE)
string(REGEX REPLACE "[^\n]*saturate_cast[^\n]*int32_t[^\n]*\n" "" VITA_OCV_SATURATE "${VITA_OCV_SATURATE}")
string(REGEX REPLACE "[^\n]*saturate_cast[^\n]*uint32_t[^\n]*\n" "" VITA_OCV_SATURATE "${VITA_OCV_SATURATE}")
file(WRITE "${OCV_SHIM_DIR}/opencv2/core/saturate.hpp" "${VITA_OCV_SATURATE}")

# LiveArea 气泡元数据。
set(VITA_APP_NAME "MenthaAR PSVita")
set(VITA_TITLEID  "MENTHA001")
set(VITA_VERSION  "01.00")
set(VITA_MKSFOEX_FLAGS "${VITA_MKSFOEX_FLAGS} -d PARENTAL_LEVEL=1")

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -std=gnu11 -O2 -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -w -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_ALIGN_STATICALLY -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT -DEIGEN_MAX_ALIGN_BYTES=16")

include_directories(
  src
  ${COMPAT}
  ${OCV_SHIM_DIR}
  ${CPP_ROOT}
  ${CPP_ROOT}/include
  ${CPP_ROOT}/Thirdparty
  ${CPP_ROOT}/Thirdparty/Eigen
  ${CPP_ROOT}/Thirdparty/g2o
  ${CPP_ROOT}/Thirdparty/srrg_hbst
  ${CPP_ROOT}/profiler
  ${VITA_OCV}/include
)

# g2o 优化库（从自带源码编译）。
add_library(g2o STATIC
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/batch_stats.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/cache.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/estimate_propagator.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/factory.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/hyper_graph.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/hyper_graph_action.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/jacobian_workspace.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/matrix_structure.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/optimizable_graph.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/optimization_algorithm.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/parameter.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/parameter_container.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/robust_kernel.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/solver.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/core/sparse_optimizer.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/stuff/os_specific.c
  ${CPP_ROOT}/Thirdparty/g2o/g2o/stuff/property.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/stuff/string_tools.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/stuff/timeutil.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/types/types_sba.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp
  ${CPP_ROOT}/Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp
)
# newlib 缺少 vasprintf，g2o 的 string_tools.cpp 需要它。
target_compile_options(g2o PRIVATE -include vasprintf_compat.h)

# MenthaAR / ORB-SLAM2 引擎（共享源码，原样编译）。
add_library(MenthaAR_Engine STATIC
  ${CPP_ROOT}/src/Converter.cc
  ${CPP_ROOT}/src/Frame.cc
  ${CPP_ROOT}/src/FrameDrawer.cc
  ${CPP_ROOT}/src/Initializer.cc
  ${CPP_ROOT}/src/KeyFrame.cc
  ${CPP_ROOT}/src/KeyFrameDatabase.cc
  ${CPP_ROOT}/src/LocalMapping.cc
  ${CPP_ROOT}/src/LoopClosing.cc
  ${CPP_ROOT}/src/Map.cc
  ${CPP_ROOT}/src/MapPoint.cc
  ${CPP_ROOT}/src/ORBextractor.cc
  ${CPP_ROOT}/src/ORBmatcher.cc
  ${CPP_ROOT}/src/Optimizer.cc
  ${CPP_ROOT}/src/PnPsolver.cc
  ${CPP_ROOT}/src/Sim3Solver.cc
  ${CPP_ROOT}/src/System.cc
  ${CPP_ROOT}/src/Tracking.cc
)
# mini OpenCV 无 calib3d，兼容头文件补充 undistortPoints / Rodrigues / solvePnP(PRansac)。
target_compile_options(MenthaAR_Engine PRIVATE -include opencv_calib3d_compat.h)

# Vita 前端与 calib3d / vasprintf 兼容实现。
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/camera.cpp
  src/renderer.cpp
  src/pointcloud.cpp
  src/font_data.c
  compat/opencv_calib3d_compat.cpp
  compat/vasprintf_compat.cpp
)

target_link_libraries(${PROJECT_NAME}
  -Wl,--start-group
    MenthaAR_Engine
    g2o
    ${VITA_OCV}/lib/libopencv_core.a
    ${VITA_OCV}/lib/libopencv_imgproc.a
    ${VITA_OCV}/lib/libopencv_features2d.a
    ${VITA_OCV}/lib/libopencv_imgcodecs.a
  -Wl,--end-group
  vita2d
  z
  m
  -Wl,--whole-archive
  pthread
  -Wl,--no-whole-archive
  stdc++
  SceGxm_stub
  SceDisplay_stub
  SceCtrl_stub
  SceCamera_stub
  SceMotion_stub
  SceIofilemgr_stub
  ScePvf_stub
  SceSysmodule_stub
  SceAppMgr_stub
  SceCommonDialog_stub
)

# UNSAFE：应用需要写入 ux0:data/MenthaAR。
vita_create_self(${PROJECT_NAME}.self ${PROJECT_NAME} UNSAFE)

vita_create_vpk(${PROJECT_NAME}.vpk ${VITA_TITLEID} ${PROJECT_NAME}.self
  VERSION ${VITA_VERSION}
  NAME ${VITA_APP_NAME}
  FILE sce_sys/icon0.png sce_sys/icon0.png
  FILE sce_sys/pic0.png sce_sys/pic0.png
  FILE sce_sys/livearea/contents/bg.png sce_sys/livearea/contents/bg.png
  FILE sce_sys/livearea/contents/startup.png sce_sys/livearea/contents/startup.png
  FILE sce_sys/livearea/contents/template.xml sce_sys/livearea/contents/template.xml
)

message(STATUS "[MenthaAR] PS Vita ORB-SLAM2 build configured -> ${PROJECT_NAME}.vpk")