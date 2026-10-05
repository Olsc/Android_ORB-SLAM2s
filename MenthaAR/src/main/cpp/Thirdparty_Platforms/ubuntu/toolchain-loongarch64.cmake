# ============================================================================
# LoongArch64 (龙芯) 交叉编译工具链文件
#
# 用法:
#   cmake .. -DCMAKE_TOOLCHAIN_FILE=toolchain-loongarch64.cmake \
#            -DCMAKE_BUILD_TYPE=Release
#
# 可通过 -DLOONGARCH_MARCH=la464|la664|loongarch64|la64v1.0|la64v1.1 指定目标 CPU:
#   loongarch64 : LA64 基础 ISA，兼容所有龙芯 64 位 CPU（默认）
#   la464       : 3A5000 / 3C5000 / 3A5000M 系列
#   la664       : 3A6000 / 3C6000 系列
#   la64v1.0    : LA64 v1.0
#   la64v1.1    : LA64 v1.1
# ============================================================================

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR loongarch64)

# ---- 工具链根目录（可用 -DLOONGARCH_TOOLCHAIN_ROOT=... 覆盖）----------------
if(NOT DEFINED LOONGARCH_TOOLCHAIN_ROOT OR LOONGARCH_TOOLCHAIN_ROOT STREQUAL "")
    set(LOONGARCH_TOOLCHAIN_ROOT "/opt/x-tools/loongarch64-unknown-linux-gnu")
endif()

set(_LOONG_TRIPLE "loongarch64-unknown-linux-gnu")
set(_LOONG_BIN "${LOONGARCH_TOOLCHAIN_ROOT}/bin")
set(_LOONG_SYSROOT "${LOONGARCH_TOOLCHAIN_ROOT}/${_LOONG_TRIPLE}/sysroot")

set(CMAKE_C_COMPILER   "${_LOONG_BIN}/${_LOONG_TRIPLE}-gcc")
set(CMAKE_CXX_COMPILER "${_LOONG_BIN}/${_LOONG_TRIPLE}-g++")
set(CMAKE_AR           "${_LOONG_BIN}/${_LOONG_TRIPLE}-ar")
set(CMAKE_RANLIB       "${_LOONG_BIN}/${_LOONG_TRIPLE}-ranlib")
set(CMAKE_STRIP        "${_LOONG_BIN}/${_LOONG_TRIPLE}-strip")
set(CMAKE_OBJCOPY      "${_LOONG_BIN}/${_LOONG_TRIPLE}-objcopy")
set(CMAKE_OBJDUMP      "${_LOONG_BIN}/${_LOONG_TRIPLE}-objdump")

# ---- 目标 CPU / ABI --------------------------------------------------------
if(NOT DEFINED LOONGARCH_MARCH OR LOONGARCH_MARCH STREQUAL "")
    set(LOONGARCH_MARCH "loongarch64")
endif()
set(LOONGARCH_ABI "lp64d" CACHE STRING "LoongArch ABI (lp64d)")

# 交给各编译器/项目使用
set(CMAKE_C_FLAGS_INIT   "-march=${LOONGARCH_MARCH} -mabi=${LOONGARCH_ABI}")
set(CMAKE_CXX_FLAGS_INIT "-march=${LOONGARCH_MARCH} -mabi=${LOONGARCH_ABI}")

# ---- sysroot / 查找规则 ----------------------------------------------------
set(CMAKE_SYSROOT "${_LOONG_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH "${_LOONG_SYSROOT}" "${LOONGARCH_TOOLCHAIN_ROOT}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ---- pkg-config：强制在 sysroot 内查找目标库 --------------------------------
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${_LOONG_SYSROOT}")
set(ENV{PKG_CONFIG_LIBDIR} "${_LOONG_SYSROOT}/usr/lib64/pkgconfig:${_LOONG_SYSROOT}/usr/lib/pkgconfig:${_LOONG_SYSROOT}/usr/share/pkgconfig")

# ---- 交叉编译时 try_run 无法执行，使用 qemu 模拟（若已安装）------------------
find_program(_LOONG_QEMU qemu-loongarch64 qemu-loongarch64-static)
if(_LOONG_QEMU)
    set(CMAKE_CROSSCOMPILING_EMULATOR "${_LOONG_QEMU};-L;${_LOONG_SYSROOT}")
endif()
