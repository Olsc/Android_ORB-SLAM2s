#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 可覆盖参数 -------------------------------------------------------------
TOOLCHAIN_ROOT="${TOOLCHAIN_ROOT:-/opt/x-tools/loongarch64-unknown-linux-gnu}"
LOONGARCH_MARCH="${LOONGARCH_MARCH:-loongarch64}"   # loongarch64|la464|la664|la64v1.0|la64v1.1
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"
STATIC="${STATIC:-0}"
# 后缀只反映真实的静态链接开关（注意 "0" 也是非空字符串，不能用 ${STATIC:+-static}）
if [ "$STATIC" = "1" ]; then STATIC_SUFFIX="-static"; else STATIC_SUFFIX=""; fi
# 构建目录默认落在本脚本同级目录
BUILD_DIR="${BUILD_DIR:-${SCRIPT_DIR}/build-loongarch64-${LOONGARCH_MARCH}${STATIC_SUFFIX}}"

# ---- 工具链自检 -------------------------------------------------------------
TRIPLE=loongarch64-unknown-linux-gnu
if [ ! -x "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-gcc" ]; then
    echo "错误: 未找到交叉编译器 ${TOOLCHAIN_ROOT}/bin/${TRIPLE}-gcc" >&2
    echo "请先解压官方 cross-tools 到 ${TOOLCHAIN_ROOT}，或用 TOOLCHAIN_ROOT=... 指定。" >&2
    exit 1
fi

echo "==> 交叉编译器: $("${TOOLCHAIN_ROOT}/bin/${TRIPLE}-gcc" -dumpversion)"
echo "==> 目标架构:   ${TRIPLE}  (-march=${LOONGARCH_MARCH} -mabi=lp64d)"
echo "==> 静态链接:   $([ "$STATIC" = "1" ] && echo ON || echo OFF)"
echo "==> 构建目录:   ${BUILD_DIR}"

# ---- 配置 & 构建 ------------------------------------------------------------
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake "${SCRIPT_DIR}" \
    -DCMAKE_TOOLCHAIN_FILE="${SCRIPT_DIR}/toolchain-loongarch64.cmake" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DLOONGARCH_TOOLCHAIN_ROOT="${TOOLCHAIN_ROOT}" \
    -DLOONGARCH_MARCH="${LOONGARCH_MARCH}" \
    -DMENTHA_STATIC_LINK="$([ "$STATIC" = "1" ] && echo ON || echo OFF)"

cmake --build . -- -j"${JOBS}"

echo
echo "==> 构建完成，产物:"
ls -la "${BUILD_DIR}/bin/"
file "${BUILD_DIR}/bin/MenthaAR_Ubuntu" "${BUILD_DIR}/bin/MenthaAR_Benchmark"
