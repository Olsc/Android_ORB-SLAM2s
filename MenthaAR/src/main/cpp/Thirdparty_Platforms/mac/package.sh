#!/bin/bash
# Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
# SPDX-License-Identifier: GPL-3.0-or-later

# ============================================================================
# MenthaAR macOS 打包脚本
# 将编译好的两个可执行文件打包成 zip，方便拷贝到其他 Apple Silicon (arm64) Mac 运行。
#
# 适用目标：任意 Apple Silicon Mac（M1/M2/M3/M4...），macOS 11.0 及以上（含 macOS 27）
#
# 用法：
#   ./package.sh            # 打包 build/bin 下已编译的产物
#   ./package.sh -u         # 打包前先编译通用二进制 (arm64 + x86_64，需 -DMENTHA_UNIVERSAL=ON)
# ============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN_DIR="${SCRIPT_DIR}/build/bin"
DIST_DIR="${SCRIPT_DIR}/dist"
PACKAGE="${SCRIPT_DIR}/MenthaAR_macOS_arm64.zip"

# 检查产物是否存在
if [ ! -f "${BIN_DIR}/MenthaAR_macOS" ] || [ ! -f "${BIN_DIR}/MenthaAR_Benchmark" ]; then
    echo "[ERROR] 未找到可执行文件，请先编译："
    echo "        cd ${SCRIPT_DIR}"
    echo "        mkdir -p build && cd build"
    echo "        cmake .. -DCMAKE_BUILD_TYPE=Release"
    echo "        cmake --build . -j\$(sysctl -n hw.ncpu)"
    exit 1
fi

echo "[Package] 检测产物架构:"
lipo -info "${BIN_DIR}/MenthaAR_macOS"

rm -rf "${DIST_DIR}"
mkdir -p "${DIST_DIR}"

cp "${BIN_DIR}/MenthaAR_macOS"      "${DIST_DIR}/MenthaAR_macOS"
cp "${BIN_DIR}/MenthaAR_Benchmark"  "${DIST_DIR}/MenthaAR_Benchmark"

# Apple Silicon 要求所有可执行文件必须带签名；linker 已内嵌 ad-hoc 签名，
# 这里显式重新 ad-hoc 签名一次，确保拷贝/打包后签名仍然有效。
codesign --force --sign - "${DIST_DIR}/MenthaAR_macOS"     >/dev/null 2>&1 || true
codesign --force --sign - "${DIST_DIR}/MenthaAR_Benchmark" >/dev/null 2>&1 || true

# 生成使用说明
cat > "${DIST_DIR}/README.txt" <<'EOF'
MenthaAR macOS (Apple Silicon / arm64)
=======================================

适用环境
--------
- Apple Silicon Mac（M1/M2/M3/M4 全系列，arm64）
- macOS 11.0 及以上（含 macOS 27）
- 无需安装 OpenCV / Eigen / g2o / Homebrew，全部已静态链接

运行方式
--------
1) 实时 SLAM 演示（相机或视频）：
      ./MenthaAR_macOS
      ./MenthaAR_macOS /path/to/video.mp4
      ./MenthaAR_macOS 1          # 指定相机索引

2) 离线性能评测：
      ./MenthaAR_Benchmark /path/to/video.mp4 [报告名] [--loops N]

如果双击或命令行提示“无法验证开发者 / 已损坏 / killed”：
这是 macOS Gatekeeper 对网络传输文件的隔离机制，不是程序损坏。
在终端执行下面任意一条解除隔离即可：

      xattr -cr ./MenthaAR_macOS ./MenthaAR_Benchmark
      或
      xattr -cr /本目录路径

说明
----
- 本包为 arm64 单架构，只支持 Apple Silicon。
  如需同时支持 Intel Mac，请在源码目录用通用二进制重新编译：
      cmake .. -DMENTHA_UNIVERSAL=ON
- ESC / q 键退出实时演示。
EOF

# 打包
cd "${SCRIPT_DIR}"
rm -f "${PACKAGE}"
zip -r -q "${PACKAGE}" dist

echo ""
echo "[Package] 打包完成: ${PACKAGE}"
echo "[Package] 包内容:"
unzip -l "${PACKAGE}" | sed 's/^/    /'
echo ""
echo "[Package] 拷贝到目标 Mac 后解压，若被 Gatekeeper 拦截，执行:"
echo "    xattr -cr ./dist"