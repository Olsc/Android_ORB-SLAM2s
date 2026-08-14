/* Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s project (a fork of ORB-SLAM2).
 * Licensed under the GNU General Public License v3.0 (or later).
 * See <https://www.gnu.org/licenses/> for the full license text.
 */
package com.orb.slam2s.ipc;

interface ISlamCallback {
    // 每帧的 MVP / 跟踪状态 / 点云已改为共享内存回传，不再走 binder 回调
    oneway void onPlaneDetected(int result);
    oneway void onSLAMInitialized(boolean success, String message);
}
