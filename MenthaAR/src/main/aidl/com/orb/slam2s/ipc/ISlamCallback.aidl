/* Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s project (a fork of ORB-SLAM2).
 * Licensed under the GNU General Public License v3.0 (or later).
 * See <https://www.gnu.org/licenses/> for the full license text.
 */
package com.orb.slam2s.ipc;

interface ISlamCallback {
    void onMVPUpdated(in float[] modelMatrix, in float[] viewMatrix, in float[] projectionMatrix, boolean draw);
    void onTrackingStatusChanged(int trackingStatus);
    void onPlaneDetected(int result);
    void onSLAMInitialized(boolean success, String message);
}
