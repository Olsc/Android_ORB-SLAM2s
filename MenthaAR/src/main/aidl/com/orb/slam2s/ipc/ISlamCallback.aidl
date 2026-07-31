package com.orb.slam2s.ipc;

interface ISlamCallback {
    void onMVPUpdated(in float[] modelMatrix, in float[] viewMatrix, in float[] projectionMatrix, boolean draw);
    void onTrackingStatusChanged(int trackingStatus);
    void onPlaneDetected(int result);
    void onSLAMInitialized(boolean success, String message);
}
