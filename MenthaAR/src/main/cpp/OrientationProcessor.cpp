/**
 * Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s project (a fork of ORB-SLAM2).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "OrientationProcessor.h"
#include "Matrix.h"
#include <cstring>
#include <cmath>

// 对应 Android SensorManager 坐标轴常量
static const int SENSOR_AXIS_X       = 1;
static const int SENSOR_AXIS_Y       = 2;
static const int SENSOR_AXIS_MINUS_X = 1 | 0x80;
static const int SENSOR_AXIS_MINUS_Y = 2 | 0x80;

OrientationProcessor::OrientationProcessor() {
    Reset();
}

void OrientationProcessor::Reset() {
    std::lock_guard<std::mutex> lock(mMutex);
    setIdentityM(mRawRotationMatrix);
    setIdentityM(mRotationMatrix);
    mHasGravity = false;
    mHasGeomagnetic = false;
    mLastGyroTimestamp = 0;
    mReady = false;
}

void OrientationProcessor::UpdateRotationVector(const float* values, int length, int displayRotation) {
    if (!values || length < 3) return;
    std::lock_guard<std::mutex> lock(mMutex);
    getRotationMatrixFromVector(mRawRotationMatrix, values, length);
    RemapForLandscapeInternal(displayRotation);
    mReady = true;
}

void OrientationProcessor::UpdateAccelerometer(const float* values, int displayRotation) {
    if (!values) return;
    std::lock_guard<std::mutex> lock(mMutex);
    std::memcpy(mGravity, values, 3 * sizeof(float));
    mHasGravity = true;
    UpdateRotationFromAccelMagInternal(displayRotation);
}

void OrientationProcessor::UpdateMagnetometer(const float* values, int displayRotation) {
    if (!values) return;
    std::lock_guard<std::mutex> lock(mMutex);
    std::memcpy(mGeomagnetic, values, 3 * sizeof(float));
    mHasGeomagnetic = true;
    UpdateRotationFromAccelMagInternal(displayRotation);
}

void OrientationProcessor::UpdateRotationFromAccelMagInternal(int displayRotation) {
    if (mHasGravity && mHasGeomagnetic) {
        if (getRotationMatrixFromAccelMag(mRawRotationMatrix, mGravity, mGeomagnetic)) {
            RemapForLandscapeInternal(displayRotation);
            mReady = true;
        }
    }
}

void OrientationProcessor::IntegrateGyroscope(const float* values, int64_t timestampNs, int displayRotation) {
    if (!values) return;
    std::lock_guard<std::mutex> lock(mMutex);
    if (mLastGyroTimestamp == 0) {
        mLastGyroTimestamp = timestampNs;
        return;
    }

    float dt = static_cast<float>(timestampNs - mLastGyroTimestamp) * 1.0e-9f;
    mLastGyroTimestamp = timestampNs;
    if (dt <= 0.0f || dt > 0.5f) {
        return;
    }

    // 小角度近似：角增量矢量 = 角速度 * dt
    float gyroDeltaVector[3] = {
        values[0] * dt,
        values[1] * dt,
        values[2] * dt
    };

    float gyroDeltaMatrix[16];
    getRotationMatrixFromVector(gyroDeltaMatrix, gyroDeltaVector, 3);

    // R = R * deltaR（复用 Matrix.cpp 的 4x4 列主序乘法）
    float tempMatrix[16];
    multiplyMM(tempMatrix, mRawRotationMatrix, gyroDeltaMatrix);
    std::memcpy(mRawRotationMatrix, tempMatrix, 16 * sizeof(float));

    // 定期正交规范化，抑制数值积分漂移（复用 Matrix.cpp 的 orthonormalize）
    orthonormalize(mRawRotationMatrix);

    // 重映射横屏
    RemapForLandscapeInternal(displayRotation);

    mReady = true;
}

void OrientationProcessor::RemapForDisplayRotation(int displayRotation) {
    std::lock_guard<std::mutex> lock(mMutex);
    RemapForLandscapeInternal(displayRotation);
}

void OrientationProcessor::RemapForLandscapeInternal(int displayRotation) {
    // 兼容 Android Surface.ROTATION_270(3) 与角度制 180（右横屏）
    if (displayRotation == 3 || displayRotation == 180) {
        // 右横屏（reverse landscape）：新 X 轴 = 旧 -Y 轴，新 Y 轴 = 旧 X 轴
        remapCoordinateSystem(mRawRotationMatrix, SENSOR_AXIS_MINUS_Y, SENSOR_AXIS_X, mRotationMatrix);
    } else {
        // 左横屏（landscape，默认）：新 X 轴 = 旧 Y 轴，新 Y 轴 = 旧 -X 轴
        remapCoordinateSystem(mRawRotationMatrix, SENSOR_AXIS_Y, SENSOR_AXIS_MINUS_X, mRotationMatrix);
    }
}

bool OrientationProcessor::IsReady() const {
    std::lock_guard<std::mutex> lock(mMutex);
    return mReady;
}

void OrientationProcessor::GetRotationMatrix(float* outMatrix) const {
    if (!outMatrix) return;
    std::lock_guard<std::mutex> lock(mMutex);
    std::memcpy(outMatrix, mRotationMatrix, 16 * sizeof(float));
}