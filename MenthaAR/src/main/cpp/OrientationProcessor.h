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

#ifndef MENTHA_ORIENTATION_PROCESSOR_H
#define MENTHA_ORIENTATION_PROCESSOR_H

#include <mutex>
#include <cstdint>

// 3DOF 姿态处理器：融合旋转矢量、加速度计/磁力计与陀螺仪数据，
// 输出横屏坐标系下的旋转矩阵。
class OrientationProcessor {
public:
    OrientationProcessor();
    ~OrientationProcessor() = default;

    // 重置所有矩阵与缓存状态
    void Reset();

    // 更新旋转矢量 (ROTATION_VECTOR 或 GAME_ROTATION_VECTOR)
    void UpdateRotationVector(const float* values, int length, int displayRotation);

    // 更新加速度计数据
    void UpdateAccelerometer(const float* values, int displayRotation);

    // 更新磁力计数据
    void UpdateMagnetometer(const float* values, int displayRotation);

    // 陀螺仪角速度增量积分
    void IntegrateGyroscope(const float* values, int64_t timestampNs, int displayRotation);

    // 根据屏幕旋转方向重映射当前旋转矩阵
    void RemapForDisplayRotation(int displayRotation);

    // 检查姿态是否已就绪（已产生至少一个有效读数）
    bool IsReady() const;

    // 获取当前针对横屏重映射后的 4x4 列主序旋转矩阵（线程安全拷贝）
    void GetRotationMatrix(float* outMatrix) const;

private:
    void RemapForLandscapeInternal(int displayRotation);
    void UpdateRotationFromAccelMagInternal(int displayRotation);

    mutable std::mutex mMutex;
    float mRawRotationMatrix[16]; // 原始旋转矩阵（未按显示方向重映射）
    float mRotationMatrix[16];    // 横屏重映射后的旋转矩阵

    float mGravity[3];
    float mGeomagnetic[3];
    bool mHasGravity{false};
    bool mHasGeomagnetic{false};

    int64_t mLastGyroTimestamp{0};
    bool mReady{false};
};

#endif // MENTHA_ORIENTATION_PROCESSOR_H