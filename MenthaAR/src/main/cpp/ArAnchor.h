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

// AR 物体/平面锚点 —— 集中式状态定义

#ifndef ORB_SLAM2_AR_ANCHOR_H
#define ORB_SLAM2_AR_ANCHOR_H

#include "Plane.h"

#include <memory>
#include <string>
#include <vector>

namespace AR {

// 锚点所在坐标系的唯一表达
enum class AnchorFrame {
    kSlam,   // 坐标在"实时 SLAM 世界帧"（手动检测的本地平面）
    kMap,    // 坐标在"（对齐后的）目标/地图帧"（从地图文件加载的平面）
};

// 附属在锚点上的可序列化 AR 物体条目（保持现有 .arinfo 文件格式）
struct ArObject {
    float modelMatrix[16] = {0};
    std::string objectId;
    bool isValid = false;
    float scale = 0.0f;
};

// 唯一事实源："当前 AR 锚点 + 它所在坐标系"。
struct ArAnchor {
    std::unique_ptr<Plane> plane;     // 可为 null（地图有物体但无平面）
    AnchorFrame frame = AnchorFrame::kSlam;
    std::vector<ArObject> objects;    // 供 getAllArObjectsData / SavePlaneAndArInfo
    bool isFromLoadedMap = false;     // 绘制门控：地图锚点必须对齐后才能显示
    bool valid = false;

    // 深拷贝：unique_ptr 复制语义（工程按 C++11 编译，不用 std::make_unique）
    ArAnchor Clone() const {
        ArAnchor c;
        c.plane = plane ? std::unique_ptr<Plane>(new Plane(*plane)) : nullptr;
        c.frame = frame;
        c.objects = objects;
        c.isFromLoadedMap = isFromLoadedMap;
        c.valid = valid;
        return c;
    }

    void Reset() {
        *this = ArAnchor();
    }
};

// 渲染层对齐滞回状态（与 SLAM 核心 mbHaveMapAlign 解耦）
struct AlignHoldState {
    bool effAligned  = false;          // 当前"按地图帧渲染"（滞回后的有效值）
    int   dropHold   = 0;              // raw 对齐丢失后的保持帧计数
    bool  hasLastGood = false;         // lastView/lastModel 是否有效
    float lastView[16] = {0};          // 最后已知"对齐帧"视图矩阵（持帧冻结用）
    float lastModel[16] = {0};         // 最后已知"对齐帧"模型矩阵（持帧冻结用）

    void Reset() {
        effAligned = false;
        dropHold = 0;
        hasLastGood = false;
    }
};

} // namespace AR

#endif // ORB_SLAM2_AR_ANCHOR_H