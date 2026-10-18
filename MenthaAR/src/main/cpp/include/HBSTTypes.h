/**
 * Copyright (C) 2025-2026 Olsc <OlscStudio@outlook.com>
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

#ifndef HBST_TYPES_H
#define HBST_TYPES_H

#define SRRG_HBST_HAS_OPENCV

#include "Thirdparty/srrg_hbst/types/binary_tree.hpp"
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

// ObjectType 为 size_t，表示关键点在关键帧中的索引
typedef srrg_hbst::BinaryMatchable256<size_t> HBSTMatchable;
typedef srrg_hbst::BinaryTree256<size_t> HBSTTree;
typedef srrg_hbst::BinaryNode256<size_t> HBSTNode;

}

#endif // HBST_TYPES_H