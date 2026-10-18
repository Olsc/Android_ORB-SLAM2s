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

#ifndef VTONAX_RANDOM_H
#define VTONAX_RANDOM_H

#include <cstdint>

namespace ORB_SLAM2 {

// 独立的随机数生成器（LCG）
class LCG {
public:
    explicit LCG(unsigned int seed = 0) : mRandState(seed) {}

    inline int randomInt(int min, int max) {
        mRandState = mRandState * 1664525u + 1013904223u;
        return min + (int)((((uint64_t)mRandState * (uint64_t)(max - min + 1)) >> 32));
    }

private:
    unsigned int mRandState;
};

} // namespace ORB_SLAM2

#endif // VTONAX_RANDOM_H