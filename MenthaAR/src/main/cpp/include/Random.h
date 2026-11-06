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
#include <cstddef>
#include <vector>
#include <utility>

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

// 无放回抽取 m 个下标（RANSAC 最小集）。
// 原实现每次迭代先整份拷贝可用下标数组，再“v[randi]=v.back(); v.pop_back()”：写入 O(N)。
// 这里在同一数组上把选中元素换到活动区末尾，抽完后按相反顺序换回：活动前缀与原实现逐步完全相同，
// 因此在同一随机源下产生逐位相同的样本序列（docs/bench/algo_bench.py §6），而写入只有 O(m)。
// vIndices 调用前后内容不变。
template<class Rng>
inline void SampleWithoutReplacement(Rng& rng, std::vector<size_t>& vIndices, const int m, size_t* out)
{
    int randi[16];
    size_t n = vIndices.size();
    for(int j=0; j<m; ++j)
    {
        randi[j] = rng.randomInt(0, (int)n - 1);
        out[j] = vIndices[randi[j]];
        std::swap(vIndices[randi[j]], vIndices[n-1]);
        --n;
    }
    for(int j=m-1; j>=0; --j)
    {
        ++n;
        std::swap(vIndices[randi[j]], vIndices[n-1]);
    }
}

} // namespace ORB_SLAM2

#endif // VTONAX_RANDOM_H