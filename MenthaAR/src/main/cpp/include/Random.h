#ifndef VTONAX_RANDOM_H
#define VTONAX_RANDOM_H

#include <cstdint>

namespace ORB_SLAM2 {

// 独立的随机数生成器（LCG），避免全局 rand() 共享及并发 race condition
class LCG {
public:
    explicit LCG(unsigned int seed = 0) : mRandState(seed) {}

    inline void setSeed(unsigned int seed) {
        mRandState = seed;
    }

    inline int randomInt(int min, int max) {
        mRandState = mRandState * 1664525u + 1013904223u;
        // 整数缩放 (state * range) >> 32 替代 double 除法+乘法，
        // 数学等价：floor(state / 2^32 * range)
        return min + (int)((((uint64_t)mRandState * (uint64_t)(max - min + 1)) >> 32));
    }

private:
    unsigned int mRandState;
};

} // namespace ORB_SLAM2

#endif // VTONAX_RANDOM_H