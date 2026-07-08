#ifndef VTONAX_RANDOM_H
#define VTONAX_RANDOM_H

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
        return min + (int)(((double)mRandState / 4294967296.0) * (max - min + 1));
    }

private:
    unsigned int mRandState;
};

} // namespace ORB_SLAM2

#endif // VTONAX_RANDOM_H
