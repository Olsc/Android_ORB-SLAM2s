#ifndef OPENCV_CPU_CONFIG_H_INCLUDED
#define OPENCV_CPU_CONFIG_H_INCLUDED

// Minimal ARM64 build - baseline only, no runtime dispatching
// NEON is auto-detected from __ARM_NEON__ on ARM64
// No CV_CPU_DISPATCH_COMPILE_* defines → dispatch falls through to baseline

#endif
