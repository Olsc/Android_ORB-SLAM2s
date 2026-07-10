#ifndef OPENCV_CPU_CONFIG_H_INCLUDED
#define OPENCV_CPU_CONFIG_H_INCLUDED

// Minimal ARM64 build - baseline NEON and FP16 support
// Required by system.cpp
#define CV_CPU_BASELINE_FEATURES CV_CPU_NEON, CV_CPU_FP16
#define CV_CPU_DISPATCH_FEATURES 0

#endif
