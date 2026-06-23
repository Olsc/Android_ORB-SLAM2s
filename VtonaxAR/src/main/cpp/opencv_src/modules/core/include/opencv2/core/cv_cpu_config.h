#ifndef OPENCV_CPU_CONFIG_H_INCLUDED
#define OPENCV_CPU_CONFIG_H_INCLUDED

// Minimal ARM64 build - baseline only, no runtime dispatching
// Required by system.cpp
#define CV_CPU_BASELINE_FEATURES 0
#define CV_CPU_DISPATCH_FEATURES 0

#endif
