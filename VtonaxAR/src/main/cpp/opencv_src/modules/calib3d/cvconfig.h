#ifndef OPENCV_CVCONFIG_H_INCLUDED
#define OPENCV_CVCONFIG_H_INCLUDED

/* OpenCV intrinsics optimized code */
#define CV_ENABLE_INTRINSICS

/* #undef CV_DISABLE_OPTIMIZATION */

/* Define to 1 if you have the <inttypes.h> header file. */
#define HAVE_INTTYPES_H 1

/* Posix threads (pthreads) */
#define HAVE_PTHREAD
#define HAVE_PTHREADS_PF

/* OpenCV trace utilities */
#define OPENCV_TRACE

#endif // OPENCV_CVCONFIG_H_INCLUDED
