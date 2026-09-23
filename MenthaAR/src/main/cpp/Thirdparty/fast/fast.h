#ifndef FAST_H
#define FAST_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { int x, y; } xy; 
typedef unsigned char byte;

int fast9_corner_score(const byte* p, const int pixel[], int bstart);

xy* fast9_detect(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);

int* fast9_score(const byte* i, int stride, xy* corners, int num_corners, int b);

xy* fast9_detect_nonmax(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);

xy* fast9_detect_nonmax_with_scores(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners, int** ret_scores);

xy* nonmax_suppression(const xy* corners, const int* scores, int num_corners, int* ret_num_nonmax);

xy* nonmax_suppression_with_scores(const xy* corners, const int* scores, int num_corners, int* ret_num_nonmax, int** ret_scores);

// 复用调用方缓冲的零分配接口
int fast9_detect_buf(const byte* im, int xsize, int ysize, int stride, int b,
                     xy* corners_out, int max_corners);

void fast9_score_buf(const byte* i, int stride, const xy* corners, int num_corners, int b,
                     int* scores_out);

int nonmax_suppression_with_scores_buf(const xy* corners, const int* scores, int num_corners,
                                       xy* nonmax_out, int* scores_out, int* row_start_buf, int max_row);

int fast9_detect_nonmax_with_scores_stream(
    const byte* im, int xsize, int ysize, int stride, int b,
    xy* corners_buf, int* scores_buf,
    xy* nonmax_out, int* nonmax_scores_out,
    int* row_start_buf, int max_corners, int max_row);

#ifdef __cplusplus
}
#endif

#endif
