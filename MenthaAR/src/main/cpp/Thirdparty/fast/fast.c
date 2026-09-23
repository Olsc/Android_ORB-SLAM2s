#include <stdlib.h>
#include "fast.h"

xy* fast9_detect_nonmax(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners)
{
	return fast9_detect_nonmax_with_scores(im, xsize, ysize, stride, b, ret_num_corners, NULL);
}

xy* fast9_detect_nonmax_with_scores(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners, int** ret_scores)
{
	xy* corners;
	int num_corners;
	int* scores;
	xy* nonmax;

	corners = fast9_detect(im, xsize, ysize, stride, b, &num_corners);
	if (num_corners <= 0)
	{
		if (corners) free(corners);
		*ret_num_corners = 0;
		if (ret_scores) *ret_scores = NULL;
		return NULL;
	}

	scores = fast9_score(im, stride, corners, num_corners, b);
	nonmax = nonmax_suppression_with_scores(corners, scores, num_corners, ret_num_corners, ret_scores);

	free(corners);
	free(scores);

	return nonmax;
}

int fast9_detect_nonmax_with_scores_stream(
    const byte* im, int xsize, int ysize, int stride, int b,
    xy* corners_buf, int* scores_buf,
    xy* nonmax_out, int* nonmax_scores_out,
    int* row_start_buf, int max_corners, int max_row)
{
	int num_corners = fast9_detect_buf(im, xsize, ysize, stride, b, corners_buf, max_corners);
	if (num_corners <= 0)
		return 0;
	if (num_corners > max_corners)
		return num_corners; // 缓冲不足：返回哨兵，调用方扩容后重试（不截断）

	fast9_score_buf(im, stride, corners_buf, num_corners, b, scores_buf);

	return nonmax_suppression_with_scores_buf(
		corners_buf, scores_buf, num_corners,
		nonmax_out, nonmax_scores_out, row_start_buf, max_row);
}
