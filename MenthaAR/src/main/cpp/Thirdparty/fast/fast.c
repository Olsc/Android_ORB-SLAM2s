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
