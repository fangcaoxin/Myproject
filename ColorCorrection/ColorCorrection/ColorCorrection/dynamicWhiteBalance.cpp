#include "dynamicWhiteBalance.h"

void dynamicWhiteBalance(Mat& src, Mat& dst) {
	Mat tmp;
	int width = src.cols;
	int height = src.rows;
	int step_x = floor(width / 4);
	int step_y = floor(height / 4);
	cvtColor(src, tmp, CV_BGR2YCrCb);
	
	double d_b[12] = { 0. };
	double d_r[12] = { 0. };
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			Scalar means, stddevs;
			Rect rect(i*step_x, j*step_y, step_x, step_y);
			meanStdDev(tmp(rect), means, stddevs);
			d_r[i * 4 + j] = stddevs[1];
			d_b[i * 4 + j] = stddevs[2];

		}
	}
	


}