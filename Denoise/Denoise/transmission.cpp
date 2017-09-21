#include <stdio.h>
#include "core.h"
#include <opencv2/highgui/highgui.hpp>
void calcTransmission(Mat& transmission, Mat& input, double A[], int radius)
{
	int width = input.cols;
	int height = input.rows;

	double w = 0.95;

	Mat normalized_input(height, width, CV_8UC3);
	for (int k = 0; k < 3; k++) {
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				double tmp = input.at<Vec3b>(i, j)[k];
				tmp = tmp / A[k] * 255.0;

				tmp = tmp > 255 ? 255 : tmp;
				normalized_input.at<Vec3b>(i, j)[k] = cvRound(tmp);
			}
		}
	}
	//imshow("normalized_input", normalized_input);
	Mat brightChannel(height, width, CV_8UC1);
	calcDarkChannel(transmission, brightChannel, normalized_input, radius);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			double tran = transmission.at<uchar>(i, j);

			tran = 1 - w*(tran / 255.0);
			tran *= 255.0;
			tran = tran > 255 ? 255 : (tran < 0 ? 0 : tran);
			transmission.at<uchar>(i, j) = cvRound(tran);
		}
	}
}