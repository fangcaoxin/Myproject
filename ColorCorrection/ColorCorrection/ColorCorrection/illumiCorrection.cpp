#include "illumiCorrection.h"
#include <opencv2/ximgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace ximgproc;
static void log_Mat(Mat& src, Mat& dst) {
	if (src.channels() != 1) {
		cout << "the channel should be one" << endl;
		return;
	}
	dst.create(src.size(), CV_32FC1);
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			if (src.at<float>(i, j) < 1) {
				dst.at<float>(i, j) = 0;
			}
			else {
				dst.at<float>(i, j) = log(src.at<float>(i, j));
			}
		}
	}
}
void illumiCorrection(Mat& src,Mat& dst) {
	Mat src_hsv,ii;
	Mat L, R;
	Mat ll_estimated,rr_estimated,ll,ll_3c;
	vector<Mat> channels;
	vector<Mat> channels_rgb;
	vector<Mat> ii_list;
	vector<Mat> ll_list;
	vector<Mat> rr_list;
	Mat ii_b, ii_g, ii_r;
	Mat ll_b, ll_g, ll_r;
	src.convertTo(src, CV_32FC3);
	split(src, channels_rgb);
	cvtColor(src, src_hsv, CV_BGR2HSV);
	
	/*for (int k = 0; k < 10; k++) {
		cout << "ii" << src.at<Vec3f>(k, 10) << endl;
	}*/
	split(src_hsv, channels);
	log_Mat(channels[2], ll);
	log_Mat(channels_rgb[0], ii_b);
	log_Mat(channels_rgb[1], ii_g);
	log_Mat(channels_rgb[2], ii_r);
	
	guidedFilter(ii_b, ll, ll_b, 32, 1e-2);
	guidedFilter(ii_g, ll, ll_g, 32, 1e-2);
	guidedFilter(ii_r, ll, ll_r, 32, 1e-2);
	/*ll_estimated.convertTo(ll_estimated, CV_8UC3);
	imshow("ll_estimated", ll_estimated);
	waitKey(0)*/;
	ll_list.push_back(ll_b);
	ll_list.push_back(ll_g);
	ll_list.push_back(ll_r);

	Mat rr_b = ii_b - ll_b;
	Mat rr_g = ii_g - ll_g;
	Mat rr_r = ii_r - ll_r;
	guidedFilter(rr_b, rr_b, rr_b, 32, 1e-2);
	guidedFilter(rr_g, rr_g, rr_g, 32, 1e-2);
	guidedFilter(rr_r, rr_r, rr_r, 32, 1e-2);
	rr_list.push_back(rr_b);
	rr_list.push_back(rr_g);
	rr_list.push_back(rr_r);
	merge(rr_list, rr_estimated);
	merge(ll_list, ll_estimated);
	exp(ll_estimated, L);
	normalize(L, L, 0, 1, NORM_MINMAX, -1, Mat());
	exp(rr_estimated, R);
	pow(L, 1./3, L);
	for (int k = 0; k < 10; k++) {
		cout << "rr" << R.at<Vec3f>(k, 10) << endl;
		cout << "ll" << L.at<float>(k, 10) << endl;
	}
	//cvtColor(L, L, CV_GRAY2BGR);
	dst = L.mul(R);
	normalize(dst, dst, 0, 255, NORM_MINMAX, -1, Mat());
	//normalize(R,R, 0, 1, NORM_MINMAX, -1, Mat());
	//L.convertTo(L, CV_8UC1);
	dst.convertTo(dst, CV_8UC3);
}