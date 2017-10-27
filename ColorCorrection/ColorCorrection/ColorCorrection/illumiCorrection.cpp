#include "illumiCorrection.h"
#include <opencv2/ximgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace ximgproc;

void illumiCorrection(Mat& src, Mat& L,Mat& R) {
	Mat src_hsv,ii;
	//Mat L, R;
	Mat ll_estimated,rr_estimated,ll;
	vector<Mat> channels;
	vector<Mat> ii_list;
	vector<Mat> ll_list;
	src.convertTo(src, CV_32FC3);
	//normalize(src, src, 0, 1, NORM_MINMAX, -1, Mat());
	cvtColor(src, src_hsv, CV_BGR2HSV);
	split(src, ii_list);

	log(ii_list[0], ii_list[0]);
	log(ii_list[1], ii_list[1]);
	log(ii_list[2], ii_list[2]);
	merge(ii_list, ii);
	imshow("ii", ii);
	src_hsv.convertTo(src_hsv, CV_32FC3);
	split(src_hsv, channels);
	guidedFilter(channels[2], channels[2], ll_estimated, 60, 1e-6);
	imshow("ll_estimated", ll_estimated);
	for (int i = 0; i < 3; i++) {
		ll_list.push_back(ll_estimated);
	}
	merge(ll_list, ll);
	normalize(ll, ll, 0, 1, NORM_MINMAX, -1, Mat());
	

	Mat rr = ii - ll;
	guidedFilter(rr, rr, rr_estimated, 60, 1e-6);
	exp(ll, L);
	exp(rr_estimated, R);
}