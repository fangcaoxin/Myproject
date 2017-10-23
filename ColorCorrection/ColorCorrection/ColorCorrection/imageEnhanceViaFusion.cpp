#include "imageEnhanceViaFusion.h"
#include <math.h>

void Scliency(Mat& src, Mat& dst) {
	Mat gfbgr,LabIm;
	GaussianBlur(src, gfbgr, Size(3, 3), 3);
	cvtColor(gfbgr, LabIm, CV_BGR2Lab);
	vector<Mat> lab_channels;
	split(LabIm, lab_channels);
	Mat l = lab_channels[0];
	l.convertTo(l, CV_32FC1);
	Mat a = lab_channels[1];
	a.convertTo(a, CV_32FC1);
	Mat b = lab_channels[2];
	b.convertTo(b, CV_32FC1);

	double lm = mean(l)[0];
	double am = mean(a)[0];
	double bm = mean(b)[0];

	/*compute saliency map*/
	Mat sm;
	sm=Mat::zeros(l.rows, l.cols, l.type());
	subtract(l, Scalar(lm), l);
	subtract(a, Scalar(am), a);
	subtract(b, Scalar(bm), b);
	add(sm, l.mul(l), sm);
	add(sm, a.mul(a), sm);
	add(sm, b.mul(b), sm);
	dst = sm;
}

void LaplacianConstrast(Mat& src, Mat& dst) {
	Mat laplacian;
	Laplacian(src, laplacian, src.depth());
	convertScaleAbs(laplacian, laplacian);
	dst = laplacian;
}

void LocalConstrast(Mat& src, Mat& dst) {
	double h[5] = {1.0/16.0,4.0/16.0,6.0/16.0,4.0/16.0,1.0/16.0};
	Mat mask(5, 5, src.type());
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 5; j++) {
			mask.at<float>(i, j) = h[i] * h[j];
		}
	}
	Mat localContrast;
	filter2D(src, localContrast, src.depth(), mask);
	for (int i = 0; i < localContrast.rows; i++) {
		for (int j = 0; j < localContrast.cols; j++) {
			if (localContrast.at<float>(i, j) > CV_PI / 2.75) localContrast.at<float>(i, j) = CV_PI / 2.75;
		}
	}
	subtract(src, localContrast, localContrast);
	dst = localContrast.mul(localContrast);
}

void Exposedness(Mat& src, Mat& dst) {
	double sigma = 0.25;
	double average = 0.5;
	int rows = src.rows;
	int cols = src.cols;
	Mat exposedness;
	exposedness = Mat::zeros(rows, cols, src.type());
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			double value = exp(-1.0*pow((src.at<float>(i, j) - average), 2.0) / (2 * pow(sigma,2.0)));
			exposedness.at<float>(i, j) = value;
		}
	}
	dst = exposedness;
}

void LaplacianPyramid(Mat& src, vector<Mat>& pyramids, int level) {
	vector<Mat> pyramids_tmp(level);
	pyramids_tmp[0] = src.clone();
	Mat tmpImg = src.clone();
	for (int i = 1; i < level; i++) {
		
		resize(tmpImg, tmpImg, Size(), 0.5, 0.5, INTER_LINEAR);
		pyramids_tmp[i] = tmpImg.clone();
		
	}
	for (int i = 0; i < level-1; i++) {
		Mat tmpPyr;
		resize(pyramids_tmp[i + 1], tmpPyr, pyramids_tmp[i].size(), 0, 0, INTER_LINEAR);
		subtract(pyramids_tmp[i], tmpPyr, pyramids_tmp[i]);
	}
	pyramids = pyramids_tmp;
}

void PyramidReconstruct(vector<Mat>& pyramid, Mat& dst) {
	int level = pyramid.size();
	for (int i = level - 1; i > 0; i--) {
		Mat tmpPyr;
		resize(pyramid[i], tmpPyr, pyramid[i - 1].size(), 0, 0, INTER_LINEAR);
		add(pyramid[i - 1], tmpPyr, pyramid[i - 1]);
	}
	dst = pyramid[0];
}

void filterMask(Mat& src, Mat& mask) {
	double h[5] = { 1.0 / 16.0, 4.0 / 16.0, 6.0 / 16.0, 4.0 / 16.0, 1.0 / 16.0 };
	mask.create(5, 5, src.type());
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 5; j++) {
			mask.at<float>(i, j) = h[i] * h[j];
		}
	}
}

void GaussianPyramid(Mat& src, vector<Mat>& pyramid, int level) {
	vector<Mat> pyramids_tmp(level);
	Mat mask;
	filterMask(src, mask);
	Mat tmp;
	filter2D(src, tmp, -1, mask);
	pyramids_tmp[0] = src.clone();
	Mat tmpImg = src.clone();
	for (int i = 1; i < level; i++) {

		resize(tmpImg, tmpImg, Size(), 0.5, 0.5, INTER_LINEAR);
		filter2D(tmpImg, tmp, -1, mask);
		pyramids_tmp[i] = tmp.clone();

	}
	
	pyramid = pyramids_tmp;
}

void SimplestColorBalance(Mat src, Mat& dst, int percent) {
	if (percent<=0) {
		percent = 5;
	}
	src.convertTo(src, CV_32F);
	double halfPercent = percent/200.0;
	vector<Mat> channel;
	int chs = src.channels();
	if (chs == 3) {
		split(src, channel);
	}
	else {
		channel.push_back(src);
	}

	vector<Mat> results;
	for (int i = 0; i < chs; i++) {
		Mat flat;
		channel[i].reshape(1, 1).copyTo(flat);
		cv::sort(flat, flat, SORT_ASCENDING);
		double lowVal = flat.at<float>(0, (int)floor(flat.cols*halfPercent));
		double topVal = flat.at<float>(0, ceil(flat.cols*(1 - halfPercent)));

		Mat channel1 = channel[i];
		for (int m = 0; m < src.rows; m++) {
			for (int n = 0; n < src.cols; n++) {
				if (channel1.at<float>(m, n) < lowVal) {
					channel1.at<float>(m, n) = lowVal;
				}

				if (channel1.at<float>(m, n) > topVal) {
					channel1.at<float>(m, n) = topVal;
				}
			}
		}
		normalize(channel1, channel1, 0, 255, NORM_MINMAX);
		results.push_back(channel1);
	}
	//results.push_back(channel[2]);
	Mat outval;
	merge(results, outval);
	dst = outval;
}