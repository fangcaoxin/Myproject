#include "colorSpaceConversion.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace std;

Color3d operator *(const cv::Mat& M, Color3d& v) {
	Color3d u = Color3d();
	for (int i = 0; i<3; i++) {
		u(i) = 0.0;
		for (int j = 0; j<3; j++) {
			//u(i) += M.at(i, j) * v(j);
		}
	}
	return u;
}
// Transformation from RGB to LMS
const double RGB2LMS[3][3] = {
	{ 0.3811, 0.5783, 0.0402 },
	{ 0.1967, 0.7244, 0.0782 },
	{ 0.0241, 0.1288, 0.8444 }
};

// Transformation from LMS to RGB
const double LMS2RGB[3][3] = {
	{ 4.4679, -3.5873,  0.1193 },
	{ -1.2186,  2.3809, -0.1624 },
	{ 0.0497, -0.2439,  1.2045 }
};

// First transformation from LMS to lab
const double LMS2lab1[3][3] = {
	{ 1.0 / sqrt(3.0), 0.0, 0.0 },
	{ 0.0, 1.0 / sqrt(6.0), 0.0 },
	{ 0.0, 0.0, 1.0 / sqrt(2.0) }
};

// Second transformation from LMS to lab
const double LMS2lab2[3][3] = {
	{ 1.0,  1.0,  1.0 },
	{ 1.0,  1.0, -2.0 },
	{ 1.0, -1.0,  0.0 }
};

const double eps = 1.0e-4;
static Color3d vecTColor3d(Vec3d& src) {
	Color3d v;
	v(0) = src[0];
	v(1) = src[1];
	v(2) = src[2];
	return v;
}

static Vec3d color3dTVec(Color3d& v) {
	Vec3d a;
	a[0] = v(0);
	a[1] = v(1);
	a[2] = v(2);
	return a;
}
const size_t bufsize = sizeof(double) * 3 * 3;
void bgrTlab(Mat& src, Mat& dst,Color3d& m,Color3d& s) {
	dst = Mat(src.size(), CV_64FC3);
	Mat src_tmp;
	Mat mRGB2LMS = Mat(3, 3, CV_64FC1);
	memcpy(mRGB2LMS.data, &RGB2LMS[0][0], bufsize);

	cv::Mat mLMS2lab1 = cv::Mat(3, 3, CV_64FC1);
	memcpy(mLMS2lab1.data, &LMS2lab1[0][0], bufsize);

	cv::Mat mLMS2lab2 = cv::Mat(3, 3, CV_64FC1);
	memcpy(mLMS2lab2.data, &LMS2lab2[0][0], bufsize);


	cv::Mat mLMS2lab = mLMS2lab2 * mLMS2lab1;
	
	Color3d v;
	
	Color3d mt = Color3d(0., 0., 0.);
	Color3d st = Color3d(0., 0., 0.);
	cvtColor(src, src_tmp, CV_BGR2RGB);
	src_tmp.convertTo(src_tmp, CV_64FC3, 1.0 / 255.0);
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			Vec3d v0 = src_tmp.at<Vec3d>(i, j);
			cout << v0 << endl;
			v = vecTColor3d(v0);
			v=mRGB2LMS*v;
			cout << mRGB2LMS << endl;
			for (int c = 0; c < 3; c++) {
				v(c) = v(c) > eps ? log10(v(c)) : log10(eps);
			}
			dst.at<Vec3d>(i, j) = color3dTVec(mLMS2lab*v);
			//cout << dst.at<Vec3d>(i, j) << endl;
			mt = mt + vecTColor3d(src_tmp.at<Vec3d>(i, j));
			st = st + vecTColor3d(src_tmp.at<Vec3d>(i, j))*vecTColor3d(src_tmp.at<Vec3d>(i, j));
		}
	}
	m = mt;
	s = st;
}
void labtbgr(Mat& src, Mat& dst) {
	dst = Mat(src.size(), CV_64FC3);
	Mat mRGB2LMS = Mat(3, 3, CV_64FC1);
	memcpy(mRGB2LMS.data, &RGB2LMS[0][0], bufsize);

	cv::Mat mLMS2lab1 = cv::Mat(3, 3, CV_64FC1);
	memcpy(mLMS2lab1.data, &LMS2lab1[0][0], bufsize);

	cv::Mat mLMS2lab2 = cv::Mat(3, 3, CV_64FC1);
	memcpy(mLMS2lab2.data, &LMS2lab2[0][0], bufsize);
	cv::Mat mLMS2RGB = cv::Mat(3, 3, CV_64FC1);
	memcpy(mLMS2RGB.data, &LMS2RGB[0][0], bufsize);

	cv::Mat mLMS2lab = mLMS2lab2 * mLMS2lab1;
	cv::Mat mlab2LMS = mLMS2lab.inv();
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			Vec3d v0 = src.at<Vec3d>(i, j);
			Color3d v = vecTColor3d(v0);
			v = mlab2LMS*v;
			for (int c = 0; c<3; c++) v(c) = v(c) > -5.0 ? pow(10.0, v(c)) : eps;
			dst.at <Vec3d>(i,j)= color3dTVec(mLMS2RGB*v);
		}
	}
	dst.convertTo(dst, CV_8UC3, 255.0);
	cvtColor(dst, dst, CV_RGB2BGR);
}

void colorTransfer(Mat& src, Mat& ref, Color3d& m_src, Color3d& s_src, Color3d& m_ref, Color3d& s_ref) {
	int Nt = src.rows*src.cols;
	int Nr = ref.rows*ref.cols;
	m_src = m_src.divide(Nt);
	m_ref = m_ref.divide(Nr);
	s_src = s_src.divide(Nt) - m_src*m_src;
	s_ref = s_ref.divide(Nr) - m_ref*m_ref;

	for (int i = 0; i < 3; i++) {
		s_src(i) = sqrt(s_src(i));
		s_ref(i) = sqrt(s_ref(i));
	}

	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			for (int c = 0; c < 3; c++) {
				double val = src.at<Vec3d>(i, j)[c];
				//cout << val << endl;
				src.at<Vec3d>(i, j)[c] = (val - m_src(c)) / s_src(c)*s_ref(c) + m_ref(c);
			}
		}
	}
}
