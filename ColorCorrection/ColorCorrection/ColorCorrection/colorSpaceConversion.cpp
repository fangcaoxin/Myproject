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
static Mat vecTMat(Vec3d& src) {
	Mat v(3, 1, CV_64FC1);
	v.at<double>(0,0) = src[0];
	v.at<double>(1, 0) = src[1];
	v.at<double>(2, 0) = src[2];
	//cout << v.data[0] << " " << v.data[1] << " " << v.data[2] << endl;
	return v;
}

static Vec3d MatTVec(Mat v) {
	Vec3d a;
	a[0] = v.at<double>(0, 0);
	a[1] = v.at<double>(1, 0);
	a[2] = v.at<double>(2, 0);
	return a;
}
const size_t bufsize = sizeof(double) * 3 * 3;
void bgrTlab(Mat& src, Mat& dst,Mat& m,Mat& s) {
	dst = Mat(src.size(), CV_64FC3);
	Mat src_tmp;
	Mat mRGB2LMS = Mat(3, 3, CV_64FC1);
	memcpy(mRGB2LMS.data, &RGB2LMS[0][0], bufsize);

	cv::Mat mLMS2lab1 = cv::Mat(3, 3, CV_64FC1);
	memcpy(mLMS2lab1.data, &LMS2lab1[0][0], bufsize);

	cv::Mat mLMS2lab2 = cv::Mat(3, 3, CV_64FC1);
	memcpy(mLMS2lab2.data, &LMS2lab2[0][0], bufsize);


	cv::Mat mLMS2lab = mLMS2lab2 * mLMS2lab1;
	
	Mat v;
	
	Mat mt(3,1,CV_64FC1,Scalar(0));
	Mat st(3, 1, CV_64FC1, Scalar(0));
	
	cvtColor(src, src_tmp, CV_BGR2RGB);
	src_tmp.convertTo(src_tmp, CV_64FC3, 1.0 / 255.0);
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			Vec3d v0 = src_tmp.at<Vec3d>(i, j);
			//cout <<"v0 "<< v0 << endl;
			v = vecTMat(v0);
			//cout << "v "<<v << endl;
			v=mRGB2LMS*v;
			//cout << v << endl;
			for (int c = 0; c < 3; c++) {
				v.at<double>(c,0) = v.at<double>(c, 0) > eps ? log10(v.at<double>(c, 0)) : log10(eps);
			}
			dst.at<Vec3d>(i, j) = MatTVec(mLMS2lab*v);
			//cout << dst.at<Vec3d>(i, j) << endl;
			mt = mt + vecTMat(src_tmp.at<Vec3d>(i, j));
			st = st + vecTMat(src_tmp.at<Vec3d>(i, j)).mul(vecTMat(src_tmp.at<Vec3d>(i, j)));
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
			Mat v = vecTMat(v0);
			v = mlab2LMS*v;
			for (int c = 0; c<3; c++) v.at<double>(c, 0) = v.at<double>(c, 0) > -5.0 ? pow(10.0, v.at<double>(c, 0)) : eps;
			dst.at <Vec3d>(i,j)= MatTVec(mLMS2RGB*v);
		}
	}
	dst.convertTo(dst, CV_8UC3, 255.0);
	cvtColor(dst, dst, CV_RGB2BGR);
}

void colorTransfer(Mat& src, Mat& ref, Mat& m_src, Mat& s_src, Mat& m_ref, Mat& s_ref) {
	int Nt = src.rows*src.cols;
	int Nr = ref.rows*ref.cols;
	cout << m_src << endl;
	m_src = m_src/Nt;
	cout << m_src << endl;
	m_ref = m_ref/Nr;
	s_src = s_src/Nt - m_src.mul(m_src);
	s_ref = s_ref/Nr - m_ref.mul(m_ref);

	for (int i = 0; i < 3; i++) {
		s_src.at<double>(i,0) = sqrt(s_src.at<double>(i, 0));
		s_ref.at<double>(i, 0) = sqrt(s_ref.at<double>(i, 0));
	}

	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			for (int c = 0; c < 3; c++) {
				double val = src.at<Vec3d>(i, j)[c];
				//cout << val << endl;
				src.at<Vec3d>(i, j)[c] = (val - m_src.at<double>(c, 0)) / s_src.at<double>(c, 0)*s_ref.at<double>(c, 0) + m_ref.at<double>(c, 0);
			}
		}
	}
}
