#include "colorSpaceConversion.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace std;


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
static void clip(Mat& img, float minval, float maxval)
{
	CV_Assert(maxval > minval);
	size_t row = img.rows;
	size_t col = img.cols;
	for (size_t i = 0; i != row; ++i)
	{
		float* temp = img.ptr<float>(i);
		for (size_t j = 0; j != col; ++j)
		{
			if (temp[j] < minval)
			{
				temp[j] = minval;
			}
			if (temp[j] > maxval)
			{
				temp[j] = maxval;
			}
		}
	}
}
void matColorTransfer(Mat& src, Mat& dst) {

	Mat labsrc, labdst;
	cvtColor(src, labsrc, COLOR_BGR2Lab);
	cvtColor(dst, labdst, COLOR_BGR2Lab);
	labsrc.convertTo(labsrc, CV_32FC3);
	labdst.convertTo(labdst, CV_32FC3);
	//计算三个通道的均值与方差
	Scalar meansrc, stdsrc, meandst, stddst;
	meanStdDev(labsrc, meansrc, stdsrc);
	meanStdDev(labdst, meandst, stddst);
	//三通道分离
	vector<Mat> srcsplit, dstsplit;
	split(labsrc, srcsplit);
	split(labdst, dstsplit);
	//每个通道减去均值
	dstsplit[0] -= meandst[0];
	dstsplit[1] -= meandst[1];
	dstsplit[2] -= meandst[2];
	//每个通道缩放
	dstsplit[0] = stddst[0] / stdsrc[0] * dstsplit[0];
	dstsplit[1] = stddst[1] / stdsrc[0] * dstsplit[1];
	dstsplit[2] = stddst[2] / stdsrc[0] * dstsplit[2];
	//加上源图像的均值
	dstsplit[0] += meansrc[0];
	dstsplit[1] += meansrc[1];
	dstsplit[2] += meansrc[2];
	//控制溢出
	clip(dstsplit[0], 0.0f, 255.0f);
	clip(dstsplit[1], 0.0f, 255.0f);
	clip(dstsplit[2], 0.0f, 255.0f);
	//转换为单字节单通道
	dstsplit[0].convertTo(dstsplit[0], CV_8UC1);
	dstsplit[1].convertTo(dstsplit[1], CV_8UC1);
	dstsplit[2].convertTo(dstsplit[2], CV_8UC1);
	//合并每个通道
	merge(dstsplit, dst);
	//从lab空间转换到RGB空间
	cvtColor(dst, dst, COLOR_Lab2BGR);
}
