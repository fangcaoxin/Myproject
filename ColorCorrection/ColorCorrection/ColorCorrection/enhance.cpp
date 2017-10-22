#include "enhance.h"
#include <opencv2/highgui/highgui.hpp>

static void calWieght(Mat& img, Mat& L, Mat& dst) {
	divide(L, Scalar(255), L);
	L.convertTo(L, CV_32F);
	Mat WL, WC, WS, WE;
	LaplacianConstrast(L, WL);
	WL.convertTo(WL, L.type());

	LocalConstrast(L, WC);
	WC.convertTo(WC, L.type());

	Scliency(img, WS);
	WS.convertTo(WS, L.type());

	Exposedness(L, WE);
	WE.convertTo(WE, L.type());

	Mat weight = WL.clone();
	add(weight, WC, weight);
	add(weight, WS, weight);
	add(weight, WE, weight);
	dst = weight;
}

static void applyCLAHE(Mat& img, Mat& L, vector<Mat>& result) {
	Ptr<CLAHE> clahe= createCLAHE();
	clahe->setClipLimit(2.0);
	Mat L2, LabIm2,img2;
	clahe->apply(L, L2);
	vector<Mat> lab;
	split(img, lab);
	vector<Mat> tmp;
	tmp.push_back(L2);
	tmp.push_back(lab[1]);
	tmp.push_back(lab[2]);
	cv::merge(tmp, LabIm2);
	cvtColor(LabIm2, img2, CV_Lab2BGR);
	result.push_back(img2);
	result.push_back(L2);

}
void enhance(Mat src, Mat& dst) {
	Mat img1;
	SimplestColorBalance(src, img1, 5);
	img1.convertTo(img1, CV_8UC3);
	imshow("colorBalance", img1);
	Mat LabIm1, L1;
	vector<Mat> result;
	cvtColor(img1, LabIm1, CV_BGR2Lab);
	extractChannel(LabIm1, L1, 0);
	applyCLAHE(LabIm1, L1, result);
	Mat img2 = result[0];
	Mat L2 = result[1];
	imshow("image after histogram", img2);
	Mat w1, w2;
	vector<Mat> weight1, weight2, bCnl1, gCnl1, rCnl1, bCnl2, gCnl2, rCnl2;
	calWieght(img1, L1, w1);
	calWieght(img2, L2, w2);

	Mat sumW;
	add(w1, w2, sumW);
	divide(w1, sumW, w1);
	divide(w2, sumW, w2);

	int level = 5;
	GaussianPyramid(w1, weight1, level);
	GaussianPyramid(w2, weight2, level);

	img1.convertTo(img1, CV_32F);
	img2.convertTo(img2, CV_32F);

	vector<Mat> bgr;
	split(img1, bgr);
	LaplacianPyramid(bgr[0], bCnl1, level);
	LaplacianPyramid(bgr[1], gCnl1, level);
	LaplacianPyramid(bgr[2], rCnl1, level);
	bgr.clear();

	split(img2, bgr);
	LaplacianPyramid(bgr[0], bCnl2, level);
	LaplacianPyramid(bgr[1], gCnl2, level);
	LaplacianPyramid(bgr[2], rCnl2, level);

	/*fusion process*/
	vector<Mat> bCnl(level);
	vector<Mat> gCnl(level);
	vector<Mat> rCnl(level);

	for (int i = 0; i < level; i++) {
		Mat cn;
		add(bCnl1[i].mul(weight1[i]), bCnl2[i].mul(weight2[i]), cn);
		bCnl[i] = cn.clone();
		add(gCnl1[i].mul(weight1[i]), gCnl2[i].mul(weight2[i]), cn);
		gCnl[i] = cn.clone();
		add(rCnl1[i].mul(weight1[i]), rCnl2[i].mul(weight2[i]), cn);
		rCnl[i] = cn.clone();
	}

	Mat bChannel, gChannel, rChannel;
	vector<Mat> channel_list;
	PyramidReconstruct(bCnl, bChannel);
	PyramidReconstruct(gCnl, gChannel);
	PyramidReconstruct(rCnl, rChannel);
	channel_list.push_back(bChannel);
	channel_list.push_back(gChannel);
	channel_list.push_back(rChannel);


	Mat fusion;
	merge(channel_list, fusion);
	fusion.convertTo(fusion, CV_8UC3);
	dst = fusion;
}