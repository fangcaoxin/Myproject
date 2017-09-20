#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
using namespace cv;
using namespace std;

void darkChannelFrames(const vector<Mat> &imgList, int frameNum, Mat& ouput,const vector<Mat>& flow);
void combineTwoImg(Mat src1, Mat src2, Mat& dst);
void directionHist(Mat& flow, int win_size);
void check_FB(const std::vector<Mat>& oldImagePyr, const std::vector<Mat>& newImagePyr,
	const std::vector<Point2f>& oldPoints, const std::vector<Point2f>& newPoints, std::vector<bool>& status);
void check_NCC(const Mat& oldImage, const Mat& newImage,
	const std::vector<Point2f>& oldPoints, const std::vector<Point2f>& newPoints, std::vector<bool>& status);
template<typename T>
T getMedian(const std::vector<T>& values);
template<typename T>
T getMedianAndDoPartition(std::vector<T>& values);
Mat getPatch(Mat image, Size patch_size, Point2f patch_center);
void calcDarkChannel(Mat& darkChannel,Mat& brightChannel, Mat& input, int radius);
void calcAirLight(Mat& darkChannel, Mat& input, double A[]);
void calcTransmission(Mat& transmission, Mat& input, double A[], int radius);
void calcRecover(Mat& result, Mat& input, Mat& transmission, double A[]);
void darkFrames(const vector<Mat>& imagelist, Mat& output,const vector<Mat>& homography);
void blockAverage(Mat& darkChannel, Mat& mask, int radius);
void refineFlow(Mat& flow1, Mat& flow2,int radius);
void refineFlowTwice(Mat& flow1, int radius);
void darkChannelMask(Mat& darkchannel, Mat& mask);
void darkFramesByMask(vector<Mat>& imagelist, Mat& output, Mat& Mask);
void linerConstraint(Mat& gray_pre, Mat& gray,Mat& output);
void getKeyPoints(Mat& flow1, Mat flow2, vector<KeyPoint>& kp1, vector<KeyPoint>& kp2);
void shapeFilter(Mat& diff_wb, Mat& labels, Mat& stats,int size,vector<int>& valid_labels);
void distributeFilter(Mat& diff_wb, Mat& labels,Mat& stats, Mat& gray,vector<int>& valid_labels,vector<int>& valid_labels1);
void maskRefinement(Mat& diff_wb, Mat& labels, Mat& gray, vector<int>& valid_labels2);