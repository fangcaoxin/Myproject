#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
using namespace cv;
using namespace std;

void darkChannelFrames(const vector<Mat> &imgList, int frameNum, Mat& ouput,const vector<Mat>& flow);
void directionHist(Mat& flow, int win_size);

void calcDarkChannel(Mat& darkChannel,Mat& brightChannel, Mat& input, int radius);
void calcAirLight(Mat& darkChannel, Mat& input, double A[]);
void calcTransmission(Mat& transmission, Mat& input, double A[], int radius);
void calcRecover(Mat& result, Mat& input, Mat& transmission, double A[]);
void darkFrames(const vector<Mat>& imagelist, Mat& output,const vector<Mat>& homography);
void blockAverage(Mat& darkChannel, Mat& mask, int radius);

void darkChannelMask(Mat& darkchannel, Mat& mask);
void darkFramesByMask(vector<Mat>& imagelist, Mat& output, Mat& Mask);
/** linerConstraint is from papaer to vertify whether the diff of two frame is a ratio of background value*/
void linerConstraint(Mat& gray_pre, Mat& gray,Mat& output);
void getKeyPoints(Mat& flow1, Mat flow2, vector<KeyPoint>& kp1, vector<KeyPoint>& kp2);
void shapeFilter(Mat& diff_wb, Mat& labels, Mat& stats,int size,vector<int>& valid_labels);
void distributeFilter(Mat& diff_wb, Mat& labels,Mat& stats, Mat& gray,vector<int>& valid_labels,vector<int>& valid_labels1);
void maskRefinement(Mat& diff_wb, Mat& labels, Mat& gray, vector<int>& valid_labels2);
void medianFramesByMask(Mat& image, Mat& stats, vector<int>& valid_labels);
void bayesianEstimation(Mat& image, Mat& labels_init, Mat& labels_estimation,int seg_num, int iter, int radius);
int sumAreaByRadius(vector<Mat>& diff_wb, Mat& sum, int radius);
void labelInitByDiff(vector<Mat>& diff_wb, Mat& label_init);
void labelInitByRedDarkChannel(Mat& red_dark, Mat& label_init);
