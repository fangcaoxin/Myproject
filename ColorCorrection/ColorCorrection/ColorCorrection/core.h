#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <opencv2/ml.hpp>
using namespace cv::ml;
using namespace cv;
using namespace std;



void calcDarkChannel(Mat& darkChannel,Mat& brightChannel, Mat& input, int radius);
void calcAirLight(Mat& darkChannel, Mat& input, double A[]);
void calcTransmission(Mat& transmission, Mat& input, double A[], int radius);
void calcRecover(Mat& result, Mat& input, Mat& transmission, double A[]);
