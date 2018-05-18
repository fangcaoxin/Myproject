#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <opencv2/ml.hpp>

void calcDarkChannel(cv::Mat& darkChannel,cv::Mat& brightChannel, cv::Mat& input, int radius);
void calcAirLight(cv::Mat& darkChannel, cv::Mat& input, double A[]);
void calcTransmission(cv::Mat& transmission, cv::Mat& input, double A[], int radius);
void calcRecover(cv::Mat& result, cv::Mat& input, cv::Mat& transmission, double A[]);
void calcDarkChannelByIllumi(cv::Mat& darkchannel, cv::Mat& input, int radius);
void calcBrightBrightChannel(cv::Mat& src, cv::Mat& bbchannel, int radius);
double evaluationScore_UCIQUE(cv::Mat& src);
/** paper: Fast Haze Removal for Nighttime Image UsingMaximumReflectance Prior
* cvpr 2017*/
void calcMaxReflectChannelColorMap(cv::Mat& src, cv::Mat& dst, int radius);
