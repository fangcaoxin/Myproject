#pragma once
#include "core.h"
#include <opencv2/ximgproc.hpp>

using namespace cv::ximgproc;
void dehaze(Mat& recover, Mat& input);