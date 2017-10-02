#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;
using namespace std;

void EMSegmetation(Mat& img,Mat& label,int pixel_num,int num);