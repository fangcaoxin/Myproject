#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;
using namespace std;

void EMSegmetation(Mat& img,Mat& label,int pixel_num,int num);
void EMSegmetationSamples(Mat& img, Mat& samples,vector<int>& valid_labels,vector<float>&probs, int num);
void createSamples(Mat& img, Mat& stats, Mat& labels, Mat& samples);