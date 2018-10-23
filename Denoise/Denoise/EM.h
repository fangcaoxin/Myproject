#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "core.h"
#include <opencv2/ml.hpp>


void EMSegmetation(Mat& img,Mat& label,int pixel_num,int num);
void EMSegmetationSamples(Mat& img, Mat& samples,vector<int>& valid_labels,vector<float>&probs, int num);
void createSamples(Mat& img, Mat& stats, Mat& labels, Mat& samples);
void createSamplesFrames(vector<Mat>& image_list_gray, Mat& label, vector<Mat>& samples, Mat& stats, int size);
void EMModel(vector<Mat>& samples, vector<Ptr<EM>>& em_models);