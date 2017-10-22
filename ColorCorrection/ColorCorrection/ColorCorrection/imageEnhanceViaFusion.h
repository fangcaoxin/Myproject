#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
using namespace cv;
using namespace std;

void Scliency(Mat& src, Mat& dst);
void LaplacianConstrast(Mat& src, Mat& dst);
void LocalConstrast(Mat& src, Mat& dst);
void Exposedness(Mat& src, Mat& dst);
void LaplacianPyramid(Mat& src, vector<Mat>& pyramids, int level);
void PyramidReconstruct(vector<Mat>& pyramid, Mat& dst);
void GaussianPyramid(Mat& src, vector<Mat>& pyramid, int level);
void SimplestColorBalance(Mat src, Mat& dst, int percent);