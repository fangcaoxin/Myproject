#pragma once

#include <opencv2/core/core.hpp>

using namespace cv;

void bgrTlab(Mat& src, Mat& dst, Mat& m, Mat& s);
void labtbgr(Mat& src, Mat& dst);
void colorTransfer(Mat& src, Mat& ref, Mat& m_src, Mat& s_src, Mat& m_ref, Mat& s_ref);
void matColorTransfer(Mat& src, Mat& dst);