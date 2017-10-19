#pragma once
#include "Color3d.h"
#include <opencv2/core/core.hpp>

using namespace cv;

void bgrTlab(Mat& src, Mat& dst, Color3d& m, Color3d& s);
void labtbgr(Mat& src, Mat& dst);
void colorTransfer(Mat& src, Mat& ref, Color3d& m_src, Color3d& s_src, Color3d& m_ref, Color3d& s_ref);