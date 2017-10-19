#include "colorSpaceConversion.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
using namespace cv;
int main(int argv, char** argc) {
	String file_src = "../../image/target01.jpg";
	String file_ref = "../../image/reference01.jpg";
	Mat m_s, s_s, m_r, s_r;
	Mat dst, dst1,res;
	Mat src = imread(file_src);
	Mat ref = imread(file_ref);
	bgrTlab(src, dst, m_s, s_s);
	bgrTlab(ref, dst1, m_r, s_r);
	colorTransfer(dst, dst1, m_s, s_s, m_r, s_r);
	labtbgr(dst, res);
	imshow("res", res);
	waitKey(0);
	return 0;
}