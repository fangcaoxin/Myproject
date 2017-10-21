#include "colorSpaceConversion.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
using namespace cv;
int main(int argv, char** argc) {
	String file_src = "../../image/target01.jpg";
	String file_ref = "../../image/reference01.jpg";
	Mat src = imread(file_src);
	Mat res = imread(file_ref);
	matColorTransfer(src, res);
	imshow("res", res);
	waitKey(0);
	return 0;
}