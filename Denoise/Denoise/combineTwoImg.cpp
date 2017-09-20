#include "core.h"

void combineTwoImg(Mat src1, Mat src2, Mat& dst) {
	Rect rect1(0, 0, src1.cols, src1.rows);
	Rect rect2(src1.cols, 0, src2.cols, src2.rows);
	src1.copyTo(dst(rect1));
	//dst(rect1) = src1;
	src2.copyTo(dst(rect2));
	//dst(rect2) = src2;
}