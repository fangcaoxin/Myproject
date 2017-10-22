
#include "core.h"
#include <opencv2/ximgproc.hpp>

using namespace cv::ximgproc;
void dehaze(Mat& input, Mat& recover);
void dehazeDC(Mat image, Mat &dehaze);
void dehazeMY(Mat image, Mat &mydehaze);