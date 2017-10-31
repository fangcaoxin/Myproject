#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
/** paper 
* Codruta O, Cosmin et al,
* Locally Adaptive Color Correction for Underwater Image Dehazing and Matching,
* cvpr workshop 2017
*/

using namespace cv;
using namespace std;

void localColorCorrection(Mat& src, Mat& dst);