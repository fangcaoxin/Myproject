
#include "core.h"
#include <opencv2/ximgproc.hpp>
/**dehaze: Kaiming He et al., Single image haze removal using dark channel prior
* dark channel prior(DCP)
* dehazeMY: improved underwater DCP*/
using namespace cv::ximgproc;
void dehaze(Mat& input, Mat& recover);
void dehazeDC(Mat image, Mat &dehaze);
void dehazeMY(Mat image, Mat &mydehaze);
void dehazeByBright(Mat& src, Mat& dst);