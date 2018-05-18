
#include "core.h"
#include <opencv2/ximgproc.hpp>
/**dehaze: Kaiming He et al., Single image haze removal using dark channel prior
* dark channel prior(DCP)
* dehazeMY: improved underwater DCP*/
//using namespace cv::ximgproc;
void dehaze(cv::Mat& input, cv::Mat& recover);
void dehazeDC(cv::Mat image, cv::Mat &dehaze);
void dehazeMY(cv::Mat image, cv::Mat &mydehaze);
void dehazeByBright(cv::Mat& src, cv::Mat& dst);