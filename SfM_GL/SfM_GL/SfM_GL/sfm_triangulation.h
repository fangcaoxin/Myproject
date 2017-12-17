#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ximgproc/slic.hpp>
#include <opencv2/video/tracking.hpp>
using namespace cv;
using namespace std;

/**
From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
*/
cv::Mat_<double> LinearLSTriangulation(cv::Point3d u,		//homogenous image point (u,v,1)
	cv::Matx34d P,		//camera 1 matrix
	cv::Point3d u1,		//homogenous image point in 2nd camera
	cv::Matx34d P1		//camera 2 matrix
);

#define EPSILON 0.00001
/**
From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
*/
cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d u,	//homogenous image point (u,v,1)
	cv::Matx34d P,			//camera 1 matrix
	cv::Point3d u1,			//homogenous image point in 2nd camera
	cv::Matx34d P1			//camera 2 matrix
);
