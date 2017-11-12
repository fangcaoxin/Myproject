#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
using namespace cv;
using namespace std;
struct CloudPoint {
	Point3d pt;
	vector<int> imgpt_for_img;
	double reprojection_error;
};

/**draw the dense flow on the color image
  *step to set output flow step*/
void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
	double, const Scalar& color);

/**caculate the most direction of dense flow*/
void calcOptDirection(const Mat& flow, int& oritention, Point2f& aver);
void check_FB(const std::vector<Mat>& oldImagePyr, const std::vector<Mat>& newImagePyr,
	const std::vector<Point2f>& oldPoints, const std::vector<Point2f>& newPoints, std::vector<bool>& status);
void check_NCC(const Mat& oldImage, const Mat& newImage,
	const std::vector<Point2f>& oldPoints, const std::vector<Point2f>& newPoints, std::vector<bool>& status);
template<typename T>
T getMedian(const std::vector<T>& values);
template<typename T>
T getMedianAndDoPartition(std::vector<T>& values);
Mat getPatch(Mat image, Size patch_size, Point2f patch_center);

size_t filterPointsInVectors(std::vector<bool>& status, std::vector<Point2f>& vec1, std::vector<Point2f>& vec2, bool goodValue);
/* image_list: gray image list(3 frames),output flow.the flow between 0 and 1, 0 and 2*/
void calcDenseFlow(vector<Mat>& image_list, vector<Mat>& flow);
/**refine the flow according to direction, length and etc.*/
void refineFlow(Mat& flow1, Mat& flow2, int radius);

/**refine the flow according block,if in the block,
* the number of flow is over some threshold
* the block is filled of flow as the same value as neighours*/
void refineFlowTwice(Mat& flow1, int radius);
/**to caculate optical flow using Lucas-Kanade method*/
void calcPyrLKflow(vector<Mat>& imageList_gray, Mat& object_area,vector<Mat>& camera_motion);
/**show the connected area label on the image*/
void showAreaLabel(Mat& image, Mat& labels, Mat& centroids,int size);
void contourSobel(Mat& image_gray, const vector<Vec4i>& hierarchy,  vector<float>& probs,vector<vector<Point>>& contour_points);

void diffFiveFrames(vector<Mat>& image_list_gray, vector<Mat>& diff);
void diffWB(vector<Mat>& diff, vector<Mat>& diff_wb, int threshold);
void snowMaskbyDiffWB(vector<Mat>& diff_wb, Mat& mask);
void temporalLikelihood(vector<Mat>& diff, vector<Mat>& diff_wb, Mat& temporal);
void modelError(vector<Mat>& diff_wb, vector<Mat>& diff, Mat& sigma);

void FrameRelativeDiff(vector<Mat>& image_list_gray, vector<Mat>& diff);
void FrameRelativeDiffBaseCameraMotion(vector<Mat>& image_list_gray, vector<Mat>& diff, vector<Mat>& camera_motion);
void diffByThreshold(vector<Mat>& diff, vector<Mat>& diff_wb, int threshold_wb);
void diffByPreNext(vector<Mat>& diff_wb, Mat& diff_output);
void neighbourBlockMatching(Mat& labels, Mat& stats, Mat& centroids, vector<Mat>& image_list_gray,vector<int>& valid_label);
void spatialFilter(Mat& labels, Mat& diff_wb, vector<int>& valid_label);
double modelError(vector<Mat>& diff, Mat& diff_out);
void neighbourBlockDiff(Mat& labels, Mat& stats, Mat& centroids, vector<Mat>& image_list_gray, vector<int>& valid_label,double model_error);
void showLabelImg(Mat& label);
void showMaskImg(Mat& label,Mat& show_img);
void getMaskFromValidLabels(Mat& mask, vector<int>& valid_labels);
void getMaskFromProbs(Mat& mask, vector<float>& prob1, vector<float>& prob2);
void nearNeighourSimilarity(Mat& image, Mat& stats, vector<float>& probs_similar);
void rgbStdDev(Mat& image, Mat& labels,Mat& stats, Mat& normalize_std, int size);
void getMaskFromStd(Mat& mask, Mat& normalize_std);
void imageListCompensation(vector<Mat>& image_list, vector<Mat>& image_list_compensation, vector<Mat>& camera_motion);
void imageListGrayCompensation(vector<Mat>& image_list_gray, vector<Mat>& image_list_gray_compensation, vector<Mat>& camera_motion);
void imageClosing(Mat& src, Mat& output, int kenel_size);
bool FindCameraMatrices(const Mat& K,
	const Mat& Kinv,
	const Mat& F,
	Matx34d& P,
	Matx34d& P1,
	const Mat& discoeff,
	const vector<KeyPoint>& imgpts1,
	const vector<KeyPoint>& imgpts2,
	vector<KeyPoint>& imgpts1_good,
	vector<KeyPoint>& imgpts2_good,
	vector<DMatch>& matches,
	vector<CloudPoint>& outCloud);
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
double TriangulatePoints(const vector<KeyPoint>& pt_set1,
	const vector<KeyPoint>& pt_set2,
	const Mat& K,
	const Mat& Kinv,
	const Matx34d& P,
	const Matx34d& P1,
	vector<CloudPoint>& pointcloud,
	vector<KeyPoint>& correspImg1Pt,
	const Mat& distcoeff);
bool TestTriangulation(const vector<CloudPoint>& pcloud, const Matx34d& P, vector<uchar>& status);
std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts);