#include "utility.h"

void calcPyrLKflow(vector<Mat>& imageList_gray, vector<vector<Point2f>>& flow_points) {
	vector<Point2f> points[3];
	
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	const int MAX_COUNT = 500;
	Size subPixWinSize(10, 10), winSize(11, 11);
	int maxLevel = 4;

	vector<uchar> status;
	vector<float> err;
	

	cv::goodFeaturesToTrack(imageList_gray[0], points[0], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
	cv::cornerSubPix(imageList_gray[1], points[0], subPixWinSize, Size(-1, -1), termcrit);

	vector<Mat> oldImagePyr, newImagePyr, newImagePyr1;
	cv::buildOpticalFlowPyramid(imageList_gray[0], oldImagePyr, winSize, maxLevel, false);
	cv::buildOpticalFlowPyramid(imageList_gray[1], newImagePyr, winSize, maxLevel, false);
	cv::buildOpticalFlowPyramid(imageList_gray[2], newImagePyr1, winSize, maxLevel, false);
	cv::calcOpticalFlowPyrLK(oldImagePyr, newImagePyr, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
	cv::calcOpticalFlowPyrLK(oldImagePyr, newImagePyr1, points[0], points[2], status, err, winSize, 3, termcrit, 0, 0.001);
	std::vector<bool>filter_status(points[0].size(), true);
	std::vector<bool> filter_status1(points[0].size(), true);
	check_FB(oldImagePyr, newImagePyr, points[0], points[1], filter_status);
	check_NCC(imageList_gray[0], imageList_gray[1], points[0], points[1], filter_status);
	check_FB(oldImagePyr, newImagePyr1, points[0], points[2], filter_status1);
	check_NCC(imageList_gray[0], imageList_gray[2], points[0], points[2], filter_status1);
	vector<Point2f> points0 = points[0];
	size_t good_points_after_filter = filterPointsInVectors(filter_status, points[0], points[1], true);
	size_t good_points_after_filter1 = filterPointsInVectors(filter_status1, points0, points[2], true);
	vector<Mat>homo_list;

	Mat homoMat = findHomography(points[0], points[1], CV_RANSAC, 3.0);
	Mat homoMat1 = findHomography(points0, points[2], CV_RANSAC, 3.0);
	
	homo_list.push_back(homoMat);
	homo_list.push_back(homoMat1);
	
	flow_points.push_back(points[0]);
	flow_points.push_back(points[1]);
	flow_points.push_back(points[2]);
#ifdef DUMP
	size_t i, k;
	imageList_gray[0].copyTo(flow);
	for (i = k = 0; i < points[0].size(); i++)
	{
		circle(flow, points[0][i], 3, Scalar(0, 0, 255), -1, 8);
	}
	for (i = k = 0; i < points[1].size(); i++)
	{
		circle(flow, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
	}
	imshow("flow", flow);
#endif //DUMP
	
}