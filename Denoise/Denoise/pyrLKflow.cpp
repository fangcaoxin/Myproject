#include "utility.h"
static Point2f getAverage(const std::vector<Point2f>& values);
void calcPyrLKflow(vector<Mat>& imageList_gray, vector<Point2f>& camera_motion) {
	vector<Point2f> points[3];
	
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	const int MAX_COUNT = 500;
	Size subPixWinSize(10, 10), winSize(11, 11);
	int maxLevel = 4;

	vector<uchar> status;
	vector<float> err;
	

	cv::goodFeaturesToTrack(imageList_gray[1], points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
	cv::cornerSubPix(imageList_gray[1], points[1], subPixWinSize, Size(-1, -1), termcrit);

	vector<Mat> oldImagePyr, newImagePyr, newImagePyr1;
	cv::buildOpticalFlowPyramid(imageList_gray[1], oldImagePyr, winSize, maxLevel, false);
	cv::buildOpticalFlowPyramid(imageList_gray[0], newImagePyr, winSize, maxLevel, false);
	cv::buildOpticalFlowPyramid(imageList_gray[2], newImagePyr1, winSize, maxLevel, false);
	cv::calcOpticalFlowPyrLK(oldImagePyr, newImagePyr, points[1], points[0], status, err, winSize, 3, termcrit, 0, 0.001);
	cv::calcOpticalFlowPyrLK(oldImagePyr, newImagePyr1, points[1], points[2], status, err, winSize, 3, termcrit, 0, 0.001);
	std::vector<bool>filter_status(points[1].size(), true);
	std::vector<bool> filter_status1(points[1].size(), true);
	check_FB(oldImagePyr, newImagePyr, points[1], points[0], filter_status);
	check_NCC(imageList_gray[1], imageList_gray[0], points[1], points[0], filter_status);
	check_FB(oldImagePyr, newImagePyr1, points[1], points[2], filter_status1);
	check_NCC(imageList_gray[1], imageList_gray[2], points[1], points[2], filter_status1);
	vector<Point2f> points1 = points[1];
	size_t good_points_after_filter = filterPointsInVectors(filter_status, points[1], points[0], true);
	size_t good_points_after_filter1 = filterPointsInVectors(filter_status1, points1, points[2], true);
	vector<Mat>homo_list;

	Point2f cur_pre_vector = getAverage(points[0])-getAverage(points[1]);
	Point2f cur_next_vector = getAverage(points[2]) - getAverage(points1);

	//Mat fundamentalMat1 = findFundamentalMat(points[0], points[1], CV_RANSAC, 3.0);
	
	//Mat fundamentalMat2 = findFundamentalMat(points0, points[2], CV_RANSAC, 3.0);
	
	camera_motion.push_back(cur_pre_vector);
	camera_motion.push_back(cur_next_vector);
	cout << "camera motion " << cur_pre_vector << " " << cur_next_vector << endl;
	//homo_list.push_back(homoMat);
	//homo_list.push_back(homoMat1);
	
	/*flow_points.push_back(points[0]);
	flow_points.push_back(points[1]);
	flow_points.push_back(points[2]);*/
#define DUMP
#ifdef DUMP
	Mat flow;
	size_t i, k;
	imageList_gray[1].copyTo(flow);
	cvtColor(flow, flow, CV_GRAY2BGR);
	for (i = k = 0; i < points1.size(); i++)
	{
		circle(flow, points1[i], 1, Scalar(0, 0, 255), -1, 8);
	}
	for (i = k = 0; i < points[2].size(); i++)
	{
		circle(flow, points[2][i], 2, Scalar(0, 255, 0), -1, 8);
	}
	imshow("flow", flow);
#endif //DUMP
	
}

static Point2f getAverage(const std::vector<Point2f>& values) {

	size_t size = values.size();
	if (size == 0) return 0;
	Point2f sum = {0.,0.};
	for (int i = 0; i < size; i++) {
		sum += values[i];
	}
	Point2f res(sum.x / size, sum.y / size);
	return res;
}