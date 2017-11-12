#include "utility.h"
#include <opencv2/highgui/highgui.hpp>
static Point2f getAverage(const std::vector<Point2f>& values);
static void checkValidArea(Mat& valid_area, vector<Point2f>& points, vector<bool>& filter_status);
static void drawCorrespondance(Mat& img, Mat& img1, vector<Point2f>& points, vector<Point2f>& points1, Mat& output);
#define DEPTHMAP
void calcPyrLKflow(vector<Mat>& imageList_gray,Mat& object_area, vector<Mat>& camera_motion) {
	vector<Point2f> points[3];
	
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	const int MAX_COUNT = 500;
	Size subPixWinSize(10, 10), winSize(11, 11);
	int maxLevel = 4;

	vector<uchar> status;
	vector<float> err;
	

	cv::goodFeaturesToTrack(imageList_gray[1], points[1], MAX_COUNT, 0.01, 10, object_area, 3, 0, 0.04);
	cv::cornerSubPix(imageList_gray[1], points[1], subPixWinSize, Size(-1, -1), termcrit);

	vector<Mat> oldImagePyr, newImagePyr, newImagePyr1;
	cv::buildOpticalFlowPyramid(imageList_gray[1], oldImagePyr, winSize, maxLevel, false);
	cv::buildOpticalFlowPyramid(imageList_gray[0], newImagePyr, winSize, maxLevel, false);
	cv::buildOpticalFlowPyramid(imageList_gray[2], newImagePyr1, winSize, maxLevel, false);
	cv::calcOpticalFlowPyrLK(oldImagePyr, newImagePyr, points[1], points[0], status, err, winSize, 3, termcrit, 0, 0.001);
	cv::calcOpticalFlowPyrLK(oldImagePyr, newImagePyr1, points[1], points[2], status, err, winSize, 3, termcrit, 0, 0.001);
	
	std::vector<bool>filter_status(points[1].size(), true);
	std::vector<bool> filter_status1(points[1].size(), true);
	//checkValidArea(object_area, points[1], filter_status);
	check_FB(oldImagePyr, newImagePyr, points[1], points[0], filter_status);
	check_NCC(imageList_gray[1], imageList_gray[0], points[1], points[0], filter_status);
	//checkValidArea(object_area, points[1], filter_status1);
	check_FB(oldImagePyr, newImagePyr1, points[1], points[2], filter_status1);
	check_NCC(imageList_gray[1], imageList_gray[2], points[1], points[2], filter_status1);
	vector<Point2f> points1 = points[1];

	size_t good_points_after_filter = filterPointsInVectors(filter_status, points[1], points[0], true);
	size_t good_points_after_filter1 = filterPointsInVectors(filter_status1, points1, points[2], true);
	

	/*Point2f cur_pre_vector = getAverage(points[0])-getAverage(points[1]);
	Point2f cur_next_vector = getAverage(points[2]) - getAverage(points1);*/
#ifdef DEPTHMAP
	Mat F = findFundamentalMat(points[0], points[1], FM_RANSAC);
	double f = 1057.14;
	double cx = 640;
	double cy = 480;
	const double K_array[3][3] = {
		{f,0,cx},
		{0,f,cy},
		{0,0,1}
	};
	const size_t bufsize = sizeof(double) * 3 * 3;
	Mat K(3,3,CV_32FC1,Scalar(0));
	memcpy(K.data, &K_array[0][0], bufsize);
	
	/*Matx33d K = Matx33d(f, 0, cx,
		0, f, cy,
		0, 0, 1);*/
	Mat E = K.t()*F*K;
	SVD svd(E);
	Matx33d W(0, -1, 0,
		1, 0, 0,
		0, 1, 1);
	Matx33d Winv(0, 1, 0,
		-1, 0, 0,
		0, 0, 1);
	Mat_<double> R = svd.u*Mat(W)*svd.vt;
	Mat_<double> t = svd.u.col(2);
	Matx34d P1 = Matx34d(R(0, 0), R(0, 1), R(0, 2), t(0),
		R(1, 0), R(1, 1), R(1, 2), t(1),
		R(2, 0), R(2, 1), R(2, 2), t(2));

#else 

	Mat homo_10 = findHomography(points[0], points[1], CV_RANSAC, 3.0);
	Mat homo_12 = findHomography(points[2], points1, CV_RANSAC, 3.0);
	camera_motion.push_back(homo_10);
	camera_motion.push_back(homo_12);
	//camera_motion.push_back(cur_pre_vector);
	//camera_motion.push_back(cur_next_vector);
	cout << "camera motion " << homo_10 << " " << homo_12 << endl;
#endif //DEPTHMAP
	//homo_list.push_back(homoMat);
	//homo_list.push_back(homoMat1);
	
	/*flow_points.push_back(points[0]);
	flow_points.push_back(points[1]);
	flow_points.push_back(points[2]);*/
//#define DUMP
#ifdef DUMP
	Mat flow,flow1,output;
	size_t i, k;
	imageList_gray[1].copyTo(flow);
	imageList_gray[2].copyTo(flow1);
	//cvtColor(flow, flow, CV_GRAY2BGR);
	drawCorrespondance(flow, flow1, points1, points[2], output);
	
	//for (i = k = 0; i < points1.size(); i++)
	//{
	//	circle(flow, points1[i], 1, Scalar(0, 0, 255), -1, 8);
	//}
	//for (i = k = 0; i < points[2].size(); i++)
	//{
	//	circle(flow, points[2][i], 1, Scalar(0, 255, 0), -1, 8);
	//}
	imwrite("match_image.jpg", output);
	imshow("flow", output);
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

static void checkValidArea(Mat& valid_area, vector<Point2f>& points, vector<bool>& status) {
	if (status.empty()) {
		status = std::vector<bool>(points.size(), true);
	}
	for (int i = 0; i < points.size(); i++) {
		Point point_int;
		point_int.x = floor(points[i].x)<0?0: floor(points[i].x);
		point_int.y= floor(points[i].y)<0?0: floor(points[i].y);
		//cout << "point_int " << point_int << endl;
		status[i] = status[i] && valid_area.at<uchar>(point_int) == 255;
		
	}
}

static void drawCorrespondance(Mat& img, Mat& img1, vector<Point2f>& points, vector<Point2f>& points1, Mat& output) {
	hconcat(img, img1, output);
	cvtColor(output, output, CV_GRAY2BGR);
	int width = img.cols;
	int height = img.rows;
	Point move(width, 0);
	for (int i = 0; i < points.size(); i++) {
		circle(output, (Point)points[i], 2, Scalar(0, 0, 255));
		circle(output, (Point)points1[i] + move, 2,Scalar(255, 0, 0));
		line(output, points[i], (Point)points1[i] + move,Scalar(255,255,0), 0.2, 8, 0);
	}
	
}

