#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ximgproc/edge_filter.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/optflow.hpp>
#include <iostream>
#include "core.h"
#include "gms_matcher.h"
#include "Header.h"
#include "math.h"
//#define FLOW
//#define MOG
//#define DEHAZE

using namespace cv::ximgproc;
//using namespace cv::optflow;
static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
	double, const Scalar& color)
{
	for (int y = 0; y < cflowmap.rows; y += step)
		for (int x = 0; x < cflowmap.cols; x += step)
		{
			const Point2f& fxy = flow.at<Point2f>(y, x);
			if (norm(fxy) > 0) {
				line(cflowmap, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
					color);
				circle(cflowmap, Point(x, y), 1, color, -1);
			}
		}
}

static void calcOptDirection(const Mat& flow, int& oritention,Point2f& aver) {
	struct val {
		int ori;
		Point2f point;
	};
	struct val *histval = NULL;
	if (histval == NULL) {
		histval = (struct val*)malloc(360 * sizeof(struct val));
	}
	for (int i = 0; i < 360; i++) {
		histval[i] = { 0 };
	}
	for (int y = 0; y < flow.rows; y++) {
		for (int x = 0; x < flow.cols; x++) {
			const Point2f& fxy = flow.at<Point2f>(y, x);
			int cur_ori = (int)fastAtan2(fxy.y, fxy.x);
			cur_ori = cur_ori > 359 ? 0 : cur_ori;
			histval[cur_ori].ori++;
			histval[cur_ori].point.x += fxy.x;
			histval[cur_ori].point.y += fxy.y;
		}
	}
	int max = 0;
	int ori = 0;
	for (int i = 0; i < 360; i++) {
		if (histval[i].ori > max) {
			max = histval[i].ori;
			ori = i;
		}
	}
	aver.x = histval[ori].point.x / max;
	aver.y = histval[ori].point.y / max;
	oritention = ori;
	if (histval != NULL) {
		free(histval);
		histval = NULL;
	}
	
}

template<typename T>
size_t filterPointsInVectors(std::vector<T>& status, std::vector<Point2f>& vec1, std::vector<Point2f>& vec2, T goodValue)
{
	CV_DbgAssert(status.size() == vec1.size() && status.size() == vec2.size());

	size_t first_bad_idx = 0;
	while (first_bad_idx < status.size())
	{
		if (status[first_bad_idx] != goodValue)
			break;
		first_bad_idx++;
	}

	if (first_bad_idx >= status.size())
		return first_bad_idx;

	for (size_t i = first_bad_idx + 1; i < status.size(); i++)
	{
		if (status[i] != goodValue)
			continue;

		status[first_bad_idx] = goodValue;
		vec1[first_bad_idx] = vec1[i];
		vec2[first_bad_idx] = vec2[i];
		first_bad_idx++;
	}
	vec1.erase(vec1.begin() + first_bad_idx, vec1.end());
	vec2.erase(vec2.begin() + first_bad_idx, vec2.end());
	status.erase(status.begin() + first_bad_idx, status.end());

	return first_bad_idx;
}

void GmsMatch(Mat &img1, Mat &img2) {
	vector<KeyPoint> kp1, kp2;
	//Mat d1, d2;
	vector<DMatch> matches_all, matches_gms;
   

	//Ptr<ORB> orb = ORB::create(10000);
	//orb->setFastThreshold(0);
	//orb->detectAndCompute(img1, Mat(), kp1, d1);
	//orb->detectAndCompute(img2, Mat(), kp2, d2);

#ifdef USE_GPU
	GpuMat gd1(d1), gd2(d2);
	Ptr<cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(NORM_HAMMING);
	matcher->match(gd1, gd2, matches_all);
#else
	//BFMatcher matcher(NORM_HAMMING);
	//matcher.match(d1, d2, matches_all);
#endif

	// GMS filter
	int num_inliers = 0;
	std::vector<bool> vbInliers;
	gms_matcher gms(kp1, img1.size(), kp2, img2.size(), matches_all);
	num_inliers = gms.GetInlierMask(vbInliers, false, false);

	cout << "Get total " << num_inliers << " matches." << endl;

	// draw matches
	for (size_t i = 0; i < vbInliers.size(); ++i)
	{
		if (vbInliers[i] == true)
		{
			matches_gms.push_back(matches_all[i]);
		}
	}

	Mat show = DrawInlier(img1, img2, kp1, kp2, matches_gms, 1);
	imshow("show", show);
	waitKey();
}

Ptr<BackgroundSubtractor> pMOG;

int main(int argc, char* argv[]) {
	//string saveFolder = "..//..//..//image//img_170721_01j//img_170721_01j_1";
	string saveFolder = "..//..//..//image//img_170724_02j//img_170724_02j_";
	string saveImage = "..//..//..//result//img_170724_02j//img_170724_02j_";
	int width = 1468; //1468
	int height = 1080; //1080
	Rect valid_rect(20, 5, width - 30, height-10);
	int width_input = valid_rect.width;
	int height_input = valid_rect.height;
	int beg_num = 11;
	int frame_num = 100;
	Mat backgroud;
	int frame_count = 0;
	Mat fgMaskMOG(height_input,width_input,CV_8UC1,Scalar(0));
	pMOG = createBackgroundSubtractorMOG2(5, 10, false);
	Ptr<FastFeatureDetector> fast = FastFeatureDetector::create();
	vector<Mat> image_list;
	vector<Mat> image_list_gray;
	
#ifdef FLOW
	vector<Point2f> points[3];
	vector<KeyPoint> keyPoints;
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	const int MAX_COUNT = 500;

	Size subPixWinSize(10, 10), winSize(11, 11);
	int maxLevel = 4;
	
	vector<Mat> imageList_gray;
	vector<Mat> flow_list;
	Mat flow, cflow, uflow, uflow1, img_edge;
#endif //FLOW
	int patchSize = 20;
	double A[3] = {0.};
	int count = 0;
	for (int i = beg_num; i < beg_num + frame_num; i = i + 1) {
		//string file_name = saveFolder + " (" + to_string(i) + ")" + ".jpg";//img_170721_01j_1 (1)
		string file_name = saveFolder + to_string(i) + ".jpg";
		string save_name = saveImage + to_string(i) + "_0918out.jpg";
		cout << "file name :" << file_name << endl;
		Mat cur = imread(file_name);
#define CONNECTED
#ifdef CONNECTED
		Mat input_resize, cur_gray;
		//cur(valid_rect).copyTo(image);
		resize(cur(valid_rect), input_resize, Size(width_input, height_input));
		//image_list.push_back(cur(valid_rect));
		image_list.push_back(input_resize);
		cvtColor(input_resize, cur_gray, CV_BGR2GRAY);
		image_list_gray.push_back(cur_gray);
		count++;

		if (count < 3) {
			continue;
		}
		else {
			Mat diff = image_list_gray[1] - image_list_gray[0];
			Mat diff_wb, labels, stats, centroids;
			Mat output(height_input, width_input, CV_8UC3);
			threshold(diff, diff_wb, 15, 255, CV_THRESH_BINARY);
			int size=connectedComponentsWithStats(diff_wb, labels, stats, centroids, 8, 4);
			vector<int> valid_label1,valid_label2;
			shapeFilter(diff_wb, labels, stats, size,valid_label1);
			cout << "ShaperFilter finished" << endl;
			distributeFilter(diff_wb, labels, stats,image_list_gray[1],valid_label1, valid_label2);
			maskRefinement(diff_wb, labels, image_list_gray[1], valid_label2);
			darkFramesByMask(image_list, output, diff_wb);
			imshow("diff", diff_wb);
			imshow("output", output);
			imshow("origial", image_list[1]);
			waitKey(0);

		}
	}
		
#endif //CONNECTED

#ifdef DEHAZE
		resize(cur(valid_rect), input_resize, Size(width_input, height_input));
		cvtColor(input_resize, input_gray, CV_BGR2GRAY);
		Mat darkChannel(height_input, width_input, CV_8UC1), brightChannel(height_input, width_input, CV_8UC1);
		Mat refineDarkChannel(height_input, width_input, CV_8UC1);
		Mat transmission(height_input, width_input, CV_8UC1);
		Mat refineTransmission(height_input, width_input, CV_8UC1);
		Mat result(height_input, width_input, CV_8UC3);
		int radius = MIN(width_input, height_input)*0.02;
		calcDarkChannel(darkChannel, brightChannel, input_resize, radius);
		calcAirLight(brightChannel, input_resize, A);
		cout << "airlight is " << A[0] << " " << A[1] << " " << A[2] << endl;
		calcTransmission(transmission, input_resize, A, radius);
		//guidedFilter(input_resize, darkChannel, refineDarkChannel, 60, 1e-6);
		guidedFilter(input_resize, transmission, refineTransmission, 60, 1e-6);
		calcRecover(result, input_resize, refineTransmission, A);
		imshow("gray", input_gray);
		imshow("brightChannel", brightChannel);
		imshow("transmission", refineTransmission);
		imshow("result", result);
		//imshow("refineTransmission", refineTransmission);
		//imshow("refineDarkChannel", refineDarkChannel);
		waitKey(0);
		//imageList.push_back(cur_resize);
	}
#endif //DEHAZE
#ifdef FLOW
	Mat input_resize, cur_gray;
	//cur(valid_rect).copyTo(image);
	resize(cur(valid_rect), input_resize, Size(width_input, height_input));
	//image_list.push_back(cur(valid_rect));
	image_list.push_back(input_resize);
	cvtColor(input_resize, cur_gray, CV_BGR2GRAY);
	imageList_gray.push_back(cur_gray);

#endif //FLOW
#ifdef MOG
	image_list.push_back(cur(valid_rect));
	vector<Mat> mask_list;
	Mat output(height_input, width_input, CV_8UC3);
	pMOG->apply(cur(valid_rect), fgMaskMOG, 0.8);
	mask_list.push_back(fgMaskMOG);
	frame_count++;
	if (frame_count == 3) {
		darkFramesByMask(image_list, output, mask_list[0]);
		
		imshow("FG Mask", mask_list[0]);
		imshow("output", output);
		//imshow("background", backgroud);
		cv::waitKey(0);
		frame_count = 2;
		mask_list.erase(mask_list.begin(), mask_list.end() - 1);
		image_list.erase(image_list.begin());

	}
	//pMOG->getBackgroundImage(backgroud);
	
	}
#endif //MOG
		
	
#ifdef FLOW
	
	Mat output(valid_rect.height,valid_rect.width,CV_8UC3);
	Mat darkChannel(height_input,width_input,CV_8UC1), brightChannel(height_input, width_input, CV_8UC1);
	Mat darkChannel1(height_input, width_input, CV_8UC1), brightChannel1(height_input, width_input, CV_8UC1);
	if (image_list.size() < 3){
		continue;
	}
	else {
#define DENSEFLOW		
#ifdef DENSEFLOW
		//calcDarkChannel(darkChannel, brightChannel, image_list[0], 2);
	   // Mat diff0 = brightChannel - darkChannel + brightChannel - imageList_gray[0];
		//calcDarkChannel(darkChannel, brightChannel, image_list[1], 0);
		//Mat diff1 = brightChannel - darkChannel + brightChannel - imageList_gray[1];
		//calcDarkChannel(darkChannel, brightChannel, image_list[2], 0);	
		//Mat diff2 = brightChannel - darkChannel + brightChannel - imageList_gray[2];

		    calcOpticalFlowFarneback(imageList_gray[1], imageList_gray[0], uflow, 0.5, 1, 50, 3, 5., 1.2, 0);
			calcOpticalFlowFarneback(imageList_gray[2], imageList_gray[0], uflow1, 0.5, 1, 50, 3, 5., 1.2, 0);
			//Mat diff_gray(height_input, width_input, CV_8UC1);
			//linerConstraint(imageList_gray[0], imageList_gray[1], diff_gray);
			//refineFlow(uflow, uflow1,1);
			//refineFlowTwice(uflow, 200);
			//refineFlowTwice(uflow1, 200);
			//cout << "some value in flow" << uflow1.at<Point2f>(10, 10) << endl;
			///flow_list.push_back(uflow);
			//flow_list.push_back(uflow1);
			vector<KeyPoint> kp1, kp2;
			vector<DMatch> matches_all, matches_gms;
		
			vector<bool> vbInliers;
			getKeyPoints(uflow, uflow1, kp1, kp2);
			for (int i = 0; i < width_input*height_input; i++) {
				DMatch tmp = { i,i,0,0 };
				tmp.distance = sqrtf((kp1[i].pt.x-kp2[i].pt.x)*(kp1[i].pt.x - kp2[i].pt.x)+ (kp1[i].pt.y - kp2[i].pt.y)*(kp1[i].pt.y - kp2[i].pt.y));
				matches_all.push_back(tmp);
			}
			gms_matcher gms(kp1, image_list[0].size(), kp2, image_list[0].size(), matches_all);
			
			long num_inliers = gms.GetInlierMask(vbInliers, false, false);
			//image_list[0].copyTo(flow);
			//drawOptFlowMap(flow_list[0], flow, 16, 1.5, Scalar(0, 255, 0));
			//drawOptFlowMap(flow_list[1], flow, 16, 1.5, Scalar(0, 0, 255));
	
			for (size_t i = 0; i < vbInliers.size(); ++i)
			{
				if (vbInliers[i] == true)
				{
					matches_gms.push_back(matches_all[i]);
				}
			}

			Mat show = DrawInlier(image_list[0], image_list[1], kp1, kp2, matches_gms, 1);
			imshow("show", show);
			waitKey();
			//darkChannelFrames(image_list, 3, output, flow_list);
			//imwrite(save_name, output);
			//cout << "file saved " << save_name << endl;
			//imshow("flow", flow);
			//imshow("output", output);
		//	imshow("diff", diff_gray);
			//imshow("diff", diff1);
			//imshow("diff", diff2);
			//imshow("darkchannel", darkChannel);
		
		
			
			
#else
		    //calcDarkChannel(darkChannel, brightChannel, image_list[0], 0);
			//calcDarkChannel(darkChannel1, brightChannel1, image_list[1], 0);
			//Mat diff = brightChannel - darkChannel;
			//Mat diff1 = brightChannel1 - darkChannel1;
			cv::goodFeaturesToTrack(imageList_gray[0], points[0], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
			cv::cornerSubPix(imageList_gray[1], points[0], subPixWinSize, Size(-1, -1), termcrit);
			vector<uchar> status;
			vector<float> err;
			vector<Mat> oldImagePyr, newImagePyr,newImagePyr1;
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
			size_t good_points_after_filter= filterPointsInVectors(filter_status, points[0], points[1], true);
			size_t good_points_after_filter1 = filterPointsInVectors(filter_status1, points0, points[2], true);
			vector<Mat>homo_list;

			Mat homoMat=findHomography(points[0], points[1], CV_RANSAC, 3.0);
			Mat homoMat1 = findHomography(points0, points[2], CV_RANSAC, 3.0);
			//Mat homoMat(3, 3, CV_8UC1, Scalar(0));
			homo_list.push_back(homoMat);
			homo_list.push_back(homoMat1);
			darkFrames(image_list, output, homo_list);
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
			imshow("output", output);
			
			


#endif 
		
		
		

	}		
	
		
   //calcOpticalFlowSF(preImage, image,flow,3, 2, 4, 4.1, 25.5, 18, 55.0, 25.5, 0.35, 18, 55.0, 25.5, 10);	
	
	
	waitKey(0);
		
	
#ifdef DENSEFLOW
	image_list.erase(image_list.begin());
	imageList_gray.erase(imageList_gray.begin());
	flow_list.erase(flow_list.begin(), flow_list.end());
#else
	image_list.erase(image_list.begin());
	imageList_gray.erase(imageList_gray.begin());
	for (int i = 0; i < 3; i++) {
		points[i].erase(points[i].begin(), points[i].end());
			}
#endif 				
	}
	
#endif //FLOW
	
	return 0;
}