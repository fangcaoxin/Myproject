
#include <opencv2/videoio/videoio.hpp>


#include <iostream>

#include "core.h"
#include "math.h"
#include "dehaze.h"
#include "utility.h"
//#define FLOW
//#define MOG
//#define DEHAZE


//using namespace cv::optflow;
Ptr<BackgroundSubtractor> pMOG;
//string folderName = "img_170721_01j";
string folderName = "img_170724_02j";
string saveFolder = "..//..//..//image//" + folderName + "//" + folderName + "_";
string saveImage = "..//..//..//result//img_170724_02j//img_170724_02j_";
int width = 1358; //1358 without black range
int height = 1080; //1080
int main(int argc, char* argv[]) {

	int width_input = width;
	int height_input = height;
	int beg_num = 0;
	int frame_num = 100;
	Mat backgroud;
	int frame_count = 0;
	Mat fgMaskMOG(height_input,width_input,CV_8UC1,Scalar(0));
	pMOG = createBackgroundSubtractorMOG2(5, 10, false);
	Ptr<FastFeatureDetector> fast = FastFeatureDetector::create();
	vector<Mat> image_list;
	vector<Mat> image_list_gray;
	
	int count = 0;
	for (int i = beg_num; i < beg_num + frame_num; i = i + 1) {
		string file_name = saveFolder + to_string(i) + ".jpg";
		string save_name = saveImage + to_string(i) + "_0918out.jpg";
		cout << "file name :" << file_name << endl;
		Mat cur = imread(file_name);
		Mat input_resize, cur_gray;
		resize(cur, input_resize, Size(width_input, height_input));
		//dehaze(input_resize, input_resize);
		image_list.push_back(input_resize);
		cvtColor(input_resize, cur_gray, CV_BGR2GRAY);
		image_list_gray.push_back(cur_gray);
		count++;
#define CONNECTED
#ifdef CONNECTED
		

		if (count < 3) {
			continue;
		}
		else {
			Mat diff = image_list_gray[1] - image_list_gray[0];
			Mat diff_wb, labels, stats, centroids;
			Mat output(height_input, width_input, CV_8UC3);
			Mat img_label(height_input, width_input, CV_8UC3);
			threshold(diff, diff_wb, 10, 255, CV_THRESH_BINARY);
			vector<vector<Point>> contours;
			vector<Vec4i> hierarchy;
			
			//imshow("diff_wb after threshold", diff_wb);
			int size=connectedComponentsWithStats(diff_wb, labels, stats, centroids, 8, 4);
			showAreaLabel(img_label, labels, centroids, size);
			vector<int> valid_label1,valid_label2;
			shapeFilter(diff_wb, labels, stats, size,valid_label1);
			findContours(diff_wb, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			contourSobel(image_list_gray[1], hierarchy, contours);
			//imshow("diff_wb after shape fliter", diff_wb);
			cout << "ShaperFilter finished" << endl;
			//distributeFilter(diff_wb, labels, stats,image_list_gray[1],valid_label1, valid_label2);
			//maskRefinement(diff_wb, labels, image_list_gray[1], valid_label2);
			darkFramesByMask(image_list, output, diff_wb);
			int idx = 0;
			for (; idx >= 0; idx = hierarchy[idx][0]) {
				drawContours(image_list[1], contours, idx, Scalar(0, 0, 255), 1, 8, hierarchy);
			}
			//imshow("area label", img_label);
			imshow("diff", diff_wb);
			//imshow("output", output);
			//imshow("origial", image_list[1]);
			image_list.erase(image_list.begin());
			image_list_gray.erase(image_list_gray.begin());
			count = 2;
			waitKey(0);

		}
	}
		
#endif //CONNECTED

#ifdef MOG
	
	vector<Mat> mask_list;
	Mat output(height_input, width_input, CV_8UC3);
	pMOG->apply(cur(valid_rect), fgMaskMOG, 0.8);
	mask_list.push_back(fgMaskMOG);

	if (count < 3) {
		continue;
	}else{
		darkFramesByMask(image_list, output, mask_list[0]);
		imshow("FG Mask", mask_list[0]);
		imshow("output", output);
		//imshow("background", backgroud);
		cv::waitKey(0);
		frame_count = 2;
		mask_list.erase(mask_list.begin(), mask_list.end() - 1);
		image_list.erase(image_list.begin());

	}
	
	}
#endif //MOG
	return 0;
}