#include <opencv2/videoio/videoio.hpp>


#include <iostream>

#include "core.h"
#include "math.h"
#include "dehaze.h"
#include "utility.h"
//#define FLOW
//#define MOG
//#define DEHAZE
#define BAYESIAN


//using namespace cv::optflow;
Ptr<BackgroundSubtractor> pMOG;
string folderName = "img_170721_01j";
//string folderName = "img_170724_02j";
string saveFolder = "..//..//..//image//" + folderName + "//" + folderName + "_";
string saveImage = "..//..//..//result//img_170724_02j//img_170724_02j_";
int width = 1358; //1358 without black range
int height = 1080; //1080
int main(int argc, char* argv[]) {


	int width_input = width/4;
	int height_input = height/4;

	int beg_num = 140;
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
			vector<Mat> diff;
			vector<Mat> diff_wb;
			vector<Mat> channels;
			//Mat diff = image_list_gray[1] - image_list_gray[0];
			Mat  labels, stats, centroids;
			Mat output(height_input, width_input, CV_8UC3);
			Mat img_label(height_input, width_input, CV_8UC3);
			Mat diff_output(height_input, width_input, CV_8UC1,Scalar(0));
			Mat diff_output1(height_input, width_input, CV_8UC1, Scalar(0));
			Mat diff_iter(height_input, width_input, CV_8UC1, Scalar(0));
			Mat label_init(height_input, width_input, CV_8UC1, Scalar(0));
			Mat darkChannel(height_input, width_input, CV_8UC1, Scalar(0));
			FrameRelativeDiff(image_list_gray, diff);
			diffByThreshold(diff, diff_wb, 3);
			//sumAreaByRadius(diff_wb, diff_output, 20);
			calcDarkChannel(darkChannel, diff_output, image_list[1], 0);
			labelInitByDiff(diff_wb, label_init);
			//imwrite("diff_sum.jpg", diff_output);
			//double error = modelError(diff, diff_output);
			diffByPreNext(diff_wb, diff_output1);
			vector<vector<Point>> contours;
			vector<Vec4i> hierarchy;
			vector<int> valid_labels;
			split(image_list[1], channels);
			Mat trans = channels[2] - darkChannel;
			threshold(trans, trans, 5, 255, CV_THRESH_BINARY);
			labelInitByRedDarkChannel(trans, label_init);
			bayesianEstimation(image_list[1], label_init, diff_iter, 3, 2, 1);
			//int size=connectedComponentsWithStats(diff_output, labels, stats, centroids, 8, 4);
			
			//showAreaLabel(img_label, labels, centroids, size);
			//vector<int> valid_label1,valid_label2;
			//shapeFilter(diff_output, labels, stats, size,valid_label1);
			//findContours(diff_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			//contourSobel(image_list_gray[1], hierarchy, contours);
			//neighbourBlockMatching(labels, stats, centroids, image_list_gray,valid_labels);
			
			//imshow("diff_by_sumarea", diff_output);
			imshow("diff_by_preNext", diff_output1);
			//double error = modelError(diff,diff_output);
			//neighbourBlockDiff(labels, stats, centroids, image_list_gray, valid_labels,100*error);
			//spatialFilter(labels, diff_output, valid_labels);

			//imwrite("out2.jpg", diff_output);
			//distributeFilter(diff_output, labels, stats,image_list_gray[1],valid_label1, valid_label2);
			//maskRefinement(diff_output, labels, image_list_gray[1], valid_label2);
			//Mat image;
			//image_list[1].copyTo(image);
			//medianFramesByMask(image, stats, valid_labels);
			darkFramesByMask(image_list, output, diff_iter);
			showLabelImg(diff_iter);
			/*int idx = 0;
			for (; idx >= 0; idx = hierarchy[idx][0]) {
				drawContours(image_list[1], contours, idx, Scalar(0, 0, 255), 1, 8, hierarchy);
			}*/
			//imshow("area label", img_label);
			//imshow("original diff", diff_output);
			imshow("result", diff_iter);
			imshow("output", output);
			imshow("trans", trans);
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