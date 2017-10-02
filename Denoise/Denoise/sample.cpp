#include <iostream>
#include "core.h"
#include "math.h"
#include "dehaze.h"
#include "utility.h"
#include "EM.h"
//#define FLOW
//#define MOG
//#define DEHAZE
#define BAYESIAN
#define SAVERESULT

//using namespace cv::optflow;
Ptr<BackgroundSubtractor> pMOG;
 //string folderName = "img_170721_01j";
string folderName = "img_170724_02j";
string saveFolder = "..//..//..//image//" + folderName + "//" + folderName + "_";
string saveImage = "..//..//..//result//20171002//" + folderName + "_";
int width = 1358; //1358 without black range
int height = 1080; //1080

int main(int argc, char* argv[]) {


	int width_input = width /4;
	int height_input = height/4;

	int beg_num = 196;
	int frame_num = 100;
	Mat backgroud;
	int frame_count = 0;
	Mat fgMaskMOG(height_input, width_input, CV_8UC1, Scalar(0));
	pMOG = createBackgroundSubtractorMOG2(5, 10, false);
	Ptr<FastFeatureDetector> fast = FastFeatureDetector::create();
	vector<Mat> image_list;
	vector<Mat> image_list_gray;


	int count = 0;
	for (int i = beg_num; i < beg_num + frame_num; i = i + 1) {
		string file_name = saveFolder + to_string(i) + ".jpg";
		string save_name = saveImage + to_string(i-1) + "_1002out.jpg";
		cout << "file name :" << file_name << endl;
		Mat cur = imread(file_name);
		Mat input_resize, cur_gray;
		Mat dehaze_out(height_input, width_input, CV_8UC3);
		resize(cur, input_resize, Size(width_input, height_input));
		//dehazeDC(input_resize, dehaze);
		//dehaze(dehaze_out,input_resize);
		image_list.push_back(input_resize);
		cvtColor(input_resize, cur_gray, CV_BGR2GRAY);
		image_list_gray.push_back(cur_gray);
		count++;
//#define TEST
		
#ifdef TEST
		string file_1 = "..//..//..//result//1-1.jpg";
		string file_2 = "..//..//..//result//2-1.jpg";

		vector<Mat> diff_patch;
		Mat img_1 = imread(file_1);
		Mat img_2 = imread(file_2);
		cvtColor(img_1, img_1, CV_BGR2GRAY);
		cvtColor(img_2, img_2, CV_BGR2GRAY);
		diff_patch.push_back(img_1);
		diff_patch.push_back(img_2);
		Mat sum(img_1.size(), CV_8UC1, Scalar(0));
		Mat inter(img_1.size(), CV_8UC1, Scalar(0));
		sumAreaByRadius(diff_patch, sum, 5);
		showLabelImg(sum);
		diffByPreNext(diff_patch, inter);
		imwrite("..//..//..//result//sum_1_2.jpg", sum);
		imwrite("..//..//..//result//inter_1_2.jpg", inter);
	}
#endif //TEST
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

			Mat diff_output(height_input, width_input, CV_8UC1, Scalar(0));
			Mat diff_output1(height_input, width_input, CV_8UC1, Scalar(0));
			Mat diff_iter(height_input, width_input, CV_8UC1, Scalar(0));
			Mat label_init(height_input, width_input, CV_8UC1, Scalar(0));
			Mat darkChannel(height_input, width_input, CV_8UC1, Scalar(0));
			FrameRelativeDiff(image_list_gray, diff);
			diffByThreshold(diff, diff_wb, 5);
			int num= sumAreaByRadius(diff_wb, diff_output, 20);
			Mat cdfd;
			diff_output.copyTo(cdfd);
			EMSegmetation(image_list[1], diff_output,num,3);
			//calcDarkChannel(darkChannel, diff_output, image_list[0], 0);
			//labelInitByDiff(diff_wb, label_init);
			//imwrite("diff_sum.jpg", diff_output);
			//double error = modelError(diff, diff_output);
			//diffByPreNext(diff_wb, diff_output1);
			vector<vector<Point>> contours;
			vector<Vec4i> hierarchy;
			vector<int> valid_labels;
			//split(image_list[0], channels);
			//Mat trans = channels[2] - darkChannel;
			//threshold(trans, trans, 20, 255, CV_THRESH_BINARY);
			//labelInitByRedDarkChannel(trans, label_init);
			//bayesianEstimation(image_list[1], diff_output, diff_iter,2, 2, 1);
			//int size=connectedComponentsWithStats(diff_output, labels, stats, centroids, 8, 4);
			//showLabelImg(diff_iter);
			
			//showAreaLabel(img_label, labels, centroids, size);
			//vector<int> valid_label1,valid_label2;
			//shapeFilter(diff_output, labels, stats, size,valid_label1);
			//findContours(diff_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			//contourSobel(image_list_gray[1], hierarchy, contours);
			//neighbourBlockMatching(labels, stats, centroids, image_list_gray,valid_labels);

			//imshow("diff_by_sumarea", diff_output);
			darkFramesByMask(image_list, output, diff_output);
			showLabelImg(diff_output);
			showLabelImg(cdfd);
#ifdef SAVERESULT
			Mat combine1, combine2, combine;
			cvtColor(cdfd, cdfd, CV_GRAY2BGR);
			cvtColor(diff_output, diff_output, CV_GRAY2BGR);
			putText(image_list[1], "Original", Point(10, 10),CV_FONT_HERSHEY_PLAIN,1,Scalar(0,255,0));
			putText(output, "Processed", Point(10,10), CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			putText(cdfd, "Combined DFD", Point(10, 10), CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			putText(diff_output, "After EM", Point(10, 10), CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));

			
			
			hconcat(image_list[1], output, combine1);
			hconcat(cdfd, diff_output, combine2);
			vconcat(combine1, combine2, combine);
			imshow("combine", combine);
			imwrite(save_name, combine);
#endif //SAVERESULT
			//imshow("diff_by_sum", diff_output);
			//imshow("area label", img_label);
			//imshow("original diff", diff_output);
			//imshow("cdfd", cdfd);
			//imshow("output", output);
			//imshow("trans", trans);
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
	}
	else {
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
