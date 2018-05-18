
#include <iostream>
//#include "core.h"
//#include "math.h"
//#include "dehaze.h"
//#include "utility.h"
//#include "EM.h"
//#include "restoration.h"
#include "particle_removal.h"
#define TIME
#ifdef TIME
#include <windows.h>
#endif 

//#define SAVERESULT
//#define EM
#define CAMERAMOTION
#define CONNECTED
#ifdef TIME
LARGE_INTEGER Freq; 
LARGE_INTEGER st;                 
LARGE_INTEGER et;
#endif
//string folderName = "img_170721_01j";
std::string folderName = "img_170724_02j";
std::string saveFolder = "..//..//..//image//" + folderName + "//" + folderName + "_";
std::string saveImage = "..//..//..//result//20171210//" + folderName + "_";
int width = 1358; //1358 without black range
int height = 1080; //1080
cv::Rect rect(6, 4, width - 6, height - 8);
int main(int argc, char* argv[]) {


	int width_input = rect.width;
	int height_input = rect.height;

	int beg_num = 39;
	int frame_num = 150;
	cv::Mat backgroud;
	int frame_count = 0;
	std::vector<cv::Mat> image_list;
	std::vector<cv::Mat> image_list_gray;
	int count = 0;
	for (int i = beg_num; i < beg_num + frame_num; i = i + 1) {
		std::string file_name = saveFolder + std::to_string(i) + ".jpg";
		std::string save_name = saveImage + std::to_string(i - 1) + "_1210out.jpg";
		std::cout << "file name :" << file_name << std::endl;
		cv::Mat cur = cv::imread(file_name);
		cv::Mat input_resize, cur_gray;
		resize(cur(rect), input_resize, cv::Size(640, 480));
		image_list.push_back(input_resize);
		cvtColor(input_resize, cur_gray, CV_BGR2GRAY);
		image_list_gray.push_back(cur_gray);
		count++;

		if (count < 3) {
			continue;
		}
		else {
			std::vector<cv::Mat> diff_c;
			std::vector<cv::Mat> diff_wb_c;
			std::vector<cv::Mat> channels;
			std::vector<cv::Mat> image_list_compensation;
			std::vector<cv::Mat> image_list_gray_compensation;
			std::vector<cv::Mat> camera_motion;
			cv::Mat output;
			cv::Mat diff_output_c, darkChannel, brightChannel;
#ifdef TIME
			QueryPerformanceFrequency(&Freq);
			QueryPerformanceCounter(&st);
#endif
#ifdef CAMERAMOTION
			calcDarkChannel(darkChannel, brightChannel, image_list[1], 0);
			cv::split(image_list[1], channels);
			cv::Mat trans = channels[2] - darkChannel;
			cv::threshold(trans, trans, 20, 255, CV_THRESH_BINARY);
			calcPyrLKflow(image_list_gray, trans,camera_motion);
			imageListCompensation(image_list, image_list_compensation, camera_motion);
			imageListGrayCompensation(image_list_gray, image_list_gray_compensation, camera_motion);
			FrameRelativeDiff(image_list_gray_compensation, diff_c);
			diffByThreshold(diff_c, diff_wb_c, 5);
#endif //CAMERAMOTION
#ifdef CONNECTED
			cv::Mat cdfd,normalize_std,normalize_std1;
			cv::Mat labels, stats, centroids;
			cv::Mat labels1, stats1, centroids1;
			cv::Mat labels_final, stats_final, centroids_final;
			cv::Mat labels_show, labels1_show;
			std::vector<cv::Mat> diff_wb_c_std;
			int size= cv::connectedComponentsWithStats(diff_wb_c[0], labels, stats, centroids, 8, 4);
			int size1 = cv::connectedComponentsWithStats(diff_wb_c[1], labels1, stats1, centroids1, 8, 4);
			rgbStdDev(image_list[1], labels, stats, normalize_std, size-1);
			rgbStdDev(image_list[1], labels1, stats1, normalize_std1, size1 - 1);
			getMaskFromStd(labels, normalize_std);
			getMaskFromStd(labels1, normalize_std1);
			int num = sumAreaByRadius(labels, labels1, centroids, centroids1, diff_output_c, 5);
			imageClosing(diff_output_c, diff_output_c, 12);
			int size_final = connectedComponentsWithStats(diff_output_c, labels_final, stats_final, centroids_final);
			floatingAreaRestoration(image_list_compensation, image_list_gray_compensation, stats_final, labels_final, output);
#ifdef TIME
			QueryPerformanceCounter(&et);
			std::cout << "Processint time: " << (et.QuadPart - st.QuadPart) * 1000 / Freq.QuadPart << "ms" << std::endl;
#endif
			//showMaskImg(labels, labels_show);
			//showMaskImg(labels1, labels1_show);
			//imwrite("label.jpg", labels_show);
			//imwrite("label1.jpg", labels1_show);
			/*diff_output_c.copyTo(cdfd);*/
			//showLabelImg(diff_output_c);
			//vector<float> probs_similar;
			//nearNeighourSimilarity(image_list[1], stats, probs_similar);
#endif //CONNECTED

#ifdef SAVERESULT
		Mat combine1, combine2, combine;
		//cvtColor(cdfd, cdfd, CV_GRAY2BGR);
		cvtColor(diff_output, diff_output, CV_GRAY2BGR);
		putText(image_list[1], "Original", Point(10, 10), CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
		putText(output, "Processed", Point(10, 10), CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
		putText(cdfd, "Combined DFD", Point(10, 10), CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
		putText(diff_output, "After EM", Point(10, 10), CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));



		hconcat(image_list[1], output, combine1);
		hconcat(cdfd, diff_output, combine2);
		vconcat(combine1, combine2, combine);
		imshow("combine", combine);
		imwrite(save_name, combine);
#endif //SAVERESULT

			//imshow("diff_by_sum", diff_output);
			//imshow("area label", show_img);
			//imshow("original diff", diff_output);
	/*	Mat res;
		cvtColor(diff_output_c, diff_output_c, CV_GRAY2BGR);
		vector<Mat> res_save;
		res_save.push_back(image_list[1]);
		res_save.push_back(diff_output_c);
		res_save.push_back(output);
		hconcat(res_save, res);*/
			//imshow("cdfd", diff_output_c);
			//imwrite("cdfd.jpg", diff_output_c);
			//imshow("output", output);
			//imshow("trans", trans);
			//imwrite(save_name, output);
			//imshow("diff_cur_pre_camera", diff_wb_c[0]);
			//imshow("diff_cur_next_camera", diff_wb_c[1]);
			//imwrite(save_name, output);
			//imshow("diff_cur_pre_std", labels_show);
			//imshow("diff_cur_next_std", labels1_show);
			//imwrite(save_name, labels_show);
			image_list.erase(image_list.begin());
			image_list_gray.erase(image_list_gray.begin());

			count = 2;
			cv::waitKey(0);


		}

	}






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