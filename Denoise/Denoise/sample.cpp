
#include <iostream>
//#include "core.h"
//#include "math.h"
//#include "dehaze.h"
//#include "utility.h"
//#include "EM.h"
//#include "restoration.h"
#include <fstream>
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
std::string folderName = "img_170721_02j";
//std::string folderName = "real_images_1";
std::string saveFolder = "ctdf//";
//std::string saveFolder = "..//..//SfM//" + folderName + "//" + "denoise//";
int width = 1358; //1358 without black range
int height = 1080; //1080
cv::Rect rect(6, 4, width - 6, height - 8);
int main(int argc, char* argv[]) {
	std::ifstream image_file;
	image_file.open("list02.txt", std::ios::in);

	cv::Mat res;
	cv::Mat backgroud;
	int frame_count = 0;
	std::vector<cv::Mat> image_list;
	std::vector<cv::Mat> image_list_gray;
	
	while (!image_file.eof()) {
		std::string save_filename = saveFolder + std::to_string(frame_count++) + ".jpg";
		std::string image_name;
		cv::Mat cur_gray;
		cv::Mat cur;
		std::getline(image_file, image_name);
		if (image_name.length() == 0) break;
		cur = cv::imread(image_name);
		cv::resize(cur, cur, cv::Size(640, 480));
		image_list.push_back(cur);
		cv::cvtColor(cur, cur_gray, CV_BGR2GRAY);
		image_list_gray.push_back(cur_gray);
		if (frame_count < 3) { continue; }
		else 
		{
			std::vector<cv::Mat> diff_c;
			std::vector<cv::Mat> diff_wb_c;
			std::vector<cv::Mat> channels;
			std::vector<cv::Mat> image_list_compensation;
			std::vector<cv::Mat> image_list_gray_compensation;
			std::vector<cv::Mat> camera_motion;
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
			calcPyrLKflow(image_list_gray, trans, camera_motion);
			imageListCompensation(image_list, image_list_compensation, camera_motion);
			imageListGrayCompensation(image_list_gray, image_list_gray_compensation, camera_motion);
			FrameRelativeDiff(image_list_gray_compensation, diff_c);
			cv::imshow("diff1", diff_c[1]);
			cv::waitKey(0);
			cv::imwrite("ctdf//diff_1.jpg", diff_c[0]);
			cv::imwrite("ctdf//diff_2.jpg", diff_c[1]);
			
			diffByThreshold(diff_c, diff_wb_c, 5);
			cv::imwrite("ctdf//diff_b1.jpg", diff_wb_c[0]);
			cv::imwrite("ctdf//diff_b2.jpg", diff_wb_c[1]);
			
#endif //CAMERAMOTION
			cv::Mat cdfd, normalize_std, normalize_std1;
			cv::Mat labels, stats, centroids;
			cv::Mat labels1, stats1, centroids1;
			cv::Mat labels_final, stats_final, centroids_final;
			cv::Mat labels_show, labels1_show;
			std::vector<cv::Mat> diff_wb_c_std;
			int size = cv::connectedComponentsWithStats(diff_wb_c[0], labels, stats, centroids, 8, 4);
			int size1 = cv::connectedComponentsWithStats(diff_wb_c[1], labels1, stats1, centroids1, 8, 4);
			rgbStdDev(image_list[1], labels, stats, normalize_std, size - 1);
			rgbStdDev(image_list[1], labels1, stats1, normalize_std1, size1 - 1);
			getMaskFromStd(labels, normalize_std);
			
			getMaskFromStd(labels1, normalize_std1);
			cv::imwrite("ctdf//label1.jpg", diff_wb_c[0]);
			cv::imwrite("ctdf//label2.jpg", diff_wb_c[1]);
			int num = sumAreaByRadius(labels, labels1, centroids, centroids1, diff_output_c, 5);
			//imageClosing(diff_output_c, diff_output_c, 12);
			cv::imwrite("ctdf//ctdf.jpg", diff_output_c);
			cv::imshow("diff", diff_output_c);
			cv::waitKey(0);
			int size_final = connectedComponentsWithStats(diff_output_c, labels_final, stats_final, centroids_final);
			floatingAreaRestoration(image_list_compensation, image_list_gray_compensation, stats_final, labels_final, res);
#ifdef TIME
			QueryPerformanceCounter(&et);
			std::cout << "Processint time: " << (et.QuadPart - st.QuadPart) * 1000 / Freq.QuadPart << "ms" << std::endl;
#endif
#if 0
			cv::imshow("debug", image_list_gray[0] -image_list_gray[1]);
#endif
			image_list.erase(image_list.begin());

			image_list_gray.erase(image_list_gray.begin());

			frame_count = 2;	
		}
		cv::imshow("result", res);
		cv::waitKey(0);
	}
	

return 0;
}