#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/saliency.hpp>
#include <iostream>
#include <fstream>
using namespace std;
using namespace cv;
using namespace saliency;
static void rgbStdDev(Mat& image, Mat& labels, Mat& stats, Mat& normalize_std, int size);
static void calcDarkChannel(Mat& darkchannel, Mat& brightchannel, Mat& input, int radius);
int main(int argc, char** argv) {
//#define SALIENCY
#ifdef SALIENCY
	string file_name = "C:\\Research\\OpenCVProject\\image\\img_170724_02j\\img_170724_02j_1.jpg";
	Ptr<Saliency> saliencyAlgorithm;
	saliencyAlgorithm = StaticSaliencySpectralResidual::create();
	Mat saliencyMap,binaryMap,image;
	image = imread(file_name);
	if (saliencyAlgorithm->computeSaliency(image, saliencyMap)) {
		StaticSaliencySpectralResidual spec;
		spec.computeBinaryMap(saliencyMap, binaryMap);

		imshow("Saliency Map", saliencyMap);
		imshow("Original Image", image);
		imshow("Binary Map", binaryMap);
		waitKey(0);
	}

#endif //SALENCY
//#define DARKRED
#ifdef DARKRED
	ifstream image_file;
	image_file.open("list//list_underwater.txt", ios::in);
	while (!image_file.eof()){
		string image_name;
		vector<Mat> channels;
		Mat darkchannel, brightchannel;
		getline(image_file, image_name);
		cout << "filename " << image_name << endl;
		if (image_name.length()==0) break;
		string file_name = "C:\\Research\\OpenCVProject\\image\\img_170724_02j\\img_170724_02j_1.jpg";
		Mat image = imread(file_name);
		split(image, channels);
		calcDarkChannel(darkchannel, brightchannel, image, 0);
		Mat res = brightchannel - darkchannel;
		//Mat res = channels[2]- darkchannel;
		//imwrite("red_channel.jpg",channels[2]);
		//imwrite("dark_channel.jpg", darkchannel);
		//imwrite("reliable_area.jpg", res);

		imshow("res", res);
		waitKey(0);	
	}
	image_file.close();
#endif //DARKRED
#ifdef RGBSTD
	Rect rect(6, 4, 1358 - 6, 1080 - 8);
	ifstream image_file("list//list_image.txt",ios::in);
	ifstream mask_file("list//list_mask.txt",ios::in);
	ofstream std_excel;
	std_excel.open("std_excel.csv", ios::app);
	while (!image_file.eof()) {
		string image_name,mask_name;
		Mat labels, stats, centroids;
		Mat stddev;
		Mat mask1;
		getline(image_file, image_name);
		getline(mask_file, mask_name);
		Mat image = imread(image_name);
		Mat mask = imread(mask_name,0);
		threshold(mask, mask1, 0, 255, CV_THRESH_BINARY);
		int size = connectedComponentsWithStats(mask1, labels, stats, centroids, 8, 4);
		rgbStdDev(image(rect), labels, stats, stddev, size - 1);
		for (int k = 0; k < stddev.rows; k++) {
			std_excel << stddev.at<float>(k, 0) << "," << stddev.at<float>(k, 1) << "," << stddev.at<float>(k, 2)<<endl;
		}
	}
	std_excel.close();
#endif //RGBSTD
//#define VIDEOOUT
#ifdef VIDEOOUT
	VideoWriter outputVideo;
	outputVideo.open("denoise.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 3, Size(676, 268), true);
	if (!outputVideo.isOpened())
	{
		cout << "Could not open the output video for write: " << endl;
		return -1;
	}
	int count = 0;
	ifstream image_file("list//list_compare.txt", ios::in); 
	while(!image_file.eof()) {
		cout << count++ << endl;
		string image_name;
		getline(image_file, image_name);
		if (image_name.length() == 0) break;
		Mat image = imread(image_name);
		Mat res = image;
		outputVideo << res;
	}
#endif //VIDEO
#define VIDEOIN
#ifdef VIDEOIN
	VideoCapture cap;
	string video_file = "C:\\Research\\video\\capture.mp4";
	string save_folder = "C:\\Research\\OpenCVProject\\image\\img_171021\\171021_capture_";
	cap.open(video_file);
	if (!cap.isOpened()) {
		cout << "not opened " << endl;
		return -1;
	}
	Mat frame;
	Rect rect(40, 3, 640, 480);
	int count = 0;
	for (;;) {
		cap >> frame;
		Mat image = frame(rect);
		string save_name = save_folder + to_string(count) + ".jpg";
		count++;
		imwrite(save_name, image);
		/*imshow("image", image);
		waitKey(0);*/
	}
#endif //VIDEOIN
	return 0;
}

static void rgbStdDev(Mat& image, Mat& labels, Mat& stats, Mat& normalize_std, int size) {
	int width = image.cols;
	int height = image.rows;
	Mat normalize_std_tmp(size, 3, CV_32FC1, Scalar(0));
	Mat rgb_std(size, 3, CV_32FC1, Scalar(0));
	Mat rgb_sum(size, 3, CV_32FC1, Scalar(0));
	Mat rgb_sum2(size, 3, CV_32FC1, Scalar(0));

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (labels.at<int>(i, j) == 0) continue;
			else {
				for (int k = 0; k < 3; k++) {
					rgb_sum.at<float>(labels.at<int>(i, j) - 1, k) += (float)image.at<Vec3b>(i, j)[k];
					rgb_sum2.at<float>(labels.at<int>(i, j) - 1, k) += (float)image.at<Vec3b>(i, j)[k] * (float)image.at<Vec3b>(i, j)[k];

				}
				//cout << i << " " << j << " " << image.at<Vec3b>(i, j) << endl;
			}
		}
	}
	for (int m = 0; m < size; m++) {
		for (int k = 0; k < 3; k++) {
			rgb_std.at<float>(m, k) = rgb_sum2.at<float>(m, k) / stats.at<int>(m + 1, 4) - (rgb_sum.at<float>(m, k) / stats.at<int>(m + 1, 4))*(rgb_sum.at<float>(m, k) / stats.at<int>(m + 1, 4));
		}
		float sum_bgr = rgb_std.at<float>(m, 0) + rgb_std.at<float>(m, 1) + rgb_std.at<float>(m, 2);

		if (stats.at<int>(m + 1, 4) < 3 || sum_bgr<0.001) {
			normalize_std_tmp.at<float>(m, 0) = 20;

		}
		else {
			float b_stddev = sqrt(rgb_std.at<float>(m, 0) / 1);
			float g_stddev = sqrt(rgb_std.at<float>(m, 1) /1);
			float r_stddev = sqrt(rgb_std.at<float>(m, 2) / 1);


			//float average_std = (b_stddev + g_stddev + r_stddev) / 3;
			//float n_stddev = ((r_stddev - average_std)*(r_stddev - average_std) + \
			//	(g_stddev - average_std)*(g_stddev - average_std) + \
			//	(b_stddev - average_std)*(b_stddev - average_std)) / 3;
			
			normalize_std_tmp.at<float>(m, 0) = b_stddev;
			normalize_std_tmp.at<float>(m, 1) = g_stddev;
			normalize_std_tmp.at<float>(m, 2) = r_stddev;



		}
		cout << "t_std " << normalize_std_tmp.at<float>(m, 0) << endl;
	}

	normalize_std = normalize_std_tmp;
}

static void calcDarkChannel(Mat& darkchannel, Mat& brightchannel, Mat& input, int radius) {


	int height = input.rows;
	int width = input.cols;
	darkchannel.create(height, width, CV_8UC1);
	brightchannel.create(height, width, CV_8UC1);

	int st_row, ed_row;
	int st_col, ed_col;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			st_row = i - radius, ed_row = i + radius;
			st_col = j - radius, ed_col = j + radius;

			st_row = st_row < 0 ? 0 : st_row;
			ed_row = ed_row >= height ? (height - 1) : ed_row;
			st_col = st_col < 0 ? 0 : st_col;
			ed_col = ed_col >= width ? (width - 1) : ed_col;


			int min = 300;
			int max = 0;
			for (int m = st_row; m <= ed_row; m++)
			{
				for (int n = st_col; n <= ed_col; n++)
				{
					for (int k = 0; k < 3; k++)
					{

						int cur = input.at<Vec3b>(m, n)[k];
						if (cur < min)
							min = cur;

						if (cur > max)
							max = cur;
					}
				}
			}
			darkchannel.at<uchar>(i, j) = min;
			brightchannel.at<uchar>(i, j) = max;
		}
	}

}