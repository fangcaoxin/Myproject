#include "colorSpaceConversion.h"
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/xphoto.hpp>
#include <opencv2/calib3d.hpp>

#include "dehaze.h"
#include "enhance.h"
#include "localColorCorrection.h"
#include "illumiCorrection.h"
#include "opticalModel.h"

using namespace cv;
using namespace std;

//#define OLD
//#define LIST
//#define NEW


static void applyCLAHE(Mat& img) {
	vector<Mat> lab_channels;
	
	cvtColor(img, img, CV_BGR2Lab);
	split(img, lab_channels);
	Ptr<CLAHE> clahe = createCLAHE(2.0, Size(8, 8));
	Mat L1;
	clahe->apply(lab_channels[0], lab_channels[0]);
	

	cv::merge(lab_channels, img);//merage histogram equilizied brightness
	cvtColor(img,img, CV_Lab2BGR);

}

static double PCQI(Mat& img_1, Mat& img_2) {
	
	int height = img_1.rows;
	int width = img_1.cols;
	img_1.convertTo(img_1, CV_32FC1);
	img_2.convertTo(img_2, CV_32FC1);
	int patch_num = 0;
	
	double pcqi = 0.;
	Rect image_rect(0, 0, width, height);
	for (int i = 0; i < height-height / 4; i = i + height / 4) {
		for (int j = 0; j < width-width / 4; j = j + width / 4) {
			patch_num++;
			Rect rect(i, j, width / 4, height / 4);
			Rect valid_rect = rect& image_rect;
			Mat patch_x = img_1(valid_rect);
			Mat patch_y = img_2(valid_rect);
			int N = sqrt(double(valid_rect.width*valid_rect.height));
			double v1 = 1. / N;

			Scalar mean_patch = mean(patch_x);
			Scalar mean_patch_y = mean(patch_y);

			double c1_x = N*mean_patch[0];
			double c1_y = N*mean_patch_y[0];
	
			Mat c2_y_r = patch_y -mean_patch_y[0];
			
			Mat v2 = (patch_x - mean_patch[0]) / norm(patch_x - mean_patch[0]);
			
			double q_s = c2_y_r.dot(v2) / (norm(c2_y_r)*norm(v2));
			cout << c2_y_r.dot(v2) << endl;
			double c2_x = norm(patch_x - mean_patch[0]);
			double c2_y = patch_y.dot(v2);
			double q_c = 4*atan(abs(c2_y/c2_x))/CV_PI; //0~2
			double q_i = exp(-(c1_x - c1_y) / (N * 255));
			pcqi += q_c*q_s*q_i;
		}
	}
	return pcqi /patch_num;
}



int main(int argc, char** argv) {
	int beg_no = 0;
#ifdef NEW
	string folder_name = "..//..//..//..//image//img_171021//171021_capture_";
	string filename = folder_name + to_string(beg_no) + ".jpg";
#endif //NEW
#ifdef OLD
	//string folderName = "img_170721_01j";
	string folderName = "img_170724_02j";
	string folder_name = "..//..//..//..//image//" + folderName + "//" + folderName + "_";
	string saveImage = "..//..//..//result//20171018//" + folderName + "_";
	string filename = folder_name + to_string(beg_no) + ".jpg";
#endif//OLD
#ifdef LIST
	ifstream image_file;
	image_file.open("list//list_out.txt", ios::in);
	vector<string> image_list;
	while (!image_file.eof()) {
		string image_name;
		getline(image_file, image_name);
		if (image_name.length() == 0) break;
		image_list.push_back(image_name);
	}
	image_file.close();
	string filename = image_list[beg_no];
#endif //LIST

	
	string filename = "..//..//image//out_4.jpg";
	string filename1 = "..//..//image//res_1.jpg";
	Mat image = imread(filename,0);
	Mat image1 = imread(filename1,0);
	/*Ptr<StereoBM> s_BM= StereoBM::create(16, 21);
	Mat disparity;
	s_BM->compute(image, image1, disparity);
	disparity.convertTo(disparity, CV_32FC1);
	normalize(disparity, disparity, 0, 0.8, NORM_MINMAX);*/
//#define PSNREVALUATION
#ifdef PSNREVALUATION
	double psnr_score = PSNR(image, image1);
	cout << "psnr_score " << psnr_score << endl;
#endif //PSNREVALUATION
#define PCQI_C
#ifdef PCQI_C
	double score_PCQI = PCQI(image, image1);
#endif //PCQI_C
	////Mat diff = abs(image - image1);
	//imshow("dis", disparity);
	Mat res,res1,res2;
	Mat L, R;
	Mat V,image_hsv;
	double s1 = evaluationScore_UCIQUE(image);
	cout << "s1 " << s1 << endl;
	//Mat gray;
	//cvtColor(image, gray, CV_BGR2GRAY);
	/*Scalar mean_gray = mean(gray);
	Mat gray_diff = gray - mean_gray[0];
	threshold(gray_diff, gray_diff, 5,255,THRESH_BINARY);*/

	//medianBlur(image, image, 5);
	//localColorCorrection(image, res);
	
	//dehazeMY(image, res);
	//illumiCorrection(image, res);
	//dehazeByBright(image, res2);
	//calcMaxReflectChannelColorMap(image, res2, 7);
	//opticalModelCorrect(image, res2);
	//vector<Mat> bgr_channels;
	//vector<Mat> hsv_channels;
	//split(image, bgr_channels);
	//cvtColor(image, image_hsv, CV_BGR2HSV);
	//split(image_hsv, hsv_channels);
	////Mat R_r= bgr_channels[2].mul(1/hsv_channels[2]);
	////normalize(R_r, R_r, 0, 1, NORM_MINMAX);
	//Mat diff =hsv_channels[2]-bgr_channels[2];
	////normalize(diff, diff, 0, 1, NORM_MINMAX);
	//imshow("diff", diff);
	//imshow("R_r", R_r);

//#define WHITEBALANCE
#ifdef WHITEBALANCE
	Ptr<xphoto::WhiteBalancer>wb;
	wb = xphoto::createSimpleWB();
	wb->balanceWhite(image, image);
	//medianBlur(res2, res2, 3);
#endif //WHITEBALANCE
//#define HISTOGRAM
#ifdef HISTOGRAM
	applyCLAHE(image);
#endif //HISTOGRAM
	//opticalModelCorrect(image, res2);
	//dehaze(image, res2);
	//dehazeMY(res, res1);
	//enhance(image, res2);
	//dehazeDC(image, res);
	//res= L.mul(R);
	//normalize(res, res, 0, 1, NORM_MINMAX, -1, Mat());
	//imwrite("dehaze10ehance.jpg", res1);
	//imshow("whitebalance", res);
	//imwrite("res.jpg", res1);
	//imwrite("res_1102.jpg", res2);
	
	//double s2 = evaluationScore_UCIQUE(res2);
	//cout << "s2 " << s2 << endl;
	//imshow("res", res2);
	////imwrite("res_udcp_color.jpg", res2);
	/*cout << "blue " << res1.at<Vec3b>(319, 126) << endl;
	cout << "green " << res1.at<Vec3b>(311, 191) << endl;
	cout << "red " << res1.at<Vec3b>(314, 238) << endl;*/
	//imshow("histogram", image);
	waitKey(0);

}

