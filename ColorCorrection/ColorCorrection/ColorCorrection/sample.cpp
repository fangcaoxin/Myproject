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

//#define WHITEBALANCE
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

	
	//string filename = "..//..//image//4_o.jpg";
	string filename1 = "..//..//image//5_o.jpg";
	//Mat image = imread(filename);
	Mat image = imread(filename1);
	/*Ptr<StereoBM> s_BM= StereoBM::create(16, 21);
	Mat disparity;
	s_BM->compute(image, image1, disparity);
	disparity.convertTo(disparity, CV_32FC1);
	normalize(disparity, disparity, 0, 0.8, NORM_MINMAX);*/
	
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
	opticalModelCorrect(image, res2);
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

#ifdef WHITEBALANCE
	Ptr<xphoto::WhiteBalancer>wb;
	wb = xphoto::createSimpleWB();
	wb->balanceWhite(res2, res2);
	//medianBlur(res2, res2, 3);
#endif //WHITEBALANCE
//#define HISTOGRAM
#ifdef HISTOGRAM
	applyCLAHE(res2);
#endif //HISTOGRAM
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
	
	double s2 = evaluationScore_UCIQUE(res2);
	cout << "s2 " << s2 << endl;
	imshow("res", res2);
	imwrite("res_2.jpg", res2);
	/*cout << "blue " << res1.at<Vec3b>(319, 126) << endl;
	cout << "green " << res1.at<Vec3b>(311, 191) << endl;
	cout << "red " << res1.at<Vec3b>(314, 238) << endl;*/
	//imshow("histogram", image);
	waitKey(0);

}

