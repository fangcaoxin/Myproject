#include "colorSpaceConversion.h"
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/xphoto.hpp>
#include "dehaze.h"
#include "enhance.h"
#include "localColorCorrection.h"

using namespace cv;
using namespace std;
//#define OLD
#define LIST
//#define NEW
//#define WHITEBALANCE
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
	//string filename = "..//..//image//2.jpg";
	//string filename = "city.png";
	Mat image = imread(filename);
	Mat res,res1;
	localColorCorrection(image, res);
	dehaze(res, res1);
	//dehazeMY(image, res);
#ifdef WHITEBALANCE
	Ptr<xphoto::WhiteBalancer>wb;
	wb = xphoto::createSimpleWB();
	wb->balanceWhite(res, res1);
#endif //WHITEBALANCE
	//enhance(image, res);
	imshow("res", res1);
  
	waitKey(0);

}