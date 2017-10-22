#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <fstream>
#include "dehaze.h"

using namespace cv;
using namespace std;
#define NEW
//#define OLD
//#define LIST
int main(int argc, char* argv[]){
	int beg_no = 0;
#ifdef NEW
	string folder_name = "..//..//..//image//img_171021//171021_capture_";
	string filename = folder_name + to_string(beg_no) + ".jpg";
#endif //NEW
#ifdef OLD
	//string folderName = "img_170721_01j";
	string folderName = "img_170724_02j";
	string folder_name="..//..//..//image//" + folderName + "//" + folderName + "_";
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
	//string filename = "city.png";
	Mat image = imread(filename);
	vector<Mat> channels;
	split(image, channels);
	Mat res;
	dehaze(image, res);
	imshow("original", image);
	imshow("res", res);
	imshow("red", channels[2]);
	waitKey(0);
	
}