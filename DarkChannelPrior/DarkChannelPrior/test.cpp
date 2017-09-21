#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <fstream>
#include "dcp.hpp"

using namespace cv;
using namespace std;
Mat frame;
Mat fgMaskMOG2;
Ptr<BackgroundSubtractor> pMOG2;
int keyboard;
int count_num = 1;
void processVideo(char* videoFilename);
void processImages(char* firstFrameFilename);
//#define HARRIS
#define FLOW
//#define VIDEOPROCESS
//#define FRAMEDIFF
//#define FILELIST
int main(int argc, char* argv[]){
	
	vector<Mat> img_list,img_list_gray;
	//string filePre = "..//..//..//video//img_170721_01j//170721_01j_000";
	string filePre = "..//..//..//video//img_170721_01j//";
	string savefilePre = "..//..//..//result//";
	string file_name = "..//..//..//result//object1.jpg";
	int beg_num = 895;
	Rect valid_rect(236, 0, 1358, 1080); //the black range
	int img_width = valid_rect.width;
	int img_height = valid_rect.height;
	Size shrink_size(img_width , img_height );
	vector<Point2f> points[2];
	int count = 0;
	

#ifdef FILELIST
	const char* file_list = "..//..//..//diff//list.txt";
	char file_name[256];
	FILE *data;
	data = fopen(file_list, "r");
	
	while (!feof(data)) {
		if (fscanf(data, "%s", file_name) == NULL) break;
		Mat src=imread(file_name);
		count++;
		Mat dst;
		threshold(src, dst, 10, 255, THRESH_BINARY);
		imshow("dst", dst);
		waitKey();

	}

#endif //FILELIST

#ifdef VIDEOPROCESS
	VideoCapture cap;
	cap.open("..//..//..//video//170721_01j.mp4");
	for (int i = 0; i < 900; i++) {
		Mat frame;
		cap >> frame;
		string save_file= filePre + "img_170721_01j_" + to_string(i) + ".jpg";
		cv::imwrite(save_file, frame(valid_rect));
		cout << "saved" << save_file << endl;
	}
#endif //VIDEOPROCESS
#ifdef FRAMEDIFF
	for (int i = beg_num; i < beg_num + 3; i++) {
		Mat cur = imread(filePre + "img_170721_01j_"+to_string(i) + ".jpg");
		Mat img_gray,image,img_mix;
		Mat darkChannel(shrink_size, CV_8UC1);
		Mat brightChannel(shrink_size, CV_8UC1);
		resize(cur, image, shrink_size);
		string savefile = filePre + to_string(i) + "_re.jpg";
		//imwrite(savefile, image);
		img_list.push_back(image);
		cvtColor(image, img_gray, CV_BGR2GRAY);
		img_list_gray.push_back(img_gray);
		count++;
		if (count < 3) {
			continue;
		}
		else {
			Mat diff(shrink_size, CV_8UC1);
			Mat range(shrink_size, CV_8UC1);
			Mat output(shrink_size, CV_8UC3);
			diffFrames(img_list_gray[0], img_list_gray[1], diff);
			Mat labels, stats, centroids;
			int size= connectedComponentsWithStats(diff, labels, stats, centroids, 8, 4);
			for (int i = 0; i < size; i++) {
				cout << stats.at<int>(i, 4) << endl;
			}
			//shapeFilter(diff, labels, stats, size);
			//frameDarkChannel(img_list, output, diff);
			//colorRanges(diff, img_list[1], range);
			
			string savefile_mask = savefilePre +"img_170724_02j_" +to_string(i-2) + "_mask.jpg";
			string savefile_out = savefilePre + "img_170724_02j_"+to_string(i - 2) + "_out.jpg";
			string savefile_diff1 = savefilePre + "img_170721_01j_" + to_string(i - 2) + "_dif1.jpg";
			string savefile_diff2 = savefilePre + "img_170721_01j_" + to_string(i - 2) + "_dif2.jpg";
			Mat diff1 = img_list_gray[1] - img_list_gray[0];
			Mat diff2 = img_list_gray[1] - img_list_gray[2];
			//imshow("mask", diff);
			//imshow("output", output);
			//imwrite(savefile_mask, diff);
			//imwrite(savefile_out, output);
			imwrite(savefile_diff1, diff1);
			imwrite(savefile_diff2, diff2);
			img_list.erase(img_list.begin());
			img_list_gray.erase(img_list_gray.begin());
			waitKey(0);
		}
		//CalcDarkChannel(darkChannel, brightChannel, image, 0);
		//goodFeaturesToTrack(darkChannel, points[1], 200,0.01,20, Mat(), 3, false, 0.04);
		//img_mix = 2*brightChannel - darkChannel-img_gray;
		/*for (size_t i = 0; i < points[1].size(); i++) {
			circle(img_mix, points[1][i], 5, Scalar(0, 0, 255), 1, 8, 0);
		}*/
		//imshow("darkchannel", darkChannel);
		//imshow("brightchannel", brightChannel);
		//imshow("mix", img_mix);

		//waitKey(0);
	}
#endif //FRAMEDIFF
#ifdef HARRIS
	   string file_name = filePre + to_string(27) + ".jpg";
	    Mat src_ori = imread(file_name);
		int thresh = 200;
		int max_thresh = 255;
		Mat src,src_gray,dst,dst_norm,dst_norm_scaled;
		resize(src_ori, src, Size(img_width / 4, img_height / 4));
		cvtColor(src, src_gray, CV_BGR2GRAY);
		int blockSize = 2;
		int apertureSize = 3;
		double k = 0.04;
		cornerHarris(src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
		normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
		convertScaleAbs(dst_norm, dst_norm_scaled);

		/// Drawing a circle around corners
		for (int j = 0; j < dst_norm.rows; j++)
		{
			for (int i = 0; i < dst_norm.cols; i++)
			{
				if ((int)dst_norm.at<float>(j, i) > thresh)
				{
					circle(src, Point(i, j), 5, Scalar(0,0,255), 2, 8, 0);
				}
			}
		}
		/// Showing the result
		
		imshow("corner window", src);
		waitKey(0);
#endif //HARRIS


	return 0;
}
    
    
void processImages(char* fistFrameFilename) {
	//read the first file of the sequence
	frame = imread(fistFrameFilename);
	if (frame.empty()) {
		//error in opening the first image
		cerr << "Unable to open first image frame: " << fistFrameFilename << endl;
		exit(EXIT_FAILURE);
	}
	//current image filename
	string fn(fistFrameFilename);
	//read input data. ESC or 'q' for quitting
	while ((char)keyboard != 'q' && (char)keyboard != 27) {
		//update the background model
		pMOG2->apply(frame, fgMaskMOG2);
		//get the frame number and write it on the current frame
		size_t index = fn.find_last_of("/");
		if (index == string::npos) {
			index = fn.find_last_of("\\");
		}
		size_t index2 = fn.find_last_of(".");
		string prefix = fn.substr(0, index + 1);
		string suffix = fn.substr(index2);
		string frameNumberString = fn.substr(index + 1, index2 - index - 1);
		istringstream iss(frameNumberString);
		int frameNumber = 0;
		iss >> frameNumber;
		rectangle(frame, cv::Point(10, 2), cv::Point(100, 20),
			cv::Scalar(255, 255, 255), -1);
		putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
			FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
		//show the current frame and the fg masks
		imshow("Frame", frame);
		imshow("FG Mask MOG 2", fgMaskMOG2);
		string save_file = prefix+to_string(count_num++) + "_mask.jpg";
		imwrite(save_file, fgMaskMOG2);
		//get the input from the keyboard
		keyboard = waitKey(30);
		//search for the next image in the sequence
		ostringstream oss;
		oss << (frameNumber + 1);
		string nextFrameNumberString = oss.str();
		string nextFrameFilename = prefix + nextFrameNumberString + suffix;
		//read the next frame
		frame = imread(nextFrameFilename);
		if (frame.empty()) {
			//error in opening the next image in the sequence
			cerr << "Unable to open image frame: " << nextFrameFilename << endl;
			exit(EXIT_FAILURE);
		}
		//update the path of the current frame
		fn.assign(nextFrameFilename);
	}
}


void processVideo(char* videoFilename) {
	//create the capture object
	VideoCapture capture(videoFilename);
	if (!capture.isOpened()) {
		//error in opening the video input
		cerr << "Unable to open video file: " << videoFilename << endl;
		exit(EXIT_FAILURE);
	}
	//read input data. ESC or 'q' for quitting
	while ((char)keyboard != 'q' && (char)keyboard != 27) {
		//read the current frame
		if (!capture.read(frame)) {
			cerr << "Unable to read next frame." << endl;
			cerr << "Exiting..." << endl;
			exit(EXIT_FAILURE);
		}
		//update the background model
		pMOG2->apply(frame, fgMaskMOG2);
		//get the frame number and write it on the current frame
		stringstream ss;
		rectangle(frame, cv::Point(10, 2), cv::Point(100, 20),
			cv::Scalar(255, 255, 255), -1);
		ss << capture.get(CAP_PROP_POS_FRAMES);
		string frameNumberString = ss.str();
		putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
			FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
		//show the current frame and the fg masks
		imshow("Frame", frame);
		imshow("FG Mask MOG 2", fgMaskMOG2);
		//get the input from the keyboard
		keyboard = waitKey(30);
	}
	//delete capture object
	capture.release();
}