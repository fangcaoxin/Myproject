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
int main(int argc, char* argv[]){
    


 //  const char *file_name="..//..//img//result_2.jpg";
 //   IplImage *input=cvLoadImage(file_name);
 //   IplImage *output=cvCreateImage(cvGetSize(input), IPL_DEPTH_8U, 3);
	//IplImage *img_hsv = cvCreateImage(cvGetSize(input), IPL_DEPTH_8U, 3);
	//
	//
	//
 //   dehaze(output, input);
 //  int width=input->width;
 //  int height=input->height;
 //  Mat output_dehaze = cvarrToMat(output);
 //  imwrite("result_dehaze.jpg", output_dehaze);
   
   
	
	


    //cvReleaseImage(&output);
    //cvReleaseImage(&input);
 
 
 
	vector<Mat> img_list;
	string filePre = "..//..//..//video//img_170721_01j//170721_01j_00";
	string savefilePre = "..//..//..//result//";
	int beg_num = 11;
	Rect valid_rect(226, 0, 1468, 1080); //the black range
	int img_width = valid_rect.width;
	int img_height = valid_rect.height;
	for (int i = beg_num; i < beg_num + 3; i++) {
		Mat cur = imread(filePre + to_string(i) + "rect.jpg");
		if (cur.cols == img_width) {
			img_list.push_back(cur);
		}
		else {
			img_list.push_back(cur(valid_rect));
		}
	}
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

		//imwrite(filePre + to_string(i) + "rect.jpg",cur(valid_rect));
	
	
	//Mat output(img_height, img_width,CV_8UC3);
	//Mat frameNumOutput(img_height, img_width, CV_8UC3);
	//FrameDiff(img_list, output,frameNumOutput);
	//string savefile = filePre +  to_string(beg_num) + "out.jpg";
	////imwrite(savefile, output);
	//string savefile_frame = filePre + to_string(beg_num) + "num.jpg";
 //   imwrite(savefile_frame, frameNumOutput);

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