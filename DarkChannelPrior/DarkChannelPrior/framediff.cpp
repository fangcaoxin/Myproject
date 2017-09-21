#include "dcp_core.h"

using std::sort;
void FrameDiff(vector<Mat> input, Mat& output,Mat& frameNumOutput) {
	
	int size = input.size();
	int width = input[0].cols;
	int height = input[0].rows;
	vector<Mat> input_gray;
	for (int i = 0; i < size; i++) {
		Mat cur;
		cvtColor(input[i], cur, CV_BGR2GRAY);
		input_gray.push_back(cur);
	}
	
	//uchar* val = (uchar*)malloc(size * sizeof(uchar));
	
		for (int m = 0; m < height; m++) {
			for (int n = 0; n < width; n++) {
				uchar min = 255;
				int layer = 0;
				Vec3b frameVal;
				frameVal.val[0] = 0;
				frameVal.val[1] = 0;
				frameVal.val[2] = 0;
				for (int k = 0; k < size; k++) {
					uchar cur = input_gray[k].at<uchar>(m, n);
					if (cur < min) {
						min = cur;
						layer = k;
					}
					
				}
				output.at<Vec3b>(m, n) = input[layer].at<Vec3b>(m, n);
				frameVal.val[layer] = 255;
				frameNumOutput.at<Vec3b>(m, n) = frameVal;
			}
			
		}
		
}

void diffFrames(Mat& pre_gray, Mat& gray, Mat& mask) {
	int width = pre_gray.cols;
	int height = pre_gray.rows;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int cur_diff = gray.at<uchar>(i, j) - pre_gray.at<uchar>(i, j);
			if (cur_diff > 10) {
				mask.at<uchar>(i, j) = 255;
			}
			else {
				mask.at<uchar>(i, j) = 0;
			}
		}
	}
}

void colorRanges(Mat& mask, Mat& img, Mat& range) {
	int width = img.cols;
	int height = img.rows;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (mask.at<uchar>(i, j) == 255) {
				range.at<uchar>(i, j) = (abs(img.at<Vec3b>(i, j)[0] - img.at<Vec3b>(i, j)[1]) + \
					abs(img.at<Vec3b>(i, j)[0] - img.at<Vec3b>(i, j)[2]) + \
					abs(img.at<Vec3b>(i, j)[1] - img.at<Vec3b>(i, j)[2])) ;
				cout << "range is " << (int)range.at<uchar>(i, j) << endl;
			}
			else {
				range.at<uchar>(i, j) = 0;
			}
		}
	}

}

void shapeFilter(Mat& diff, const Mat& labels,const Mat& stats,int size) {
	vector<int> big_area_labels;
	int width = diff.cols;
	int height = diff.rows;
	for (int i = 1; i < size; i++) {
		int area = stats.at<int>(i, 4);
		int box_width = stats.at<int>(i, 2);//box width
		int box_height = stats.at<int>(i, 3); //box height
		if (area > 1000||area<4) {
			big_area_labels.push_back(i);
		}

		if ((float)box_width / (float)box_height > 4 || (float)box_width /(float) box_height < 0.25) {
			big_area_labels.push_back(i);
		}
	}

	for (int m = 0; m < height; m++) {
		for (int n = 0; n < width; n++) {
			for (int k = 0; k < big_area_labels.size(); k++) {
				int label_num = labels.at<int>(m, n);
				if (label_num == big_area_labels[k]) {
					diff.at<uchar>(m, n) = 0;
					break;
				}
			}
		}
	}
	
}


void frameDarkChannel(vector<Mat>& img_list, Mat& output, Mat& mask) {
	int size = img_list.size();
	int width = img_list[0].cols;
	int height = img_list[0].rows;
	vector<Mat> input_gray;
	for (int i = 0; i < size; i++) {
		Mat cur;
		cvtColor(img_list[i], cur, CV_BGR2GRAY);
		input_gray.push_back(cur);
	}

	//uchar* val = (uchar*)malloc(size * sizeof(uchar));

	for (int m = 0; m < height; m++) {
		for (int n = 0; n < width; n++) {
			
			int layer = 1;
			int min = input_gray[1].at<uchar>(m, n);
			Vec3b frameVal = { 0,0,0 };
			if (mask.at<uchar>(m, n) == 255) {
				/*for (int k = 0; k < size; k++) {
					uchar cur = input_gray[k].at<uchar>(m, n);
					if (cur < min) {
						min = cur;
						layer = k;
					}
				}*/
				output.at<Vec3b>(m, n) = img_list[0].at<Vec3b>(m, n)/2+ img_list[2].at<Vec3b>(m, n)/2;
			}
			else {
				layer = 1;
				output.at<Vec3b>(m, n) = img_list[layer].at<Vec3b>(m, n);
			}
			
			
			
		}

	}

}
