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