#include "core.h"
#define USE_FLOW 
static Point2i matMultipyPoint(Point2i& input_point, Mat Homograhy) {
	Point2i result = { 0 };
	//double a = Homograhy.at<double>(0,0);
	double temp_x = Homograhy.at<double>(0,0) * input_point.x + Homograhy.at<double>(0, 1) * input_point.y + Homograhy.at<double>(0, 2);
	double temp_y= Homograhy.at<double>(1, 0) * input_point.x + Homograhy.at<double>(1, 1) * input_point.y + Homograhy.at<double>(1, 2);
	double temp_w = Homograhy.at<double>(2, 0) * input_point.x + Homograhy.at<double>(2, 1) * input_point.y + Homograhy.at<double>(2, 2);
	if (temp_w <DBL_EPSILON) {
		return input_point;
	}
	else {
		result.x = temp_x / temp_w;
		result.y = temp_y / temp_w;
		return result;
	}
}
void darkFrames(const vector<Mat>& imagelist, Mat& output, const vector<Mat>& homography) {
	int height = imagelist[0].rows;
	int width = imagelist[0].cols;
	int frameNum = imagelist.size();
	vector<Mat> imgListGray;
	for (int i = 0; i < frameNum; i++) {
		Mat cur;
		cvtColor(imagelist[i], cur, CV_BGR2GRAY);
		imgListGray.push_back(cur);
	}
	cout << "homography[0] is " << homography[0] << endl;
	cout << "homography[1] is " << homography[1] << endl;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int darkFrameNum = 0;
			int min_value = imagelist[0].at<uchar>(i, j);
			Vec3b frameNumvalue;

			frameNumvalue.val[0] = 0;
			frameNumvalue.val[1] = 0;
			frameNumvalue.val[2] = 0;
			
			Point2i cur_pos = { 0 }, res_pos = {0};
			for (int k = 1; k < frameNum; k++) {
				
				cur_pos.x = j;
				cur_pos.y = i;
#ifdef USE_FLOW
				/*vector<Point2i> cur_pos_vec(1);
				vector<Point2i> cur_pos_vec_trans(1);
				cur_pos_vec[0] = cur_pos;*/
				cur_pos = matMultipyPoint(cur_pos, homography[k-1]);
				
				
				cur_pos.x = cur_pos.x < 0 ? 0 : cur_pos.x;
				cur_pos.x = cur_pos.x > width - 1 ? width - 1 : cur_pos.x;
				cur_pos.y = cur_pos.y < 0 ? 0 : cur_pos.y;
				cur_pos.y = cur_pos.y > height - 1 ? height - 1 : cur_pos.y;
#endif //USE_FLOW
				int cur_value = imagelist[k].at<uchar>(cur_pos);
				if (cur_value < min_value) {
					min_value = cur_value;
					darkFrameNum = k;
				}
			}
			/*if (darkFrameNum > 0) {
			cout << "darknum is :" << darkFrameNum << endl;
			}*/
			//frameNumvalue.val[darkFrameNum] = 255;
		
			if (darkFrameNum == 0) {
				res_pos.x = j;
				res_pos.y = i;
			}
			else {

				res_pos.x = j;
				res_pos.y = i;
#ifdef USE_FLOW	
				/*vector<Point2i> res_pos_vec(1);
				vector<Point2i> res_pos_vec_trans(1);
				res_pos_vec[0] = resPoint;*/
				res_pos = matMultipyPoint(res_pos, homography[darkFrameNum-1]);
				
				res_pos.x = res_pos.x < 0 ? 0 : res_pos.x;
				res_pos.x = res_pos.x > width - 1 ? width - 1 : res_pos.x;
				res_pos.y = res_pos.y < 0 ? 0 : res_pos.y;
				res_pos.y = res_pos.y > height - 1 ? height - 1 : res_pos.y;
				
				
#endif //USE_FLOW

			}
			output.at<Vec3b>(i, j) = imagelist[darkFrameNum].at<Vec3b>(res_pos);
			
			/*cout << "point is " << i<< " " << j <<
			"cal point is "<<resPoint.y<<" "<<resPoint.x << endl;*/
		}
	}
}
#define DARK

void darkFramesByMask(vector<Mat>& imagelist, Mat& output, Mat& Mask) {
	int height = imagelist[0].rows;
	int width = imagelist[0].cols;
	int frameNum = imagelist.size();
	vector<Mat> imgListGray;
	for (int i = 0; i < frameNum; i++) {
		Mat cur;
		cvtColor(imagelist[i], cur, CV_BGR2GRAY);
		imgListGray.push_back(cur);
	}
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int dark_frame_num = frameNum / 2;
			if (Mask.at<uchar>(i, j) == 0) {
				output.at<Vec3b>(i, j) = imagelist[frameNum/2].at<Vec3b>(i, j);
			}
			else {
				
#ifdef DARK
				int min = 255;
			
				for (int k = 0; k < frameNum; k++) {
					if (imgListGray[k].at<uchar>(i, j) < min) {
						min = imgListGray[k].at<uchar>(i, j);
						dark_frame_num = k;
					}
				}
				//dark_frame_num = imgListGray[0].at<uchar>(i, j) > imgListGray[2].at<uchar>(i, j) ? 2 : 0;
				
#else
				Vec3b sum = { 0,0,0 };
				int frame_num = 0;
				sum = imagelist[0].at<Vec3b>(i, j) + imagelist[2].at<Vec3b>(i, j);
				output.at<Vec3b>(i, j) = sum / 2;
#endif //DARK
			}
#ifdef DARK
			output.at<Vec3b>(i, j) = imagelist[dark_frame_num].at<Vec3b>(i, j);
#endif //DARK
		}
	}
}

void shapeFilter(Mat& diff_wb, Mat& labels, Mat& stats,int size,vector<int>& valid_labels) {
	int height = diff_wb.rows;
	int width = diff_wb.cols;
	vector<int> outlier;
	for (int k = 1; k < size; k++) {
		int area = stats.at<int>(k, 4);
		float area_width = stats.at<int>(k, 2);
		float area_height = stats.at<int>(k, 3);
		if (area > (float)height*(float)width/(10*50) || area < 4) {
			outlier.push_back(k);
		}else if (area_width*area_height>3*area) {
			outlier.push_back(k);
		}
		else {
			valid_labels.push_back(k);
		}
	}
	
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int label = labels.at<int>(i, j);
			vector<int>::const_iterator index = find(outlier.begin(), outlier.end(), label);
			
				if (index != outlier.end()) {
					diff_wb.at<uchar>(i, j) = 0;
					labels.at<int>(i, j) = 0;
					
				}
			}
		}
	}


void distributeFilter(Mat& diff_wb, Mat& labels,Mat& stats, Mat& gray, vector<int>& valid_labels, vector<int>& valid_labels2) {
	int size = valid_labels.size();
	int width = diff_wb.cols;
	int height = diff_wb.rows;
	//vector<uchar> *shapes = NULL;
	vector <vector<uchar>> shapes(size);
	vector<int> area(size);
	//shapes=(vector<uchar> *)malloc(size * sizeof(vector<uchar>));

	for (int m = 0; m < height; m++) {
		for (int n = 0; n < width; n++) {
			int label_valid = labels.at<int>(m, n);
			if (label_valid == 0) {
				continue;
			}
			else {
				vector<int>::const_iterator index = find(valid_labels.begin(), valid_labels.end(), label_valid);
				int shapes_index = index-valid_labels.begin();
				shapes[shapes_index].push_back(gray.at<uchar>(m, n));
				area[shapes_index] = stats.at<int>(index[0], 4);
			}
		}
	}

	/**calucate the distribution*/
	double *variance = (double *)malloc(size * sizeof(double));
	for (int i = 0; i < size; i++) {
		
		double sum = 0, sum2 = 0;

		for (int j = 0; j < shapes[i].size(); j++) {
			sum += shapes[i][j];
			sum2 += shapes[i][j] * shapes[i][j];
		}
		variance[i] = sum2 / shapes[i].size() - (sum / shapes[i].size())*(sum / shapes[i].size());
		cout << "variance/area is: " << variance[i]/area[i]<<endl;
		if (variance[i]/(double)area[i] >0.2) {
			valid_labels2.push_back(valid_labels[i]);
		}
	}
	
}

void maskRefinement(Mat& diff_wb, Mat& labels, Mat& gray, vector<int>& valid_labels2) {
	int size = valid_labels2.size();
	int width = diff_wb.cols;
	int height = diff_wb.rows;
	for (int m = 0; m < height; m++) {
		for (int n = 0; n < width; n++) {
			int label_num = labels.at<int>(m, n);
			vector<int>::const_iterator index = find(valid_labels2.begin(), valid_labels2.end(), label_num);
			if (index == valid_labels2.end()) {
				diff_wb.at<uchar>(m, n) = 0;
			}
			
		}
	}
}

