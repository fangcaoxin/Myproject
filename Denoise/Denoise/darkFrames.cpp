#include "core.h"
#include "utility.h"
#define USE_FLOW 

using std::sort;

template<typename T>
T getMedian(const std::vector<T>& values)
{
	std::vector<T> copy(values);
	return getMedianAndDoPartition(copy);
}

template<typename T>
T getMedianAndDoPartition(std::vector<T>& values)
{
	size_t size = values.size();
	if (size % 2 == 0)
	{
		std::nth_element(values.begin(), values.begin() + size / 2 - 1, values.end());
		T firstMedian = values[size / 2 - 1];

		std::nth_element(values.begin(), values.begin() + size / 2, values.end());
		T secondMedian = values[size / 2];

		return (firstMedian + secondMedian) / (T)2;
	}
	else
	{
		size_t medianIndex = (size - 1) / 2;
		std::nth_element(values.begin(), values.begin() + medianIndex, values.end());

		return values[medianIndex];
	}
}
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

void medianFramesByMask(Mat& image,  Mat& stats, vector<int>& valid_labels) {
	for (int i = 0; i < valid_labels.size(); i++) {
		Rect rect(stats.at<int>(valid_labels[i], 0), stats.at<int>(valid_labels[i], 1), stats.at<int>(valid_labels[i], 2), stats.at<int>(valid_labels[i], 3));
		int left_top_x = rect.x-rect.width;
		int left_top_y= rect.y-rect.height;
		int right_bottom_x = left_top_x +  3*rect.width;
		int right_bottom_y=left_top_y+  3*rect.height;
		left_top_x = left_top_x < 0 ? 0 : left_top_x;
		left_top_y = left_top_y < 0 ? 0 : left_top_y;
		right_bottom_x = right_bottom_x > image.cols - 1 ? image.cols - 1 : right_bottom_x;
		right_bottom_y = right_bottom_y > image.rows - 1 ? image.rows - 1 : right_bottom_y;
		Rect new_rect(left_top_x, left_top_y, right_bottom_x - left_top_x, right_bottom_y - left_top_y);
		int ksize = MAX(rect.width, rect.height)*1.5;
		ksize = ksize % 2 == 0 ? ksize + 1 : ksize;
		medianBlur(image(new_rect), image(new_rect),ksize);
		/*int st_row, ed_row;
		int st_col, ed_col;
		
		vector<int> b_vals, g_vals, r_vals;
		for (int i= left_top_y; i< right_bottom_y;i++) {
			for (int j = left_top_x; j < right_bottom_y ; j++) {

				st_row = i - rect.height, ed_row = i + rect.height;
				st_col = j - rect.width, ed_col = j + rect.width;

				st_row = st_row < 0 ? 0 : st_row;
				ed_row = ed_row >= image.rows ? (image.rows - 1) : ed_row;
				st_col = st_col < 0 ? 0 : st_col;
				ed_col = ed_col >= image.cols ? (image.cols - 1) : ed_col;

				for (int m = st_row; m <= ed_row; m++)
				{
					for (int n = st_col; n <= ed_col; n++)
					{
						b_vals.push_back(image.at<Vec3b>(m, n)[0]);
						g_vals.push_back(image.at<Vec3b>(m, n)[1]);
						r_vals.push_back(image.at<Vec3b>(m, n)[2]);
					}
				}
				int b_val = getMedian(b_vals);
				int g_val = getMedian(g_vals);
				int r_val = getMedian(r_vals);
				Vec3b new_val(b_val, g_val, r_val);
				image.at<Vec3b>(i, j) = new_val;
			}
			
		}*/
		
		
		

	}

	
}