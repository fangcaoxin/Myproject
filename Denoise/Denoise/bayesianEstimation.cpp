#include "core.h"
#include <numeric>
#include <opencv2/highgui/highgui.hpp>

#define MAX_ITER 10
static void priorModel(Mat& image, Mat& diff, Point current_point,vector<float>& p,int radius,int seg_num) {
	int height = image.rows;
	int width = image.cols;

	int st_row, ed_row;
	int st_col, ed_col;

	const int beta1 = 1;
	const int beta2 = -1;

	int i = current_point.y;
	int j = current_point.x;
	
	
			st_row = i - radius, ed_row = i + radius;
			st_col = j - radius, ed_col = j + radius;

			st_row = st_row < 0 ? 0 : st_row;
			ed_row = ed_row >= height ? (height - 1) : ed_row;
			st_col = st_col < 0 ? 0 : st_col;
			ed_col = ed_col >= width ? (width - 1) : ed_col;

			int count = 0;
			
			
			int* seg_count = (int *)malloc(sizeof(int)*seg_num);
			for (int k = 0; k < seg_num; k++) {
				seg_count[k] = 0;
			}
			for (int r = st_row; r <= ed_row; r++) {
				for (int c = st_col; c<= ed_col; c++) {
					if (r == i&&c == j) continue;
					else{
						count++;
						int seg = diff.at<uchar>(r, c) == 255 ? 0 : 1;
						seg_count[seg]++;
					}
				}
			}
			float p_0 = seg_count[0] == 0 ? 0.001 : (float)seg_count[0] / (float)count;
			float p_1 = seg_count[1] == 0 ? 0.001 : (float)seg_count[1] / (float)count;
			
			
			
			p.push_back(p_0);
			p.push_back(p_1);

}

static void gaussianPram(vector<int>& list, float& mean, float& var) {
	if (list.size() == 0) {
		mean = 0;
		var = 0.1;
	}
	else {
		float sum_0 = std::accumulate(list.begin(), list.end(), 0.0);
		mean = sum_0 / list.size();
		float sq_sum_0 = std::inner_product(list.begin(), list.end(), list.begin(), 0.0);
		var = std::sqrt(sq_sum_0 / list.size() - mean*mean);
	}
}

static void likelihoodModel(Mat& image, Mat& diff, Point current_point,vector<float>& p, int radius, int num) {
	int height = image.rows;
	int width = image.cols;

	int st_row, ed_row;
	int st_col, ed_col;

	int i = current_point.y;
	int j = current_point.x;

	
			st_row = i - radius, ed_row = i + radius;
			st_col = j - radius, ed_col = j + radius;

			st_row = st_row < 0 ? 0 : st_row;
			ed_row = ed_row >= height ? (height - 1) : ed_row;
			st_col = st_col < 0 ? 0 : st_col;
			ed_col = ed_col >= width ? (width - 1) : ed_col;
			vector<int> seg_0, seg_1;

			for (int r = st_row; r <= ed_row; r++) {
				for (int c = st_col; c <= ed_col; c++) {
					if (r == i&&c == j) continue;
					else{
						if (diff.at<uchar>(r, c) == 255) {
							seg_0.push_back(image.at<uchar>(r, c));
						}
						else {
							seg_1.push_back(image.at<uchar>(r, c));
						}
					}
				}
			}
			float mean_0=0., mean_1=0., var_0=0., var_1=0.;
			gaussianPram(seg_0, mean_0, var_0);
			gaussianPram(seg_1, mean_1, var_1);
			int cur_val = image.at<uchar>(current_point);
			float p0 = exp(-(cur_val - mean_0)*(cur_val - mean_0) / (2 * var_0*var_0)) / (var_0*sqrt(2 * CV_PI));
			float p1= exp(-(cur_val - mean_1)*(cur_val - mean_1) / (2 * var_1*var_1)) / (var_1*sqrt(2 * CV_PI));
			p.push_back(p0);
			p.push_back(p1);
}

void bayesianEstimation(Mat& image, Mat& labels_init, Mat& labels_estimation,int seg_num, int max_iter, int radius) {
	int iter = 0;
	int height = image.rows;
	int width = image.cols;
	labels_init.copyTo(labels_estimation);

	while (iter < max_iter) {
		
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				Point cur(j, i);
				vector<float>pp, pl;
				priorModel(image, labels_estimation, cur, pp, radius, seg_num);
				likelihoodModel(image, labels_estimation, cur, pl, radius, seg_num);
				int label_val = log(pp[0]) + log(pl[0]) > log(pp[1]) + log(pl[1]) ? 255 : 0;
				labels_estimation.at<uchar>(i, j) = label_val;
			}
		}
		iter++;
		//cvResizeWindow("iter result", width, height);
		//imshow("iter result", labels_estimation);
		//waitKey(0);
	}



}

