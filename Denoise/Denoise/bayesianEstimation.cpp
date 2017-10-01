#include "core.h"
#include <numeric>
#include <opencv2/highgui/highgui.hpp>

#define MAX_ITER 10
static void priorModel(Mat& image, Mat& diff, Point current_point,vector<double>& p,int radius,int seg_num) {
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
			
			double* pp = (double *)malloc(sizeof(double)*seg_num);
			int* seg_count = (int *)malloc(sizeof(int)*seg_num);
			for (int k = 0; k < seg_num; k++) {
				seg_count[k] = 0;
			}
			for (int r = st_row; r <= ed_row; r++) {
				for (int c = st_col; c<= ed_col; c++) {
					if (r == i&&c == j) continue;
					else{
						count++;
						int seg = diff.at<uchar>(r, c);
						seg_count[seg]++;
					}
				}
			}
			for (int k = 0; k < seg_num; k++) {
				 pp[k] = seg_count[k] == 0 ? 0.001 : (double)seg_count[k] / (double)count;
				p.push_back(pp[k]);
			}
			free(pp);
			free(seg_count);

}

static void gaussianPram(vector<int>& list, double& mean, double& var) {
	
		if (list.size() == 0) {
			mean = 0;
			var = 0.1;
		}
		else {
			double sum_0 = std::accumulate(list.begin(), list.end(), 0.0);
			mean = sum_0 / list.size();
			double sq_sum_0 = std::inner_product(list.begin(), list.end(), list.begin(), 0.0);
			var = std::sqrt(sq_sum_0 / list.size() - mean*mean);
		}
	
}

static void likelihoodModel(Mat& image, Mat& diff, Point current_point,vector<double>& p, int radius, int num) {
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
			vector<vector<int>> seg_b(num);
			vector<vector<int>> seg_g(num);
			vector<vector<int>> seg_r(num);

			for (int r = st_row; r <= ed_row; r++) {
				for (int c = st_col; c <= ed_col; c++) {
					if (r == i&&c == j) continue;
					else{
						int label = diff.at<uchar>(r, c);
						seg_b[label].push_back(image.at<Vec3b>(r, c)[0]);
						seg_g[label].push_back(image.at<Vec3b>(r, c)[1]);
						seg_r[label].push_back(image.at<Vec3b>(r, c)[2]);
					}
				}
			}
			double **mean = (double **)malloc(sizeof(double)*num);
			double **var = (double **)malloc(sizeof(double*)*num);
			double **pl = (double **)malloc(sizeof(double*)*num);
			Vec3b cur_val = image.at<Vec3b>(current_point);
			for (int k = 0; k < num; k++) {
				mean[k] = (double *)malloc(sizeof(double) * 3);
				var[k] = (double *)malloc(sizeof(double) * 3);
				pl[k] = (double *)malloc(sizeof(double) * 3);
				gaussianPram(seg_b[k], mean[k][0], var[k][0]);
				gaussianPram(seg_g[k], mean[k][1], var[k][1]);
				gaussianPram(seg_r[k], mean[k][2], var[k][2]);
				for (int ch = 0; ch < 3; ch++) {
					
					var[k][ch] = var[k][ch] < 1 ? 1 : var[k][ch]; /*avoid 0*/
					pl[k][ch] = exp(-(cur_val[ch] - mean[k][ch])*(cur_val[ch] - mean[k][ch]) / (2 * var[k][ch] * var[k][ch])) / (var[k][ch] * sqrt(2 * CV_PI));
					pl[k][ch] = pl[k][ch] < 0.001 ? 0.001 : pl[k][ch]; //avoid 0
					//p.push_back(pl[k][0]+ pl[k][1]+pl[k][2]);
					
				}
				p.push_back(pl[k][2]);
				//p.push_back(pl[k][0]*pl[k][1]*pl[k][2]);
			}
			free(mean);
			free(var);
			free(pl);
}

void bayesianEstimation(Mat& image, Mat& labels_init, Mat& labels_estimation,int seg_num, int max_iter, int radius) {
	int iter = 0;
	int height = image.rows;
	int width = image.cols;
	labels_init.copyTo(labels_estimation);

	while (iter < max_iter) {
		
		for (int i = 3; i < height; i++) {
			for (int j = 4; j < width; j++) {
				Point cur(j, i);
				vector<double>pp, pl;
				priorModel(image, labels_estimation, cur, pp, radius, seg_num);
				likelihoodModel(image, labels_estimation, cur, pl, radius, seg_num);
				double max = log(pp[0]) + log(pl[0]);
				int label_val = 0;
				for (int k = 1; k < seg_num; k++) {
					if (log(pp[k]) + log(pl[k]) > max) {
						max = log(pp[k]) + log(pl[k]);
						label_val = k;
					}
				}
				labels_estimation.at<uchar>(i, j) = label_val;
				//cout << "label: " << label_val << endl;
			}
		}
		iter++;
		//cvResizeWindow("iter result", width, height);
		//imshow("iter result", labels_estimation);
		//waitKey(0);
	}



}

void labelInitByDiff(vector<Mat>& diff_wb, Mat& label_init) {
	int width = diff_wb[0].cols;
	int height = diff_wb[0].rows;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (diff_wb[0].at<uchar>(i, j) == 255 && diff_wb[1].at<uchar>(i, j)==0) {
				label_init.at<uchar>(i, j) = 1; //object
			}
			else if (diff_wb[0].at<uchar>(i, j) == 0 && diff_wb[1].at<uchar>(i, j) == 255) {
				label_init.at<uchar>(i, j) = 1;
			}
			else if (diff_wb[0].at<uchar>(i, j) == 0 && diff_wb[1].at<uchar>(i, j) == 0) {
				label_init.at<uchar>(i, j) = 0; //background
			}
			else {
				label_init.at<uchar>(i, j) = 2; //snow
			}
		}
	}
}

void labelInitByRedDarkChannel(Mat& red_dark, Mat& label_init) {
	int width = red_dark.cols;
	int height = red_dark.rows;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (red_dark.at<uchar>(i, j) == 255) {
				label_init.at<uchar>(i, j) = 1;
			}
		}
	}
}