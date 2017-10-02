#include "core.h"
#define USE_FLOW

void darkChannelFrames(const vector<Mat> &imgList, int frameNum, Mat& ouput, const vector<Mat>& flow) {
	vector<Mat> imgListGray;

	for (int i = 0; i < frameNum; i++) {
		Mat cur;
		cvtColor(imgList[i], cur, CV_BGR2GRAY);
		imgListGray.push_back(cur);
	}

	int width = imgList[0].cols;
	int height = imgList[0].rows;
	
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int darkFrameNum = 0;
			int min_value = imgListGray[0].at<uchar>(i, j);
			//int min_value = 255;
			Vec3b frameNumvalue;

			frameNumvalue.val[0] = 0;
			frameNumvalue.val[1] = 0;
			frameNumvalue.val[2] = 0;

			for (int k = 1; k < frameNum; k++) {
				Point2i cur_pos;
				cur_pos.x = j;
				cur_pos.y = i;



#ifdef USE_FLOW
			if (norm(flow[k - 1].at<Point2f>(cur_pos)) < 0.1) {
					int cur_value = imgListGray[k].at<uchar>(cur_pos);
					if (cur_value < min_value) {
						min_value = cur_value;
						darkFrameNum = k;
					}
				}
				else {
					darkFrameNum = 0;
				}

			}





#else
				int cur_value = imgListGray[k].at<uchar>(cur_pos);
				if (cur_value < min_value) {
					min_value = cur_value;
					darkFrameNum = k;
				}
				/*cur_pos.x = j + flow[k-1].at<Point2f>(i,j).x;
				cur_pos.y = i + flow[k-1].at<Point2f>(i,j).y;
				cur_pos.x = cur_pos.x < 0 ? 0 : cur_pos.x;
				cur_pos.x = cur_pos.x > width-1 ? width-1 : cur_pos.x;
				cur_pos.y = cur_pos.y < 0 ? 0 : cur_pos.y;
				cur_pos.y = cur_pos.y > height-1 ? height-1 : cur_pos.y;*/
#endif //USE_FLOW

				ouput.at<Vec3b>(i, j) = imgList[darkFrameNum].at<Vec3b>(i, j);

		}
	}
}

int sumAreaByRadius(vector<Mat>& diff_wb, Mat& sum, int radius) {
	int size = diff_wb.size();
	int height = diff_wb[0].rows;
	int width = diff_wb[0].cols;
	vector<Mat> diff_1(size);
	vector<Mat> labels(size);
	vector<Mat> stats(size);
	vector<Mat> centroids(size);
	for (int i = 0; i < size; i++) {
	   connectedComponentsWithStats(diff_wb[i], labels[i], stats[i], centroids[i], 8, 4);
	}
	int st_row, ed_row;
	int st_col, ed_col;
	
	vector<int> list_0;
	vector<int> list_1;
	for (int k = 1; k < centroids[0].rows; k++) {
		int i = centroids[0].at<double>(k, 1);
		int j = centroids[0].at<double>(k, 0);
		st_row = i - radius, ed_row = i + radius;
		st_col = j - radius, ed_col = j + radius;

		st_row = st_row < 0 ? 0 : st_row;
		ed_row = ed_row >= height ? (height - 1) : ed_row;
		st_col = st_col < 0 ? 0 : st_col;
		ed_col = ed_col >= width ? (width - 1) : ed_col;

		for (int m = st_row; m <= ed_row; m++)
		{
			for (int n = st_col; n <= ed_col; n++)
			{
				if (labels[1].at<int>(m, n) != 0) {
					int label = labels[1].at<int>(m, n);
					double dis = (centroids[1].at<double>(label, 1) - i)*(centroids[1].at<double>(label, 1) - i) + (centroids[1].at<double>(label, 0) - j)*(centroids[1].at<double>(label, 0) - j);
					if (dis < radius*radius&&stats[1].at<int>(label, 4) < width*height*0.003&&stats[0].at<int>(k, 4) < width*height*0.003) {

						list_0.push_back(k);
						list_1.push_back(label);
					}
				}
			}
		}
	}
	int num = 0;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int label_0 = labels[0].at<int>(i, j);
			int label_1 = labels[1].at<int>(i, j);
			vector<int>::iterator iter_0 = find(list_0.begin(), list_0.end(), label_0);
			vector<int>::iterator iter_1 = find(list_1.begin(), list_1.end(), label_1);
			if (iter_0 != list_0.end() || iter_1 != list_1.end()) {
				sum.at<uchar>(i,j) = 1;
				num++;
			}
		}
	}
	return num;

}