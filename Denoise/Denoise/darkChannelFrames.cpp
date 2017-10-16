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

int sumAreaByRadius(Mat& label_pre,Mat& label_next,Mat& centroids_pre,Mat& centroids_next, Mat& sum1, int radius) {
	int height = label_pre.rows;
	int width =label_pre.cols;
	Mat sum(height, width, CV_8UC1, Scalar(0));
	int st_row, ed_row;
	int st_col, ed_col;
	
	vector<int> list_0;
	vector<int> list_1;
	for (int k = 1; k < centroids_pre.rows; k++) {
		int i = centroids_pre.at<double>(k, 1);
		int j = centroids_pre.at<double>(k, 0);
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
				if (label_next.at<int>(m, n) != 0) {
					int label = label_next.at<int>(m, n);
					double dis = (centroids_next.at<double>(label, 1) - i)*(centroids_next.at<double>(label, 1) - i) + (centroids_next.at<double>(label, 0) - j)*(centroids_next.at<double>(label, 0) - j);
					if (dis < radius*radius) {

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
			int label_0 = label_pre.at<int>(i, j);
			int label_1 = label_next.at<int>(i, j);
			vector<int>::iterator iter_0 = find(list_0.begin(), list_0.end(), label_0);
			vector<int>::iterator iter_1 = find(list_1.begin(), list_1.end(), label_1);
			if (iter_0 != list_0.end()) {
				sum.at<uchar>(i,j) = 255;
				num++;
			}

			if (iter_1 != list_1.end()) {
				sum.at<uchar>(i, j) = 255;
				num++;
			}
		}
	}
	sum1 = sum;
	return num;

}

void restorationBaseMationBrightEM(vector<Mat>& image_list_compensation,vector<Mat>& image_list_gray, Mat& label, vector<Ptr<EM>>& models,Mat& output) {
	int width = image_list_compensation[0].cols;
	int height = image_list_compensation[0].rows;

	image_list_compensation[1].copyTo(output);
	struct max_probs_labels {
		Point pos;
		int frame_num;
		float prob;
	};
	vector<int> flag;
	vector<max_probs_labels> label_max(models.size());
	vector<Point> need_to_repair;
	for (int k = 0; k < models.size(); k++) {
		Mat means = models[k]->getMeans();
		int flag_tmp = means.at<double>(0, 0) > means.at<double>(1, 0) ? 1 : 0;
		flag.push_back(flag_tmp);
	}

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int label_num = label.at<int>(i, j);
			if (label_num > 0) {
				float max = 0.;
				int frame_num = 1;
				label_max[label_num - 1].prob = 0.5;
				for (int k = 0; k < 3; k++) {
					vector<float> probs;
					models[label_num-1]->predict2(image_list_gray[k].at<uchar>(i, j), probs);
					if (probs[flag[label_num-1]] > max) {
						max = probs[flag[label_num-1]];				
						frame_num = k;
					}
					if (probs[flag[label_num - 1]] > label_max[label_num - 1].prob) {
						label_max[label_num - 1].frame_num = k;
						label_max[label_num - 1].pos = Point(j, i);
						label_max[label_num - 1].prob = probs[flag[label_num - 1]];
					}
				}
				//cout << "probs " << max << " frame "<< frame_num<< endl;
				if (max > 0) {
					output.at<Vec3b>(i, j) = image_list_compensation[frame_num].at<Vec3b>(i, j);
				}
				else {
					//output.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
					need_to_repair.push_back(Point(j, i));
				}
			}
		}
	}
	if (need_to_repair.size() > 0) {
		for (int k = 0; k < need_to_repair.size(); k++) {
			int label_num = label.at<int>(need_to_repair[k]);
			output.at<Vec3b>(need_to_repair[k]) = image_list_compensation[label_max[label_num - 1].frame_num].at<Vec3b>(label_max[label_num - 1].pos);
		}
	}


}