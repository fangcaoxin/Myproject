#include "utility.h"

void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
	double, const Scalar& color) {
	for (int y = 0; y < cflowmap.rows; y += step)
		for (int x = 0; x < cflowmap.cols; x += step)
		{
			const Point2f& fxy = flow.at<Point2f>(y, x);
			if (norm(fxy) > 0) {
				line(cflowmap, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
					color);
				circle(cflowmap, Point(x, y), 1, color, -1);
			}
		}
}

void calcOptDirection(const Mat& flow, int& oritention, Point2f& aver) {
	struct val {
		int ori;
		Point2f point;
	};
	struct val *histval = NULL;
	if (histval == NULL) {
		histval = (struct val*)malloc(360 * sizeof(struct val));
	}
	for (int i = 0; i < 360; i++) {
		histval[i] = { 0 };
	}
	for (int y = 0; y < flow.rows; y++) {
		for (int x = 0; x < flow.cols; x++) {
			const Point2f& fxy = flow.at<Point2f>(y, x);
			int cur_ori = (int)fastAtan2(fxy.y, fxy.x);
			cur_ori = cur_ori > 359 ? 0 : cur_ori;
			histval[cur_ori].ori++;
			histval[cur_ori].point.x += fxy.x;
			histval[cur_ori].point.y += fxy.y;
		}
	}
	int max = 0;
	int ori = 0;
	for (int i = 0; i < 360; i++) {
		if (histval[i].ori > max) {
			max = histval[i].ori;
			ori = i;
		}
	}
	aver.x = histval[ori].point.x / max;
	aver.y = histval[ori].point.y / max;
	oritention = ori;
	if (histval != NULL) {
		free(histval);
		histval = NULL;
	}

}

void showAreaLabel(Mat& image, Mat& labels, Mat& centroids,int size) {
	vector<Vec3b> colors(size);
	int width = labels.cols;
	int height = labels.rows;
	colors[0] = Vec3b(0, 0, 0);
	for (int i = 1; i < size; i++) {
		colors[i] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
	}

	for (int r = 0; r < height; r++) {
		for (int c = 0; c < width; c++) {
			int label = labels.at<int>(r, c);
			image.at<Vec3b>(r, c) = colors[label];
		}
	}

	/*for (int i = 1; i < size; i++) {
		Point2i center(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
		putText(image, to_string(i), center,FONT_HERSHEY_SIMPLEX,1.0f,Scalar::all(255),1,4,false);
	}*/
}
static void pointSobel(Mat& image_gray, Point& center_point, Point2f& dst_gradient) {
	int point_x_left = center_point.x - 1 < 0 ? 0 : center_point.x - 1;
	int point_x_right = center_point.x + 1 > image_gray.cols - 1 ? image_gray.cols - 1 : center_point.x + 1;
	int point_y_top = center_point.y - 1 < 0 ? 0 : center_point.y - 1;
	int point_y_bottom = center_point.y + 1 > image_gray.rows - 1 ? image_gray.rows - 1 : center_point.y + 1;
	dst_gradient.x =3 *image_gray.at<uchar>(Point(point_x_right, point_y_top)) + \
		10 * image_gray.at<uchar>(Point(point_x_right, center_point.y)) + \
		3*image_gray.at<uchar>(Point(point_x_right, point_y_bottom)) - \
		3*image_gray.at<uchar>(Point(point_x_left, point_y_top)) - \
		10 * image_gray.at<uchar>(Point(point_x_left, center_point.y)) - \
		3*image_gray.at<uchar>(Point(point_x_left, point_y_bottom));

	dst_gradient.y = 3*image_gray.at<uchar>(Point(point_x_left, point_y_bottom)) + \
		10 * image_gray.at<uchar>(Point(center_point.x, point_y_bottom)) + \
		3*image_gray.at<uchar>(Point(point_x_right, point_y_bottom)) - \
		3*image_gray.at<uchar>(Point(point_x_left, point_y_top)) - \
		10 * image_gray.at<uchar>(Point(center_point.x, point_y_top)) - \
		3*image_gray.at<uchar>(Point(point_x_right, point_y_top));
	dst_gradient.x /= 16;
	dst_gradient.y /= 16;
}

void contourSobel(Mat& image_gray, const vector<Vec4i>& hierarchy, vector<float>& probs,vector<vector<Point>>& contour_points) {
	int idx = 0;
	Mat grad_show(image_gray.size(), CV_8UC1,Scalar(0));
	
	for (; idx >= 0; idx = hierarchy[idx][0]) {
		int contour_points_of_each = contour_points[idx].size();
		vector<Point2f> gradient_list;

		vector<float> angles;
		//vector<int> scores;
		int max = 0, min = 255;
		/*if (contourArea(contour_points[idx]) < 4) {
			continue;
		}*/
		int bins[12] = {0};
		for (int i = 0; i < contour_points_of_each; i++) {
			
			Point2f dst_gradient(0, 0);
			pointSobel(image_gray, contour_points[idx][i], dst_gradient);
			gradient_list.push_back(dst_gradient);
			angles.push_back(fastAtan2(dst_gradient.y, dst_gradient.x));
			//cout << "idx "<<idx<<" angle: " << angles[i] << endl;
			int index = angles[i] / 30 == 12 ? 0 : angles[i] / 30;
			//cout << "index " << index << endl;
			bins[index]++;
			/*int grad = 3*abs(dst_gradient.x) +3*abs(dst_gradient.y);
			
			if (grad > max) max = grad;
			if (grad < min) min = grad;
			grad = grad > 255 ? 255 : grad;
			grad_show.at<uchar>(contour_points[idx][i]) = grad;*/
		}
		/*int* gradient_table = (int *)malloc(sizeof(int)*(max - min + 1));
		for (int k = 0; k < max - min + 1; k++) {
			gradient_table[k] = 0;
		}*/
		int sub_score = 0;
		for (int bins_index = 0; bins_index < 6; bins_index++) {
			if (bins[bins_index] > 0) sub_score++;
		}
		for (int bins_index = 6; bins_index < 12; bins_index++) {
			if (bins[bins_index] > 0) sub_score++;
		}
		//scores.push_back(sub_score);
		probs.push_back((float)sub_score / 12.);
		/*if (sub_score>4) {
			drawContours(image_gray, contour_points, idx, Scalar(0, 0, 255), 1, 8);
		}*/
	/*	for (int i = 0; i < contour_points_of_each; i++) {
			int grad = abs(gradient_list[i].x) + abs(gradient_list[i].y);
			gradient_table[grad - min]++;
		}*/
		/*for (int j = 0; j < max - min + 1; j++) {
			printf("%d: %d\n", j, gradient_table[j]);
		}*/
	}
	//imshow("grad", grad_show);
}


void FrameRelativeDiff(vector<Mat>& image_list_gray, vector<Mat>& diff) {
	int size = image_list_gray.size();
	int mid = size / 2;
	vector<Mat> forward_diff;
	vector<Mat> backward_diff;
	for (int i = 0; i < mid;i++) {
		forward_diff.push_back(image_list_gray[i + 1] - image_list_gray[i]);
	}

	for (int i = mid ; i < size - 1; i++) {
		backward_diff.push_back(image_list_gray[i] - image_list_gray[i + 1]);
	}

	diff.insert(diff.end(), forward_diff.begin(), forward_diff.end());
	diff.insert(diff.end(), backward_diff.begin(), backward_diff.end());
}

void FrameRelativeDiffBaseCameraMotion(vector<Mat>& image_list_gray, vector<Mat>& diff,vector<Mat>& camera_motion) 
{
	int size = image_list_gray.size();
	int mid = size / 2;
	int width = image_list_gray[0].cols;
	int height = image_list_gray[0].rows;

	Mat image_pre_compensation(height, width, CV_8UC1, Scalar(0));
	Mat image_next_compensation(height, width, CV_8UC1, Scalar(0));
	
	//Mat image_current;
	//image_list_gray[1].copyTo(image_current);
	//image_current.convertTo(image_current, CV_32F);
	Mat p0;
	Mat p1;
	
	warpPerspective(image_list_gray[0], image_pre_compensation, camera_motion[0], Size(width, height),1,BORDER_REPLICATE);
	warpPerspective(image_list_gray[2], image_next_compensation,  camera_motion[1], Size(width, height),1,BORDER_REPLICATE);

	diff.push_back(image_list_gray[1]-image_pre_compensation);
	diff.push_back(image_list_gray[1]-image_next_compensation);
	
		
}

void diffByThreshold(vector<Mat>& diff, vector<Mat>& diff_wb, int threshold_wb) {
	int size = diff.size();
	
	for (int i = 0; i < size; i++) {
		Mat tmp_wb;
		threshold(diff[i], tmp_wb, threshold_wb, 255, CV_THRESH_BINARY);
		diff_wb.push_back(tmp_wb);
	}
}

void diffByPreNext(vector<Mat>& diff_wb, Mat& diff_output) {
	int size = diff_wb.size();
	int mid = size / 2;
	int width = diff_wb[0].cols;
	int height = diff_wb[0].rows;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (diff_wb[mid - 1].at<uchar>(i, j) == 255 && diff_wb[mid].at<uchar>(i, j) == 255) {
				diff_output.at<uchar>(i, j) = 255;
			}

		}
	}
}


void diffFiveFrames(vector<Mat>& image_list_gray, vector<Mat>& diff) {
	Mat diff_1_0 = image_list_gray[1] - image_list_gray[0];
	diff.push_back(diff_1_0);
	Mat diff_2_1 = image_list_gray[2] - image_list_gray[1];
	diff.push_back(diff_2_1);
	Mat diff_2_3 = image_list_gray[2] - image_list_gray[3];
	diff.push_back(diff_2_3);
	Mat diff_3_4 = image_list_gray[3] - image_list_gray[4];
	diff.push_back(diff_3_4);
}

void diffWB(vector<Mat>& diff, vector<Mat>& diff_wb, int threshold_val) {
	for (int i = 0; i < diff.size(); i++) {
		threshold(diff[i], diff_wb[i], threshold_val, 255, CV_THRESH_BINARY);
		//string file_name = "..//..//..//result//"+to_string(i) + ".jpg";
		//imwrite(file_name, diff_wb[i]);
	}
}

void snowMaskbyDiffWB(vector<Mat>& diff_wb, Mat& mask) {
	int height = diff_wb[0].rows;
	int width = diff_wb[0].cols;

	for (int r = 0; r < height; r++) {
		for (int c = 0; c < width; c++) {
			int val[4] = { 0 };
			for (int i = 0; i < diff_wb.size(); i++) {
				val[i] = diff_wb[i].at<uchar>(r, c);
			}
			if (val[1] == 255 & val[2] == 255) {
				if (val[0] == 255) {
					mask.at<uchar>(r, c) = 0;
				}
				else {
					mask.at<uchar>(r, c) = 255;
				}
				
			}
		}
	}
	
}
void modelError(vector<Mat>& diff_wb, vector<Mat>& diff, Mat& sigma) {
	int height = diff_wb[0].rows;
	int width = diff_wb[0].cols;

	for (int r = 0; r < height; r++) {
		for (int c = 0; c < width; c++) {
			int val[4] = { 0 };
			double signma = 0;
			for (int i = 0; i < diff_wb.size(); i++) {
				val[i] = diff_wb[i].at<uchar>(r, c);
			}
			if (val[0] == 0 & val[1] == 0& val[2]==0&val[3]==0) {
				signma += (double)diff[0].at<uchar>(r, c)/4;
				signma += (double)diff[1].at<uchar>(r, c)/4;
				signma += (double)diff[2].at<uchar>(r, c)/4;
				signma += (double)diff[3].at<uchar>(r, c)/4;
				
			}
			sigma.at<double>(r, c) = signma;
		}
	}
}
static int spatialDistance(vector<Mat>& image_list_gray, Point current_point,int radius) {
	int dis = 0;
	for (int i = -radius; i <=radius; i++) {
		for (int j = -radius; j <= radius; j++) {
			dis += abs(image_list_gray[0].at<uchar>(current_point + Point(j, i))- image_list_gray[1].at<uchar>(current_point));
			dis += abs(image_list_gray[2].at<uchar>(current_point + Point(j, i))- image_list_gray[1].at<uchar>(current_point));
		}
		
	}
	return dis;
}
void LikelihoodTS(vector<Mat>& diff, Mat& diff_out, vector<Mat>& image_list_gray,Mat& p_likehood) {
	int width = diff[0].cols;
	int height = diff[0].rows;
	int size = diff.size();
	double alpha1 = 0.5;
	double alpha2 = 0.5;
	
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (diff_out.at<uchar>(i, j) == 255) {
				p_likehood.at<float>(i, j) = exp(-alpha1*spatialDistance(image_list_gray, Point(j, i), 1));
			}
			else {
				p_likehood.at<float>(i, j) = exp(-alpha2*(abs(image_list_gray[0].at<uchar>(i, j) -image_list_gray[1].at<uchar>(i, j)) + \
					abs(image_list_gray[1].at<uchar>(i, j)-image_list_gray[2].at<uchar>(i, j))));
			}
		}
	}
}


static void nearBlockMatching(Mat& image1, Mat& image2,Rect& rect, int step,int& sad) {
	int top_left_x = rect.x - 2 * rect.width <0 ? 0 : rect.x - 2 * rect.width;
	int top_left_y = rect.y - 2 * rect.height < 0 ? 0 : rect.y - 2 * rect.height;
	int right_bottom_x = top_left_x + 5 * rect.width > image2.cols - 1 ? image2.cols - 1 : top_left_x + 5 * rect.width;
	int right_bottom_y = top_left_y + 5 * rect.height > image2.rows - 1 ? image2.rows - 1 : top_left_y + 5 * rect.height;
	Rect search_area(top_left_x, top_left_y, right_bottom_x - top_left_x, right_bottom_y - top_left_y);
	vector<int> sad_gray_list;
	for (int i = top_left_y; i < right_bottom_y-rect.height; i++) {
		for (int j = top_left_x; j < right_bottom_x-rect.width; j++) {
			 int r_base = rect.y,sad_gray=0;
			for (int r = i; r <i+ rect.height; r++) {
				int c_base = rect.x;
				for (int c = j; c < j+rect.width; c++) {
					sad_gray += abs(image1.at<uchar>(r_base, c_base) - image2.at<uchar>(r, c));
					c_base++;
				}
				r_base++;
				//cout << "c_base " << c_base << "r_base " << r_base << endl;
			}
			sad_gray_list.push_back(sad_gray);
		}
	}
	int min = 1e6;
	for (int m = 0; m < sad_gray_list.size(); m++) {
		if (sad_gray_list[m] < min) {
			min = sad_gray_list[m];
		}
	}
	sad = min;
}

void neighbourBlockMatching(Mat& labels, Mat& stats, Mat& centroids, vector<Mat>& image_list_gray,vector<int>& valid_labels) {
	int width = labels.cols;
	int height = labels.rows;
	int mid = image_list_gray.size() / 2;


	for (int k = 1; k < stats.rows; k++) {
		if (stats.at<int>(k, 4) > 4) {
			int sad = 0;
			Rect rect(stats.at<int>(k, 0), stats.at<int>(k, 1), stats.at<int>(k, 2), stats.at<int>(k, 3));
			nearBlockMatching(image_list_gray[mid], image_list_gray[mid - 1], rect, 0, sad);
			int num = rect.width*rect.height;
			if ((float)sad / num > 3) {
				valid_labels.push_back(k);
			}
			//cout << "SAD "<<k<<":" << (float)sad / num << endl;

		}

	}
}

void spatialFilter(Mat& labels, Mat& diff_wb, vector<int>& valid_label) {
	int width = diff_wb.cols;
	int height = diff_wb.rows;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int label_val = labels.at<int>(i, j);
			if (label_val != 0) {
				vector<int>::iterator iter = find(valid_label.begin(), valid_label.end(), label_val);
				if (iter == valid_label.end()) {
					//cout << i << " " << j << endl;
					diff_wb.at<uchar>(i, j) = 0;
				}

			}
		}
	}
}

double modelError(vector<Mat>& diff, Mat& diff_out) {
	int count = 0;
	double error = 0.;
	int width = diff_out.cols;
	int height = diff_out.rows;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (diff_out.at<uchar>(i, j) == 0) {
				count++;
				error += diff[0].at<uchar>(i, j)*diff[0].at<uchar>(i, j);
				error += diff[1].at<uchar>(i, j)*diff[1].at<uchar>(i, j);
					
					
			}
		}
	}
	return error / count;
}

static double neighbourDiff(Mat& labels, Rect rect, Mat& image) {
	int top_left_x = rect.x - rect.width <0 ? 0 : rect.x -  rect.width;
	int top_left_y = rect.y - rect.height < 0 ? 0 : rect.y - rect.height;
	int right_bottom_x = top_left_x + 3 * rect.width > image.cols - 1 ? image.cols - 1 : top_left_x + 3* rect.width;
	int right_bottom_y = top_left_y + 3 * rect.height > image.rows - 1 ? image.rows - 1 : top_left_y + 3 * rect.height;
	Rect search_area(top_left_x, top_left_y, right_bottom_x - top_left_x, right_bottom_y - top_left_y);
	vector<uchar> neighbourList;
	for (int i = top_left_y; i < right_bottom_y - rect.height; i++) {
		for (int j = top_left_x; j < right_bottom_x - rect.width; j++) {
			if (labels.at<int>(i, j) == 0) {
				neighbourList.push_back(image.at<uchar>(i,j));
			}
		}
	}
	double sum = 0, sum2 = 0;
	for (int i = 0; i < neighbourList.size(); i++) {

		
		sum += neighbourList[i];
		sum2 += neighbourList[i] * neighbourList[i];
	}
		
		double variance = sum2 / neighbourList.size() - (sum / neighbourList.size())*(sum / neighbourList.size());
		
		return variance;
	
}

void neighbourBlockDiff(Mat& labels, Mat& stats, Mat& centroids, vector<Mat>& image_list_gray, vector<int>& valid_label, double model_error) {
	int width = labels.cols;
	int height = labels.rows;
	int mid = image_list_gray.size() / 2;


	for (int k = 1; k < stats.rows; k++) {
		if (stats.at<int>(k, 4) > 4) {
			int sad = 0;
			Rect rect(stats.at<int>(k, 0), stats.at<int>(k, 1), stats.at<int>(k, 2), stats.at<int>(k, 3));
			double var = neighbourDiff(labels, rect, image_list_gray[1]);
			if (var < model_error) {
				valid_label.push_back(k);
			}

//cout << "VAR "<<k<<":" <<var<< endl;

		}

	}
}

void showLabelImg(Mat& label) {
	int width = label.cols;
	int height = label.rows;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			label.at<uchar>(i, j) = label.at<uchar>(i, j) > 0 ? 255 : 0;
		}
	}
}

void showMaskImg(Mat& label, Mat& show_img) {
	
	int width = label.cols;
	int height = label.rows;
	show_img.create(height, width, CV_8UC1);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			show_img.at<uchar>(i, j) = label.at<int>(i, j) > 0 ? 255 : 0;
		}
	}
}

void getMaskFromValidLabels(Mat& mask, vector<int>& valid_labels) {
	int width = mask.cols;
	int height = mask.rows;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int cur = mask.at<int>(i, j);
			vector<int>::iterator iter = find(valid_labels.begin(), valid_labels.end(), cur);
			if (iter == valid_labels.end()) {
				mask.at<int>(i, j) = 0;
			}
		}
	}
}

void getMaskFromProbs(Mat& mask, vector<float>& prob1, vector<float>& prob2) {
	int width = mask.cols;
	int height = mask.rows;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {

			int cur = mask.at<int>(i, j);
			if (cur == 0) continue;
			if (prob1[cur - 1] < 0.1) {
				mask.at<int>(i, j) = 0;
			}
		}
	}
}
static void getPatchCenter(int img_width, int img_height, Size winSize, Point center, Point& patch_center) {

}
void nearNeighourSimilarity(Mat& image, Mat& stats, vector<float>& probs_similar) {

	vector<double> similar;
	for (int i = 1; i < stats.rows; i++) {
		vector<double> sads;
		Point center_point(stats.at<int>(i, 0) + stats.at<int>(i, 2) / 2, stats.at<int>(i, 1) + stats.at<int>(i, 3) / 2);
		int patch_width = stats.at<int>(i, 2);
		int patch_height = stats.at<int>(i, 3);
		Mat center_patch = getPatch(image, Size(patch_width, patch_height), center_point);
		int area = patch_width*patch_height;
		double s0 = cv::sum(center_patch)(0) / area;

		for (int m = -1; m <= 1; m++) {
			for (int n = -1; n <= 1; n++) {
				if (m == 0 && n == 0) continue;
				Point patch_center(center_point.x - n*patch_width, center_point.y - m*patch_height);
				patch_center.x = patch_center.x < 0 ? 0 : patch_center.x;
				patch_center.y = patch_center.y < 0 ? 0 : patch_center.y;
				patch_center.x = patch_center.x > image.cols - 1 ? image.cols - 1 : patch_center.x;
				patch_center.y = patch_center.y > image.rows - 1 ? image.rows - 1 : patch_center.y;
				Mat neig = getPatch(image, Size(patch_width, patch_height), patch_center);
				double s1 = cv::sum(neig)(0) / area;
				sads.push_back(abs(s1 - s0));
			}
		}
		double min = sads[0];
		for (int k = 1; k < sads.size(); k++) {
			if (sads[k] < min) min = sads[k];
		}
		similar.push_back(exp(-min));
		//probs_similar.push_back(min);
		
	}
	double sum = 0.;
	for (int r = 0; r < similar.size(); r++) {
		sum += similar[r];
	}

	for (int c = 0; c < similar.size(); c++) {
		probs_similar.push_back(similar[c] / sum);
		cout << "similar" << similar[c] / sum << endl;
	}

}

void rgbStdDev(Mat& image, Mat& labels,Mat& stats, Mat& normalize_std, int size) {
	int width = image.cols;
	int height = image.rows;
	Mat normalize_std_tmp(size, 2, CV_32FC1,Scalar(0));
	Mat rgb_std(size, 3, CV_32FC1,Scalar(0));
	Mat rgb_sum(size, 3, CV_32FC1,Scalar(0));
	Mat rgb_sum2(size, 3, CV_32FC1,Scalar(0));
	
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (labels.at<int>(i, j) == 0) continue;
			for (int k = 0; k < 3; k++) {
				rgb_sum.at<float>(labels.at<int>(i, j) - 1, k) += (float)image.at<Vec3b>(i, j)[k];
				rgb_sum2.at<float>(labels.at<int>(i,j) - 1, k) += (float)image.at<Vec3b>(i, j)[k] * (float)image.at<Vec3b>(i, j)[k];
			}
		}
	}
	for (int m = 0; m < size; m++) {
		for (int k = 0; k < 3; k++) {
			rgb_std.at<float>(m, k) = rgb_sum2.at<float>(m, k) / stats.at<int>(m+1, 4) - (rgb_sum.at<float>(m, k) / stats.at<int>(m+1, 4))*(rgb_sum.at<float>(m, k) / stats.at<int>(m+1, 4));
		}
		float sum_bgr = rgb_std.at<float>(m, 0) + rgb_std.at<float>(m, 1) + rgb_std.at<float>(m, 2);
		
		if (stats.at<int>(m+1,4) < 3||sum_bgr<0.001) {
			normalize_std_tmp.at<float>(m, 0) = 20;

		}
		else {
			float b_stddev = sqrt(rgb_std.at<float>(m, 0) / 1);
			float g_stddev = sqrt(rgb_std.at<float>(m, 1) / 1);
			float r_stddev = sqrt(rgb_std.at<float>(m, 2) / 1);


			float average_std = (b_stddev + g_stddev + r_stddev) / 3;
			float n_stddev = ((r_stddev - average_std)*(r_stddev - average_std) + \
				(g_stddev - average_std)*(g_stddev - average_std) + \
				(b_stddev - average_std)*(b_stddev - average_std)) / 3;
			/*float abs_bg = abs(sqrt(rgb_std.at<float>(m, 0) / sum_bgr) - sqrt(rgb_std.at<float>(m, 1) / sum_bgr));
			float abs_gr = abs(sqrt(rgb_std.at<float>(m, 1) / sum_bgr) - sqrt(rgb_std.at<float>(m, 2) / sum_bgr));
			normalize_std_tmp.at<float>(m, 0) = cv::abs(abs_bg - abs_gr);*/
			normalize_std_tmp.at<float>(m, 0) = sqrt(n_stddev);


			
		}
		//cout << "t_std " << normalize_std_tmp.at<float>(m, 0) << endl;
	}

	normalize_std = normalize_std_tmp;
}
	
void getMaskFromStd(Mat& mask, Mat& normalize_std) {
	int width = mask.cols;
	int height = mask.rows;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {

			int cur = mask.at<int>(i, j);
			if (cur == 0) continue;
			if (normalize_std.at<float>(cur-1,0) > 5) {
				mask.at<int>(i, j) = 0;
			}
		}
	}
}

void imageListCompensation(vector<Mat>& image_list, vector<Mat>& image_list_compensation, vector<Mat>& camera_motion) {

	Mat image_pre_compensation;
	Mat image_next_compensation;

	warpPerspective(image_list[0], image_pre_compensation, camera_motion[0], image_list[0].size(), 1, BORDER_REPLICATE);
	warpPerspective(image_list[2], image_next_compensation, camera_motion[1], image_list[1].size(), 1, BORDER_REPLICATE);

	image_list_compensation.push_back(image_pre_compensation);
	image_list_compensation.push_back(image_list[1]);
	image_list_compensation.push_back(image_next_compensation);

}
void imageListGrayCompensation(vector<Mat>& image_list_gray, vector<Mat>& image_list_gray_compensation, vector<Mat>& camera_motion) 
{
	Mat image_pre_compensation;
	Mat image_next_compensation;

	warpPerspective(image_list_gray[0], image_pre_compensation, camera_motion[0], image_list_gray[0].size(), 1, BORDER_REPLICATE);
	warpPerspective(image_list_gray[2], image_next_compensation, camera_motion[1], image_list_gray[1].size(), 1, BORDER_REPLICATE);

	image_list_gray_compensation.push_back(image_pre_compensation);
	image_list_gray_compensation.push_back(image_list_gray[1]);
	image_list_gray_compensation.push_back(image_next_compensation);
}

void imageClosing(Mat& src, Mat& output, int kenel_size) {
	Mat element = getStructuringElement(MORPH_RECT, Size(kenel_size, kenel_size));
	morphologyEx(src, output, MORPH_CLOSE, element);
}

void calcDepth(Mat& camera_motion, Mat& dst, int width, int height) {
	Mat_<double> H = camera_motion;
	dst.create(height, width, CV_32FC1);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			float x1 = (H(0, 0)*j + H(0, 1)*i + H(0, 2)) / (H(2, 0)*j + H(2, 1)*i + H(2, 2));
			float y1= (H(1, 0)*j + H(1, 1)*i + H(1, 2)) / (H(2, 0)*j + H(2, 1)*i + H(2, 2));
			dst.at<float>(i, j) = abs(x1 - j) + abs(y1 - i);
		}
	}
	normalize(dst, dst, 0, 1, NORM_MINMAX);
}

bool FindCameraMatrices(const Mat& K,
	const Mat& Kinv,
	const Mat& F,
	Matx34d& P,
	Matx34d& P1,
	const Mat& discoeff,
	const vector<KeyPoint>& imgpts1,
	const vector<KeyPoint>& imgpts2,
	vector<KeyPoint>& imgpts1_good,
	vector<KeyPoint>& imgpts2_good,
	vector<DMatch>& matches,
	vector<CloudPoint>& outCloud) {
	Mat_<double> E = K.t() * F * K; // Essential Matrix
	if (fabsf(determinant(E)) > 1e-05) {
		cout << "det(E) != 0 : " << determinant(E) << "\n";
		return false;
	}
	Mat_<double> R1(3, 3);
	Mat_<double> R2(3, 3);
	Mat_<double> t1(1, 3);
	Mat_<double> t2(1, 3);
	SVD svd(E);
	Matx33d W(0, -1, 0,
		1, 0, 0,
		0, 0, 1);
	Matx33d Wt(0, 1, 0,
		-1, 0, 0,
		0, 0, 1);
	R1 = svd.u * Mat(W) * svd.vt; //Rotation solution 1
	R2 = svd.u * Mat(Wt) * svd.vt; //Rotation solution 2
	t1 = svd.u.col(2); //Translatiion solution 1
	t2 = -svd.u.col(2); //Translation solution 2

	P1 = Matx34d(R1(0, 0), R1(0, 1), R1(0, 2), t1(0),
		R1(1, 0), R1(1, 1), R1(1, 2), t1(1),
		R1(2, 0), R1(2, 1), R1(2, 2), t1(2)); // Camara Matrix
	P = Matx34d(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);

	cout << "Testing P1 " << endl << Mat(P1) << endl;

	vector<CloudPoint> pcloud, pcloud1;
	vector<KeyPoint> corresp;

	double reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, P, P1, pcloud, corresp, discoeff);
	double reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, P1, P, pcloud1, corresp, discoeff);
	vector<uchar> tmp_status;
	if (!TestTriangulation(pcloud, P1, tmp_status) || !TestTriangulation(pcloud1, P, tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0)
	{
		P1 = Matx34d(R1(0, 0), R1(0, 1), R1(0, 2), t2(0),
			R1(1, 0), R1(1, 1), R1(1, 2), t2(1),
			R1(2, 0), R1(2, 1), R1(2, 2), t2(2));
		cout << "Testing P1 " << endl << Mat(P1) << endl;

		pcloud.clear(); pcloud1.clear(); corresp.clear();
		reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, P, P1, pcloud, corresp, discoeff);
		reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, P1, P, pcloud1, corresp, discoeff);

		if (!TestTriangulation(pcloud, P1, tmp_status) || !TestTriangulation(pcloud1, P, tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {


			P1 = Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), t1(0),
				R2(1, 0), R2(1, 1), R2(1, 2), t1(1),
				R2(2, 0), R2(2, 1), R2(2, 2), t1(2));
			//cout << "Testing P1 "<< endl << Mat(P1) << endl;

			pcloud.clear(); pcloud1.clear(); corresp.clear();
			reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, P, P1, pcloud, corresp, discoeff);
			reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, P1, P, pcloud1, corresp, discoeff);

			if (!TestTriangulation(pcloud, P1, tmp_status) || !TestTriangulation(pcloud1, P, tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
				P1 = Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), t2(0),
					R2(1, 0), R2(1, 1), R2(1, 2), t2(1),
					R2(2, 0), R2(2, 1), R2(2, 2), t2(2));
				cout << "Testing P1 " << endl << Mat(P1) << endl;

				pcloud.clear(); pcloud1.clear(); corresp.clear();
				reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, P, P1, pcloud, corresp, discoeff);
				reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, P1, P, pcloud1, corresp, discoeff);

				if (!TestTriangulation(pcloud, P1, tmp_status) || !TestTriangulation(pcloud1, P, tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
					cout << "Err is too big." << endl;
					return false;
				}
			}
		}
	}
	for (unsigned int i = 0; i<pcloud.size(); i++) {
		outCloud.push_back(pcloud[i]);
	}

}

Mat_<double> LinearLSTriangulation(Point3d u,		//homogenous image point (u,v,1)
	Matx34d P,		//camera 1 matrix
	Point3d u1,		//homogenous image point in 2nd camera
	Matx34d P1		//camera 2 matrix
)
{

	//build matrix A for homogenous equation system Ax = 0
	//assume X = (x,y,z,1), for Linear-LS method
	//which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
	//	cout << "u " << u <<", u1 " << u1 << endl;
	//	Matx<double,6,4> A; //this is for the AX=0 case, and with linear dependence..
	//	A(0) = u.x*P(2)-P(0);
	//	A(1) = u.y*P(2)-P(1);
	//	A(2) = u.x*P(1)-u.y*P(0);
	//	A(3) = u1.x*P1(2)-P1(0);
	//	A(4) = u1.y*P1(2)-P1(1);
	//	A(5) = u1.x*P(1)-u1.y*P1(0);
	//	Matx43d A; //not working for some reason...
	//	A(0) = u.x*P(2)-P(0);
	//	A(1) = u.y*P(2)-P(1);
	//	A(2) = u1.x*P1(2)-P1(0);
	//	A(3) = u1.y*P1(2)-P1(1);
	Matx43d A(u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
		u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
		u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
		u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
	);
	Matx41d B(-(u.x*P(2, 3) - P(0, 3)),
		-(u.y*P(2, 3) - P(1, 3)),
		-(u1.x*P1(2, 3) - P1(0, 3)),
		-(u1.y*P1(2, 3) - P1(1, 3)));

	Mat_<double> X;
	solve(A, B, X, DECOMP_SVD);

	return X;
}




Mat_<double> IterativeLinearLSTriangulation(Point3d u,	//homogenous image point (u,v,1)
	Matx34d P,			//camera 1 matrix
	Point3d u1,			//homogenous image point in 2nd camera
	Matx34d P1			//camera 2 matrix
)
{
	double wi = 1, wi1 = 1;
	Mat_<double> X(4, 1);
	for (int i = 0; i<10; i++)
	{
		Mat_<double> X_ = LinearLSTriangulation(u, P, u1, P1);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;

		//recalculate weights
		double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
		double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);

		//breaking point
		if (fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

		wi = p2x;
		wi1 = p2x1;

		//reweight equations and solve
		Matx43d A((u.x*P(2, 0) - P(0, 0)) / wi, (u.x*P(2, 1) - P(0, 1)) / wi, (u.x*P(2, 2) - P(0, 2)) / wi,
			(u.y*P(2, 0) - P(1, 0)) / wi, (u.y*P(2, 1) - P(1, 1)) / wi, (u.y*P(2, 2) - P(1, 2)) / wi,
			(u1.x*P1(2, 0) - P1(0, 0)) / wi1, (u1.x*P1(2, 1) - P1(0, 1)) / wi1, (u1.x*P1(2, 2) - P1(0, 2)) / wi1,
			(u1.y*P1(2, 0) - P1(1, 0)) / wi1, (u1.y*P1(2, 1) - P1(1, 1)) / wi1, (u1.y*P1(2, 2) - P1(1, 2)) / wi1
		);
		Mat_<double> B = (Mat_<double>(4, 1) << -(u.x*P(2, 3) - P(0, 3)) / wi,
			-(u.y*P(2, 3) - P(1, 3)) / wi,
			-(u1.x*P1(2, 3) - P1(0, 3)) / wi1,
			-(u1.y*P1(2, 3) - P1(1, 3)) / wi1
			);

		solve(A, B, X_, DECOMP_SVD);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
	}
	return X;
}

//Triagulate points
double TriangulatePoints(const vector<KeyPoint>& pt_set1,
	const vector<KeyPoint>& pt_set2,
	const Mat& K,
	const Mat& Kinv,
	const Matx34d& P,
	const Matx34d& P1,
	vector<CloudPoint>& pointcloud,
	vector<KeyPoint>& correspImg1Pt,
	const Mat& distcoeff)
{

	vector<double> depths;


	//	pointcloud.clear();
	correspImg1Pt.clear();

	Matx44d P1_(P1(0, 0), P1(0, 1), P1(0, 2), P1(0, 3),
		P1(1, 0), P1(1, 1), P1(1, 2), P1(1, 3),
		P1(2, 0), P1(2, 1), P1(2, 2), P1(2, 3),
		0, 0, 0, 1);
	Matx44d P1inv(P1_.inv());

	cout << "Triangulating Now . . .";
	double t = getTickCount();
	vector<double> reproj_error;
	unsigned int pts_size = pt_set1.size();

#if 0
	//Using OpenCV's triangulation
	//convert to Point2f
	vector<Point2f> _pt_set1_pt, _pt_set2_pt;
	KeyPointsToPoints(pt_set1, _pt_set1_pt);
	KeyPointsToPoints(pt_set2, _pt_set2_pt);

	//undistort
	Mat pt_set1_pt, pt_set2_pt;
	undistortPoints(_pt_set1_pt, pt_set1_pt, K, distcoeff);
	undistortPoints(_pt_set2_pt, pt_set2_pt, K, distcoeff);

	//triangulate
	Mat pt_set1_pt_2r = pt_set1_pt.reshape(1, 2);
	Mat pt_set2_pt_2r = pt_set2_pt.reshape(1, 2);
	Mat pt_3d_h(1, pts_size, CV_32FC4);
	cv::triangulatePoints(P, P1, pt_set1_pt_2r, pt_set2_pt_2r, pt_3d_h);
	//calculate reprojection
	vector<Point3f> pt_3d;
	convertPointsHomogeneous(pt_3d_h.reshape(4, 1), pt_3d);
	cv::Mat_<double> R = (cv::Mat_<double>(3, 3) << P(0, 0), P(0, 1), P(0, 2), P(1, 0), P(1, 1), P(1, 2), P(2, 0), P(2, 1), P(2, 2));
	Vec3d rvec; Rodrigues(R, rvec);
	Vec3d tvec(P(0, 3), P(1, 3), P(2, 3));
	vector<Point2f> reprojected_pt_set1;
	projectPoints(pt_3d, rvec, tvec, K, distcoeff, reprojected_pt_set1);
	for (unsigned int i = 0; i<pts_size; i++) {
		CloudPoint cp;
		cp.pt = pt_3d[i];
		pointcloud.push_back(cp);
		reproj_error.push_back(norm(_pt_set1_pt[i] - reprojected_pt_set1[i]));
	}
#else
	Mat_<double> KP1 = K * Mat(P1);
#pragma omp parallel for num_threads(1)
	for (int i = 0; i<pts_size; i++)
	{
		Point2f kp = pt_set1[i].pt;
		Point3d u(kp.x, kp.y, 1.0);
		Mat_<double> um = Kinv * Mat_<double>(u);
		u.x = um(0); u.y = um(1); u.z = um(2);

		Point2f kp1 = pt_set2[i].pt;
		Point3d u1(kp1.x, kp1.y, 1.0);
		Mat_<double> um1 = Kinv * Mat_<double>(u1);
		u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

		Mat_<double> X = IterativeLinearLSTriangulation(u, P, u1, P1);

		//		cout << "3D Point: " << X << endl;
		//		Mat_<double> x = Mat(P1) * X;
		//		cout <<	"P1 * Point: " << x << endl;
		//		Mat_<double> xPt = (Mat_<double>(3,1) << x(0),x(1),x(2));
		//		cout <<	"Point: " << xPt << endl;
		Mat_<double> xPt_img = KP1 * X;				//reproject
													//		cout <<	"Point * K: " << xPt_img << endl;
		Point2f xPt_img_(xPt_img(0) / xPt_img(2), xPt_img(1) / xPt_img(2));

#pragma omp critical
		{
			double reprj_err = norm(xPt_img_ - kp1);
			reproj_error.push_back(reprj_err);

			CloudPoint cp;
			cp.pt = Point3d(X(0), X(1), X(2));
			cp.reprojection_error = reprj_err;

			pointcloud.push_back(cp);
			correspImg1Pt.push_back(pt_set1[i]);

			depths.push_back(X(2));

		}
	}
#endif

	Scalar mse = mean(reproj_error);
	t = ((double)getTickCount() - t) / getTickFrequency();
	cout << "Done. \n\r" << pointcloud.size() << "points, " << "mean square reprojetion err = " << mse[0] << endl;

	//show "range image"

	{
		double minVal, maxVal;
		minMaxLoc(depths, &minVal, &maxVal);
		Mat tmp(1224, 1632, CV_8UC3, Scalar(0, 0, 0)); //cvtColor(img_1_orig, tmp, CV_BGR2HSV);
		for (unsigned int i = 0; i<pointcloud.size(); i++) {
			double _d = MAX(MIN((pointcloud[i].pt.z - minVal) / (maxVal - minVal), 1.0), 0.0);
			circle(tmp, correspImg1Pt[i].pt, 1, Scalar(255 * (1.0 - (_d)), 255, 255), CV_FILLED);
		}
		cvtColor(tmp, tmp, CV_HSV2BGR);
		imshow("Depth Map", tmp);
		waitKey(0);
		destroyWindow("Depth Map");
	}


	return mse[0];
}



bool TestTriangulation(const vector<CloudPoint>& pcloud, const Matx34d& P, vector<uchar>& status) {
	vector<Point3d> pcloud_pt3d = CloudPointsToPoints(pcloud);
	vector<Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());

	Matx44d P4x4 = Matx44d::eye();
	for (int i = 0; i<12; i++) P4x4.val[i] = P.val[i];

	perspectiveTransform(pcloud_pt3d, pcloud_pt3d_projected, P4x4);

	status.resize(pcloud.size(), 0);
	for (int i = 0; i<pcloud.size(); i++) {
		status[i] = (pcloud_pt3d_projected[i].z > 0) ? 1 : 0;
	}
	int count = countNonZero(status);

	double percentage = ((double)count / (double)pcloud.size());
	cout << count << "/" << pcloud.size() << " = " << percentage*100.0 << "% are in front of camera" << endl;
	if (percentage < 0.8)
		return false; //less than 80% of the points are in front of the camera

					  //check for coplanarity of points
	if (false) //not
	{
		cv::Mat_<double> cldm(pcloud.size(), 3);
		for (unsigned int i = 0; i<pcloud.size(); i++) {
			cldm.row(i)(0) = pcloud[i].pt.x;
			cldm.row(i)(1) = pcloud[i].pt.y;
			cldm.row(i)(2) = pcloud[i].pt.z;
		}
		cv::Mat_<double> mean;
		cv::PCA pca(cldm, mean, CV_PCA_DATA_AS_ROW);

		int num_inliers = 0;
		cv::Vec3d nrm = pca.eigenvectors.row(2); nrm = nrm / norm(nrm);
		cv::Vec3d x0 = pca.mean;
		double p_to_plane_thresh = sqrt(pca.eigenvalues.at<double>(2));

		for (int i = 0; i<pcloud.size(); i++) {
			Vec3d w = Vec3d(pcloud[i].pt) - x0;
			double D = fabs(nrm.dot(w));
			if (D < p_to_plane_thresh) num_inliers++;
		}

		cout << num_inliers << "/" << pcloud.size() << " are coplanar" << endl;
		if ((double)num_inliers / (double)(pcloud.size()) > 0.85)
			return false;
	}

	return true;
}

std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts)
{
	std::vector<cv::Point3d> out;
	for (unsigned int i = 0; i<cpts.size(); i++)
	{
		out.push_back(cpts[i].pt);
	}
	return out;
}