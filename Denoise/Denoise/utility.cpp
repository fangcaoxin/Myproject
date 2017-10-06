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

void contourSobel(Mat& image_gray, const vector<Vec4i>& hierarchy, vector<vector<Point>>& contour_points) {
	int idx = 0;
	Mat grad_show(image_gray.size(), CV_8UC1,Scalar(0));
	
	for (; idx >= 0; idx = hierarchy[idx][0]) {
		int contour_points_of_each = contour_points[idx].size();
		vector<Point2f> gradient_list;

		vector<float> angles;

		int max = 0, min = 255;

		for (int i = 0; i < contour_points_of_each; i++) {
			Point2f dst_gradient(0, 0);
			pointSobel(image_gray, contour_points[idx][i], dst_gradient);
			
			gradient_list.push_back(dst_gradient);

			angles.push_back(fastAtan2(dst_gradient.y, dst_gradient.x));
			//cout << "idx "<<idx<<" angle: " << angles[i] << endl;

			int grad = abs(dst_gradient.x) +abs(dst_gradient.y);
			
			if (grad > max) max = grad;
			if (grad < min) min = grad;
			//grad = grad > 255 ? 255 : grad;
			//grad_show.at<uchar>(contour_points[idx][i]) = grad;
		}
		int* gradient_table = (int *)malloc(sizeof(int)*(max - min + 1));
		for (int k = 0; k < max - min + 1; k++) {
			gradient_table[k] = 0;
		}
		for (int i = 0; i < contour_points_of_each; i++) {
			int grad = abs(gradient_list[i].x) + abs(gradient_list[i].y);
			gradient_table[grad - min]++;
		}
		for (int j = 0; j < max - min + 1; j++) {
			printf("%d: %d\n", j, gradient_table[j]);
		}
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

void neighbourBlockDiff(Mat& labels, Mat& stats, Mat& centroids, vector<Mat>& image_list_gray, vector<int>& valid_label,double model_error) {
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
			label.at<uchar>(i, j) = label.at<uchar>(i, j) == 1 ? 255 : 0;
		}
	}
}