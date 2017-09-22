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
		int max = 0, min = 255;
		for (int i = 0; i < contour_points_of_each; i++) {
			Point2f dst_gradient(0, 0);
			pointSobel(image_gray, contour_points[idx][i], dst_gradient);
			
			gradient_list.push_back(dst_gradient);
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

