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


