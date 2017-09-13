#include "dcp_core.h"

using std::sort;
struct Pixel
{
	uchar value;
	int i, j;
};
void RemovalBaseMask(Mat input, Mat mask,Mat& output,int radius_init) {
	int width = input.cols;
	int height = input.rows;
	Mat input_gray;
	int radius = radius_init;
	output = input;
	cvtColor(input, input_gray, CV_BGR2GRAY);
	if (mask.cols != width || mask.rows != height) {
		printf("Input image size is not equal to mask size");
		exit(-1);
	}

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			uchar mask_value = mask.at<uchar>(i, j);
			if (mask_value != 0) {
				int st_row = i - radius, ed_row = i + radius;
				int st_col = j - radius, ed_col = j + radius;
				int count = 0;
				int count_total = 0;
				st_row = st_row < 0 ? 0 : st_row;
				ed_row = ed_row >= height ? (height - 1) : ed_row;
				st_col = st_col < 0 ? 0 : st_col;
				ed_col = ed_col >= width ? (width - 1) : ed_col;
				struct Pixel *patch_pixels = (struct Pixel*)malloc((ed_row - st_row + 1)*(ed_col - st_col + 1) * sizeof(struct Pixel));
				for (int m = st_row; m <= ed_row; m++) {
					for (int n = st_col; n <= ed_col; n++) {
						
							uchar cur= input_gray.at<uchar>(m, n);
							if (cur == input_gray.at<uchar>(i, j)) {
								continue;
							}
							else {
								struct Pixel p = { cur,m,n };
								patch_pixels[count++] = p;
							}
					}
				}
				sort(patch_pixels, patch_pixels + count, [](struct Pixel &a, struct Pixel &b) { return a.value < b.value; });
				int re_row = patch_pixels[(int)round(count / 2)].i;
				int re_col = patch_pixels[(int)round(count / 2)].j;
				output.at<Vec3b>(i, j) = input.at<Vec3b>(re_row, re_col);
			}
		}
	}
}