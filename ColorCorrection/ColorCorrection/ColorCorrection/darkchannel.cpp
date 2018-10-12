#include "core.h"
#include  <opencv2/highgui/highgui.hpp>

void calcDarkChannel(cv::Mat& darkchannel, cv::Mat& brightchannel, cv::Mat& input, int radius) {

	
	int height = input.rows;
	int width = input.cols;
	darkchannel.create(height, width, CV_8UC1);
	brightchannel.create(height, width, CV_8UC1);
	int channels = input.channels();
	int st_row, ed_row;
	int st_col, ed_col;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			st_row = i - radius, ed_row = i + radius;
			st_col = j - radius, ed_col = j + radius;

			st_row = st_row < 0 ? 0 : st_row;
			ed_row = ed_row >= height ? (height - 1) : ed_row;
			st_col = st_col < 0 ? 0 : st_col;
			ed_col = ed_col >= width ? (width - 1) : ed_col;

			
			int min = 300;
			int max = 0;
			for (int m = st_row; m <= ed_row; m++)
			{
				for (int n = st_col; n <= ed_col; n++)
				{
					if (channels == 3) {
						for (int k = 0; k < channels; k++)
						{

							int cur = input.at<cv::Vec3b>(m, n)[k];
							if (cur < min)
								min = cur;

							if (cur > max)
								max = cur;
						}
					}
					else if (channels == 1) {
						int cur = input.at<uchar>(m, n);
						if (cur < min)
							min = cur;

						if (cur > max)
							max = cur;
					
					}
				} 
			}
			darkchannel.at<uchar>(i, j) = min;
			brightchannel.at<uchar>(i, j) = max;
		}
	}
	//imshow("darkchannel", darkchannel);

}

void blockAverage(cv::Mat& darkChannel, cv::Mat& mask, int radius) {
	int width = darkChannel.cols;
	int height = darkChannel.rows;

	int st_row, ed_row;
	int st_col, ed_col;
	for (int i = 0; i < height; i = i + radius / 2)
	{
		for (int j = 0; j < width; j = j + radius / 2)
		{
			st_row = i - radius, ed_row = i + radius;
			st_col = j - radius, ed_col = j + radius;

			st_row = st_row < 0 ? 0 : st_row;
			ed_row = ed_row >= height ? (height - 1) : ed_row;
			st_col = st_col < 0 ? 0 : st_col;
			ed_col = ed_col >= width ? (width - 1) : ed_col;

			int block_pixel = 0;
			int pixel_sum = 0;
			for (int m = st_row; m <= ed_row; m++)
			{
				for (int n = st_col; n <= ed_col; n++)
				{
					block_pixel++;
					pixel_sum += darkChannel.at<uchar>(m, n);
				}
			}
			float block_average = pixel_sum / block_pixel;
			for (int m = st_row; m <= ed_row; m++)
			{
				for (int n = st_col; n <= ed_col; n++)
				{
					float inv = abs(darkChannel.at<uchar>(m, n) - block_average);
				}
			}

		}
	}

}

struct pixel {
	int value;
	std::vector<cv::Point2i> pos;
};
//#define DRAWHIST
void darkChannelMask(cv::Mat& darkchannel, cv::Mat& mask) {
#ifdef DRAWHIST
	Mat hist;
	int histSize = 256;
	float range[] = { 0,256 };
	const float* histRange = { range };
	bool uniform = true; bool accumulate = false;
	calcHist(&darkchannel, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound((double)hist_w / histSize);

	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

	normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	for (int i = 1; i < histSize; i++)
	{
		line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
			Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
			Scalar(255, 0, 0), 2, 8, 0);
	}
	imshow("hist", histImage);
#else
	int width = darkchannel.cols;
	int height = darkchannel.rows;
	struct pixel table[256] = { 0 };
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int cur_val = darkchannel.at<uchar>(i, j);
			table[cur_val].value++;
			Point2i cur_pos(j, i);
			table[cur_val].pos.push_back(cur_pos);
		}
	}
	for (int k = 150; k < 256; k++) {
		cout << "k: " << table[k].value << endl;
		int size = table[k].pos.size();
		for (int i = 0; i < size; i++) {
			mask.at<uchar>(table[k].pos[i]) = 255;
		}
	}
#endif //DRAWHIST
}


void calcDarkChannelByIllumi(Mat& darkchannel, Mat& input, int radius) {


	int height = input.rows;
	int width = input.cols;
	darkchannel.create(height, width, CV_8UC1);
	
	Mat gray;
	cvtColor(input, gray, CV_BGR2GRAY);
	Scalar mean_gray = mean(gray);
	Mat gray_diff = gray - mean_gray[0];
	threshold(gray_diff, gray_diff, 5,255,THRESH_BINARY_INV);
	imshow("gray_diff", gray_diff);
	int st_row, ed_row;
	int st_col, ed_col;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			st_row = i - radius, ed_row = i + radius;
			st_col = j - radius, ed_col = j + radius;

			st_row = st_row < 0 ? 0 : st_row;
			ed_row = ed_row >= height ? (height - 1) : ed_row;
			st_col = st_col < 0 ? 0 : st_col;
			ed_col = ed_col >= width ? (width - 1) : ed_col;


			int min = 300;
			int max = 0;
			if (gray_diff.at<uchar>(i, j) == 255) {
				for (int m = st_row; m <= ed_row; m++)
				{
					for (int n = st_col; n <= ed_col; n++)
					{
						
							for (int k = 0; k < 3; k++)
							{

								int cur = input.at<Vec3b>(m, n)[k];
								if (cur < min)
									min = cur;
							}
						


					}
				}
			}
			else {
				min = 0;
			}
			darkchannel.at<uchar>(i, j) = min;
			
		}
	}
	//Mat tmp;
	//darkchannel.copyTo(tmp);
	//imshow("darkchannel", tmp);

}

void calcBrightBrightChannel(Mat& src, Mat& bbchannel,int radius) {
	int height = src.rows;
	int width = src.cols;
	bbchannel.create(height, width, CV_8UC1);
	Mat src_hsv;
	cvtColor(src, src_hsv, CV_BGR2HSV);

	int st_row, ed_row;
	int st_col, ed_col;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			st_row = i - radius, ed_row = i + radius;
			st_col = j - radius, ed_col = j + radius;

			st_row = st_row < 0 ? 0 : st_row;
			ed_row = ed_row >= height ? (height - 1) : ed_row;
			st_col = st_col < 0 ? 0 : st_col;
			ed_col = ed_col >= width ? (width - 1) : ed_col;


			int min = 300;
			int max = 0;
			for (int m = st_row; m <= ed_row; m++)
			{
				for (int n = st_col; n <= ed_col; n++)
				{
						int cur = src_hsv.at<Vec3b>(m, n)[2];

						if (cur > max)
							max = cur;
					
				}
			}
			
			bbchannel.at<uchar>(i, j) = max;
		}
	}
}

void calcMaxReflectChannelColorMap(Mat& src, Mat& dst, int radius) {
	int height = src.rows;
	int width = src.cols;
	dst.create(height, width, CV_8UC3);
	

	int st_row, ed_row;
	int st_col, ed_col;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			st_row = i - radius, ed_row = i + radius;
			st_col = j - radius, ed_col = j + radius;

			st_row = st_row < 0 ? 0 : st_row;
			ed_row = ed_row >= height ? (height - 1) : ed_row;
			st_col = st_col < 0 ? 0 : st_col;
			ed_col = ed_col >= width ? (width - 1) : ed_col;

			int max[3] = { 0 };
			for (int m = st_row; m <= ed_row; m++)
			{
				for (int n = st_col; n <= ed_col; n++)
				{
					for (int k = 0; k < 3; k++) {
						if (src.at<Vec3b>(m,n)[k] > max[k]) {
							max[k] = src.at<Vec3b>(m,n)[k];
						}
					}
				}
			}
			dst.at<Vec3b>(i, j)[0] = max[0];
			dst.at<Vec3b>(i, j)[1] = max[1];
			dst.at<Vec3b>(i, j)[2] = max[2];
		}
	}
	

}

void calcRedChannel(Mat& src, Mat& redchannel, int radius) {
	int height = src.rows;
	int width = src.cols;
	redchannel.create(height, width, CV_8UC1);
	Mat src_hsv;
	
	int st_row, ed_row;
	int st_col, ed_col;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			st_row = i - radius, ed_row = i + radius;
			st_col = j - radius, ed_col = j + radius;

			st_row = st_row < 0 ? 0 : st_row;
			ed_row = ed_row >= height ? (height - 1) : ed_row;
			st_col = st_col < 0 ? 0 : st_col;
			ed_col = ed_col >= width ? (width - 1) : ed_col;


			int min = 300;
			int max = 0;
			for (int m = st_row; m <= ed_row; m++)
			{
				for (int n = st_col; n <= ed_col; n++)
				{
					int blue_c = src.at<Vec3b>(m, n)[0];
					int green_c = src.at<Vec3b>(m, n)[1];
					int red_c = src.at<Vec3b>(m, n)[2];
					int min_local = MIN(MIN(blue_c, green_c), 255 - red_c);
					min = MIN(min_local, min);
				}
			}

			redchannel.at<uchar>(i, j) = min;
		}
	}
}