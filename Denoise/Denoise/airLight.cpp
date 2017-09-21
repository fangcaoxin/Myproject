#include <stdio.h>
#include "core.h"

#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
using std::sort;

struct Pixel {
	int value;
	int i, j;
};

void calcAirLight(Mat& darkChannel, Mat& input, double A[]) {
	int height = input.rows;
	int width = input.cols;

	struct Pixel *v_darkchannel = (struct Pixel *)malloc(sizeof(struct Pixel) * height * width);
	int count = 0;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int value = darkChannel.at<uchar>(i, j);
			struct Pixel p = { value,i,j };
			v_darkchannel[count++] = p;
		}
	}
	sort(v_darkchannel, v_darkchannel + count, [](struct Pixel &a, struct Pixel &b) { return a.value > b.value; });
	Mat mask(height, width, CV_8UC1,Scalar(0));
	for (int i = 0; i < count*0.001; i++) {
		struct Pixel p = v_darkchannel[i];
		mask.at<uchar>(p.i, p.j) = 255;
	}
	//imshow("Mask", mask);
	for (int k = 0; k < 3; k++)
	{
		struct Pixel *v_channel = (struct Pixel *)malloc(sizeof(struct Pixel) * height * width);
		int count = 0;

		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				int flag = mask.at<uchar>(i, j);
				if (flag == 0)
					continue;
				int value = input.at<Vec3b>(i, j)[k];
				
				struct Pixel p = { value, i, j };
				v_channel[count++] = p;
			}
		}

		sort(v_channel, v_channel + count, [](struct Pixel &a, struct Pixel &b) { return a.value>b.value; });

		double channel_airlight = 0;
		for (int i = 0; i < count * 0.01; i++)
		{
			channel_airlight += v_channel[i].value;
		}
		channel_airlight = channel_airlight / (count * 0.01);
		A[k] = channel_airlight;

		free(v_channel);
	}
	free(v_darkchannel);
}