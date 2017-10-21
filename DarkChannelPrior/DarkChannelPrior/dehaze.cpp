#include "dehaze.h"
#include <opencv2/highgui/highgui.hpp>

void dehaze(Mat& recover, Mat& input) {
	int height = input.rows;
	int width = input.cols;

	Mat darkchannel(height, width, CV_8UC1);
	Mat brightchannel(height, width, CV_8UC1);
	Mat transmission(height, width, CV_8UC1);
	Mat refine_transmission(height, width, CV_8UC1);

	int darkchannelradius = cvRound(MIN(width, height)*0.005);
	double Airlight[3] = { 0,0,0 };
	
	printf("CalcDarkChannel...");
	calcDarkChannel(darkchannel, brightchannel, input, darkchannelradius);

	printf("CalcAirLight...");
	calcAirLight(darkchannel, input, Airlight);

	printf("CalcTransmission...");
	calcTransmission(transmission, input, Airlight, darkchannelradius);

	printf("GuidedFilterColor...");
	guidedFilter(input, transmission, refine_transmission, 60, 1e-6);

	printf("CalcRecover...");
	calcRecover(recover, input, refine_transmission, Airlight);
}


void dehazeDC(Mat image, Mat &dehaze)
{
	Mat transmisson = Mat::ones(image.rows, image.cols, CV_32FC1);
	vector<Mat>trans, bgr;
	split(image, trans);
	split(image, bgr);
	Mat gray;
	cvtColor(image, gray, CV_BGR2GRAY);
	double Dxmax = 0.0;
	int patch = 1;
	bgr[0].convertTo(bgr[0], CV_32FC1);
	bgr[1].convertTo(bgr[1], CV_32FC1);
	bgr[2].convertTo(bgr[2], CV_32FC1);
	normalize(bgr[0], bgr[0], 0, 1, NORM_MINMAX, -1, Mat());
	normalize(bgr[1], bgr[1], 0, 1, NORM_MINMAX, -1, Mat());
	normalize(bgr[2], bgr[2], 0, 1, NORM_MINMAX, -1, Mat());
	double minValb = 0, maxValb = 0, minValg = 0, maxValg = 0, minValr = 0, maxValr = 0;
	for (int i = 0; i<image.cols; i++)
		for (int j = 0; j<image.rows; j++)
		{
			Rect d = Rect(i - patch / 2, j - patch / 2, patch, patch)&Rect(0, 0, image.cols, image.rows);

			minMaxLoc(bgr[0](d), &minValb, &maxValb);
			minMaxLoc(bgr[1](d), &minValg, &maxValg);
			minMaxLoc(bgr[2](d), &minValr, &maxValr);

			double maxValc = min(maxValb, maxValg);
			transmisson.at<float>(j, i) = maxValr - maxValc;

			if (transmisson.at<float>(j, i)>Dxmax)
				Dxmax = transmisson.at<float>(j, i);
		}
	transmisson = transmisson + (1 - Dxmax);
	guidedFilter(gray, transmisson, transmisson, 10, 1e-1);
	Mat redtrans = Mat::ones(image.rows, image.cols, CV_32FC3);

	minMaxLoc(transmisson, &minValr, &maxValr);
	cout << "redtair=" << minValr << endl;
	double A = minValr;

	Mat tb = Mat::ones(image.rows, image.cols, CV_32FC1);
	Mat tg = Mat::ones(image.rows, image.cols, CV_32FC1);
	Mat tr = Mat::ones(image.rows, image.cols, CV_32FC1);

	for (int i = 0; i<image.cols; i++)
		for (int j = 0; j<image.rows; j++)
		{
			float t = ((double)transmisson.at<float>(j, i));

			tb.at<float>(j, i) = (bgr[0].at<float>(j, i) - A) / t;
			tg.at<float>(j, i) = (bgr[1].at<float>(j, i) - A) / t;
			tr.at<float>(j, i) = (bgr[2].at<float>(j, i) - A) / t;
			redtrans.at<Vec3f>(j, i)[0] = 0.0;
			redtrans.at<Vec3f>(j, i)[1] = 0.0;
			redtrans.at<Vec3f>(j, i)[2] = transmisson.at<float>(j, i);
		}

	//imshow("redt", transmisson);
	trans[0] = tb + A;
	trans[1] = tg + A;
	trans[2] = tr + A;
	merge(trans, dehaze);
	//cv::imshow("dehaze", dehaze);
	//dehaze.convertTo(dehaze, CV_8UC3);
	for (int i = 5; i < 100; i++) {
		cout << dehaze.at<Vec3f>(i, 20) << endl;
	}

}