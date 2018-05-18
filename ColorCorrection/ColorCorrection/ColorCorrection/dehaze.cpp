#include "dehaze.h"
#include <opencv2/highgui/highgui.hpp>

void dehaze(cv::Mat& input, cv::Mat& recover) {
	int height = input.rows;
	int width = input.cols;

	cv::Mat darkchannel;
	cv::Mat brightchannel;
	cv::Mat transmission;
	cv::Mat refine_transmission;
    recover= cv::Mat(height, width, CV_8UC3);
	int darkchannelradius = cvRound(MIN(width, height)*0.015);
	//int darkchannelradius = 10;
	double Airlight[3] = { 0,0,0 };
	
	printf("CalcDarkChannel...");
	calcDarkChannel(darkchannel, brightchannel, input, darkchannelradius);
	//calcDarkChannelByIllumi(darkchannel, input, darkchannelradius);

	printf("CalcAirLight...");
	calcAirLight(darkchannel, input, Airlight);

	printf("CalcTransmission...");
	calcTransmission(transmission, input, Airlight, darkchannelradius);

	printf("GuidedFilterColor...");
	cv::ximgproc::guidedFilter(input, transmission, refine_transmission, 32, 1e-2);

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

void dehazeMY(cv::Mat image, cv::Mat &mydehaze)
{
	std::vector<cv::Mat>bgr;
	split(image, bgr);
	bgr[0].convertTo(bgr[0], CV_32FC1);
	bgr[1].convertTo(bgr[1], CV_32FC1);
	bgr[2].convertTo(bgr[2], CV_32FC1);
	cv::normalize(bgr[0], bgr[0], 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
	cv::normalize(bgr[1], bgr[1], 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
	cv::normalize(bgr[2], bgr[2], 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

	cv::Mat bright = cv::Mat::zeros(image.rows, image.cols, CV_8U);
	cv::Mat dark = cv::Mat::zeros(image.rows, image.cols, CV_8U);

	//CalcBrightChannel(bright, image, 0);
	//CalcDarkChannel(dark, image, 0);
	calcDarkChannel(dark, bright, image, 5);
	bright.convertTo(bright, CV_32FC1);
	dark.convertTo(dark, CV_32FC1);
	cv::normalize(bright, bright, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
	cv::normalize(dark, dark, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
	//Mat transmission = 1 - bright + dark; //key
	cv::Mat transmission = 1-dark;

	//Mat reddark=Mat::zeros(image.rows, image.cols, CV_8U);
	//CalcRedDarkChannel(reddark, image, 3);
	//reddark.convertTo(reddark, CV_32FC1);
	//normalize(reddark, reddark, 0, 1, NORM_MINMAX, -1, Mat() );
	//transmission=1-transmission;
	//Mat gray;
	//cvtColor(image, gray, CV_BGR2GRAY);
	cv::ximgproc::guidedFilter(image, transmission, transmission, 60, 1e-6);
	imshow("trans", transmission);
	double minVal, maxVal;
	minMaxLoc(transmission, &minVal, &maxVal);

	double A = minVal;
	std::cout << "myair=" << minVal << std::endl;
	cv::Mat tb = cv::Mat::ones(image.rows, image.cols, CV_32FC1);
	cv::Mat tg = cv::Mat::ones(image.rows, image.cols, CV_32FC1);
	cv::Mat tr = cv::Mat::ones(image.rows, image.cols, CV_32FC1);
	cv::Mat redtrans = cv::Mat::ones(image.rows, image.cols, CV_32FC3);

	for (int i = 0; i<image.cols; i++)
		for (int j = 0; j<image.rows; j++)
		{
			float t = max((double)transmission.at<float>(j, i), 0.001);
			//float t = max((double)transmission.at<float>(j, i)*exp(-1 * A), 0.75);
			//cout << "t" << t << " trans"<< transmission.at<float>(j, i)<<endl;
			tb.at<float>(j, i) = (bgr[0].at<float>(j, i) - t) / t+t;
			tg.at<float>(j, i) = (bgr[1].at<float>(j, i) - t) / t+t;
			tr.at<float>(j, i) = (bgr[2].at<float>(j, i)-t) / t+t;
			redtrans.at<cv::Vec3f>(j, i)[0] = 0.0;
			redtrans.at<cv::Vec3f>(j, i)[1] = 0.0;
			redtrans.at<cv::Vec3f>(j, i)[2] = transmission.at<float>(j, i);
		}

	//imshow("redt",redtrans);
	
	/*bgr[0] = tb;
	bgr[1] = tg;
	bgr[2] = tr;*/
	cv::normalize(bgr[0], bgr[0], 0, 1, NORM_MINMAX);
	cv::normalize(bgr[1], bgr[1], 0, 1, NORM_MINMAX);
	cv::normalize(bgr[2], bgr[2], 0, 1, NORM_MINMAX);
	cv::merge(bgr, mydehaze);

	//mydehaze.convertTo(mydehaze, CV_8UC3);
}

void dehazeByBright(Mat& src, Mat& dst) {
	Mat bbchannel;
	vector<Mat> channels;
	//dst.create(src.size(), CV_8UC3);
	Mat dst_tmp(src.size(), CV_32FC3);
	vector<Mat> bgr;
	calcBrightBrightChannel(src, bbchannel, 5);
	
	guidedFilter(src, bbchannel, bbchannel, 32, 1e-2);
	bbchannel.convertTo(bbchannel, CV_32FC1);
	normalize(bbchannel, bbchannel, 0, 1, NORM_MINMAX, -1, Mat());
	double minVal, maxVal;
	imshow("bbchannel", bbchannel);
	minMaxIdx(bbchannel, &minVal, &maxVal);
	cout << "min " << minVal << " max " << maxVal << endl;
	src.convertTo(src, CV_32FC3);
	normalize(src, src, 0, 1, NORM_MINMAX, -1, Mat());
	Mat trans_mat = bbchannel;
	split(src, channels);
	Mat tb = Mat::ones(src.rows, src.cols, CV_32FC1);
	Mat tg = Mat::ones(src.rows, src.cols, CV_32FC1);
	Mat tr = Mat::ones(src.rows, src.cols, CV_32FC1);
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			float trans = trans_mat.at<float>(i, j);
			trans = max(trans, (float)0.5);
			trans = min(trans, (float)1);
			float trans_r = trans*exp(0.2);
			dst_tmp.at<Vec3f>(i, j)[0] = (src.at<Vec3f>(i, j)[0]-trans) / trans+trans;
			dst_tmp.at<Vec3f>(i, j)[1] = (src.at<Vec3f>(i, j)[1]-trans) / trans+trans ;
			dst_tmp.at<Vec3f>(i, j)[2] = (src.at<Vec3f>(i, j)[2]-trans) / trans+trans ;
		/*	tb.at<float>(i, j) = channels[0].at<float>(i,j)*exp(-1.147);
			tg.at<float>(i, j) = channels[1].at<float>(i, j) * exp(-0.87);
			tr.at<float>(i, j) = channels[2].at<float>(i, j) * exp(-0.5);*/
			
		}
	}
	/*imshow("tb", tb);
	imshow("tg", tg);
	imshow("tr", tr);*/
	//bgr.push_back(tb);
	//bgr.push_back(tg);
	//bgr.push_back(tr);
	
	/*merge(bgr, dst);*/
	normalize(dst_tmp, dst, 0, 1, NORM_MINMAX);
	//dst.convertTo(dst, CV_8UC3);
	

}