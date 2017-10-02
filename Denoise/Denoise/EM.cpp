#include "EM.h"
#include <opencv2/ml.hpp>
using namespace cv::ml;

void EMSegmetation(Mat& img,Mat& label,int pixel_num,int num) {
	int width = img.cols;
	int height = img.rows;
	Mat img_tmp;
	Mat floatSource;
	cvtColor(img, img_tmp, CV_BGR2HSV);
	img_tmp.convertTo(floatSource, CV_32F);
	Mat samples(pixel_num, 3, CV_32FC1);
	int idx = 0;

	/* make samples*/
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			if (label.at<uchar>(y, x) == 1) {
				/*float sum = floatSource.at<Vec3f>(y, x)[0] + floatSource.at<Vec3f>(y, x)[1] + floatSource.at<Vec3f>(y, x)[2];
				Vec3f cur(floatSource.at<Vec3f>(y, x)[0] / sum, floatSource.at<Vec3f>(y, x)[1] / sum, floatSource.at<Vec3f>(y, x)[2] / sum);*/
				samples.at<Vec3f>(idx++, 0) = floatSource.at<Vec3f>(y,x);
				//cout << "floatSource" << cur << endl;
			}
		}
	}

	/*create EM*/
	Mat labels;
	Ptr<EM> em_model = EM::create();
	
	em_model->setClustersNumber(num);
	em_model->setCovarianceMatrixType(EM::COV_MAT_SPHERICAL);
	em_model->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
	em_model->trainEM(samples, noArray(), labels, noArray());

	Mat means = em_model->getMeans();
	Mat weights = em_model->getWeights();
	cout << "means: " << means << endl;
	cout << "weidths: " << weights << endl;
	/*Scalar mean_0, mean_1, dev_0, dev_1;
	meanStdDev(means.row(0), mean_0, dev_0);
	meanStdDev(means.row(1), mean_1, dev_1);*/
	//int flag = dev_0.val[0] > dev_1.val[0] ? 1 : 0;
	double max = 0;
	int flag = 0;
	for (int i = 0; i < num; i++) {
		if (means.at<double>(i, 0) >max) {
			max = means.at<double>(i, 0);
			flag = i;
		}
	}
	
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			int response=0;
			if (label.at<uchar>(y, x) == 1) {
				/*float sum = floatSource.at<Vec3f>(y, x)[0] + floatSource.at<Vec3f>(y, x)[1] + floatSource.at<Vec3f>(y, x)[2];
				Vec3f cur(floatSource.at<Vec3f>(y, x)[0] / sum, floatSource.at<Vec3f>(y, x)[1] / sum, floatSource.at<Vec3f>(y, x)[2] / sum);*/
				Vec3f cur = floatSource.at<Vec3f>(y, x);
				response = em_model->predict2(cur, noArray())[1];
				
				if (response == flag) {
					label.at<uchar>(y, x) = 1;
				}
				else {
					label.at<uchar>(y, x) = 0;
				}
			}
		}
	}

}