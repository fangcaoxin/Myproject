#include "core.h"
#include <opencv2/highgui/highgui.hpp>
/**paper
* An Underwater Color Image Quality Evalution Metric,IEEE Transactions On Image Processing
* Formula :UCIQUE=c1*sigma_c+c2*con_l+c3*u_s 
* sigma_c is the standard deviatin of chroma,
* con_l is the contrast of luminance, the difference between the bottom 1% and the top 1% of all pixel value in luminance channel
* u_s is the average of saturation
* c1,c2,c3 are weighted coefficients
*/
double evaluationScore(Mat& src_original) {
	double c1 = 0.4680;
	double c2 = 0.2745;
	double c3 = 0.2575;
	Mat src;
	src_original.copyTo(src);
	Mat src_lab,src_float;
	vector<Mat> lab_channels;
	Scalar b_mean, b_stddev,a_mean,a_stddev;
	if (src.type() == CV_8UC3) {
		src.convertTo(src_float, CV_32FC3);
		normalize(src_float, src_float, 0, 1, NORM_MINMAX, -1, Mat());
	}
	else if(src.type()==CV_32FC3){
		src_float = src;
		normalize(src_float, src_float, 0, 1, NORM_MINMAX, -1, Mat());
	}
	else {
		cout << "not support " << endl;
		return 0;
	}
	
	
	cvtColor(src_float, src_float,CV_BGR2Lab);
	normalize(src_float, src_float, 0, 1, NORM_MINMAX, -1, Mat());
	split(src_float, lab_channels);
	meanStdDev(lab_channels[0], a_mean, a_stddev);
	meanStdDev(lab_channels[1], b_mean, b_stddev);
	Mat flat;
	vector<float> low_value, high_value;
	lab_channels[2].reshape(1, 1).copyTo(flat);
	cv::sort(flat, flat, SORT_ASCENDING);
	int area = flat.cols*0.01;
	for (int i = 0; i < area; i++) {
		low_value.push_back(flat.at<float>(0, i));
	}
	for (int j = flat.cols - 1; j >= flat.cols - area; j--) {
		high_value.push_back(flat.at<float>(0, j));
	}
	double low_high_dot = 0.,high_dot=0.,low_dot=0.;
	for (int k = 0; k < area; k++) {
		low_high_dot += low_value[k] * high_value[k];
		high_dot += high_value[k] * high_value[k];
		low_dot += low_value[k] * low_value[k];
	}
	double low_norm = norm(low_value);
	double high_norm = norm(high_value);
	double low_high_norm = norm(low_value, high_value);
	//double con_l=low_high_norm/(low_norm*high_norm);
//double con_l = 1-low_high_dot / sqrt(low_dot*high_dot);
	double con_l = low_high_norm*low_high_norm / area;
	double score = c1*a_stddev[0] + c2*con_l + c3*b_mean[0];
	return score;
}