#include "core.h"
#include <opencv2/highgui/highgui.hpp>
/**paper
* An Underwater Color Image Quality Evalution Metric,IEEE Transactions On Image Processing
* Formula :UCIQUE=c1*sigma_c+c2*con_l+c3*u_s 
* sigma_c is the standard deviatin of chroma,
* con_l is the contrast of luminance, the difference between the bottom 1% and the top 1% of all pixel value in luminance channel
* u_s is the average of saturation
* c1,c2,c3 are weighted coefficients
* lab color space chroma C_ab=sqrt(a*a+b*b)
* lab color space saturation S_ab=
*/
double evaluationScore_UCIQUE(Mat& src_original) {
	double c1 = 0.4680;
	double c2 = 0.2745;
	double c3 = 0.2575;
	Mat src;
	src_original.copyTo(src);
	Mat src_lab,src_float;
	vector<Mat> lab_channels;
	Scalar c_mean, c_stddev,s_mean,s_stddev;
	if (src.type() == CV_8UC3) {
		src.convertTo(src_float, CV_32FC3);
		normalize(src_float, src_float, 0, 1, NORM_MINMAX, -1, Mat());
		//src_float = src;
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
	//normalize(src_float, src_float, 0, 1, NORM_MINMAX, -1, Mat());
	cout << "value" << src_float.at<Vec3f>(10, 10) << endl;
	split(src_float, lab_channels);
	Mat l = lab_channels[0];
	Mat a = lab_channels[1];
	Mat b = lab_channels[2];
	Mat C_ab, S_ab;
	Mat S_ab_bottom;
	cv::sqrt(a.mul(a) + b.mul(b),C_ab);
	sqrt(a.mul(a) + b.mul(b) + l.mul(l), S_ab_bottom);
	S_ab = C_ab.mul(1 / S_ab_bottom);
	normalize(C_ab, C_ab, 0, 1, NORM_MINMAX);
	meanStdDev(C_ab, c_mean, c_stddev);
	meanStdDev(S_ab, s_mean, s_stddev);
	//Mat H_ab(src.size(),CV_32FC1);
	//for (int i = 0; i < src.rows; i++) {
	//	for (int j = 0; j < src.cols; j++) {
	//		H_ab.at<float>(i, j) = cvFastArctan(b.at<float>(i, j), a.at<float>(i, j));
	//		//cout << "H_ab value " << H_ab.at<float>(i, j)<<endl;
	//	}
	//}
	//normalize(H_ab, H_ab, 0, 1, NORM_MINMAX);
	//meanStdDev(H_ab, h_mean, h_stddev);
	Mat flat;
	vector<float> low_value, high_value;
	lab_channels[0].reshape(1, 1).copyTo(flat);
	cv::sort(flat, flat, SORT_ASCENDING);
	int area = flat.cols*0.01;
	double low_value_average = 0.;
	double high_value_average = 0.;

	for (int i = 0; i < area; i++) {
		low_value.push_back(flat.at<float>(0, i));
		low_value_average += flat.at<float>(0, i) / area;
	}
	for (int j = flat.cols - 1; j >= flat.cols - area; j--) {
		high_value.push_back(flat.at<float>(0, j));
		high_value_average += flat.at<float>(0, j) / area;
	}
	double con_l = (high_value_average - low_value_average)/100;
	//double con_l=norm(low_value, high_value)/(100*area);
	double score = c1*c_stddev[0] + c2*con_l + c3*s_mean[0];
	return score;
}