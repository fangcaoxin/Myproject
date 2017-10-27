#include "localColorCorrection.h"
#include <opencv2/highgui/highgui.hpp>
void localColorCorrection(Mat& src, Mat& dst) {
	vector<Mat>bgr;
	vector<Mat> correct(3);
	vector<Mat> correct_2(3);
	vector<Mat> color_corrected_list(3);
	vector<Mat> composited_list(3);
	vector<Mat> composited_split;
	src.convertTo(src, CV_32FC3);
	normalize(src, src, 0, 1, NORM_MINMAX, -1, Mat());
	split(src, bgr);
	/*normalize(bgr[0], bgr[0], 0, 1, NORM_MINMAX, -1, Mat());
	normalize(bgr[1], bgr[1], 0, 1, NORM_MINMAX, -1, Mat());
	normalize(bgr[2], bgr[2], 0, 1, NORM_MINMAX, -1, Mat());*/
	Mat r_pow;
	pow(bgr[2], 1.2, r_pow);
	//imshow("r_pow", r_pow); ok
	Mat attenuation_map = 1 - r_pow;
	//imshow("attenation map", attenuation_map); ok
	Mat composite_image,src1,src2;
	Scalar mean_value = mean(src);
	double mean_b = mean_value[0];
	double mean_g = mean_value[1];
	double mean_r = mean_value[2];
	multiply(r_pow, Scalar(mean_b), correct[0]);
	multiply(r_pow, Scalar(mean_g), correct[1]);
	multiply(r_pow, Scalar(mean_r), correct[2]);
	
	correct_2[0] = attenuation_map.mul(bgr[0]);
	correct_2[1] = attenuation_map.mul(bgr[1]);
	correct_2[2] = attenuation_map.mul(bgr[2]);
	composited_list[0] = correct[0] + correct_2[0];
	composited_list[1] = correct[1] + correct_2[1];
	composited_list[2] = correct[2] + correct_2[2];
	normalize(composited_list[0], composited_list[0], 0, 1, NORM_MINMAX, -1, Mat());
	normalize(composited_list[1], composited_list[1], 0, 1, NORM_MINMAX, -1, Mat());
	normalize(composited_list[2], composited_list[2], 0, 1, NORM_MINMAX, -1, Mat());
	//merge(composited_list, composite_image);
	
	
	//imshow("compostie image", composite_image);
	//split(composite_image, composited_split);
	Mat color_corrected;
	color_corrected_list[0] = attenuation_map.mul(composited_list[0]) + r_pow.mul(bgr[0]);
	color_corrected_list[1] = attenuation_map.mul(composited_list[1]) + r_pow.mul(bgr[1]);
	color_corrected_list[2] = attenuation_map.mul(composited_list[2]) + r_pow.mul(bgr[2]);
	normalize(color_corrected_list[0], color_corrected_list[0], 0, 1, NORM_MINMAX, -1, Mat());
	normalize(color_corrected_list[1], color_corrected_list[1], 0, 1, NORM_MINMAX, -1, Mat());
	normalize(color_corrected_list[2], color_corrected_list[2], 0, 1, NORM_MINMAX, -1, Mat());
	merge(color_corrected_list, color_corrected);
	normalize(color_corrected, dst, 0, 255, NORM_MINMAX, -1, Mat());
	dst.convertTo(dst, CV_8UC3);
}