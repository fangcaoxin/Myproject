#include "opticalModel.h"
#include "param.h"
#include "core.h"
#include <opencv2/highgui/highgui.hpp>

#define TRANSESTIMATION
#define COLORREFLECTFREE
#define BRIGHTCHANNEL
void opticalModelCorrect(Mat& src, Mat& dst) {
	int width = src.cols;
	int height = src.rows;
	int radius = cvRound(MIN(width, height)*0.015);
	dst.create(src.size(), CV_8UC3);
	Mat bbchannel,darkchannel,colorRelfectMap,brightchannel;
	vector<Mat> colorRelfectMapChannels;

#ifdef COLORREFLECTFREE
	/*caculate bright R,G,B channel*/
	calcMaxReflectChannelColorMap(src, colorRelfectMap, radius);
#ifdef BRIGHTCHANNEL
	/*caculate bright birightness channel according to brightness channel*/
	calcBrightBrightChannel(src, bbchannel, radius);
	bbchannel.convertTo(bbchannel, CV_32FC1);
#endif //BRIGHTCHANNEL

	/*caculate color of ambient illumination*/
	colorRelfectMap.convertTo(colorRelfectMap, CV_32FC3);
	split(colorRelfectMap, colorRelfectMapChannels);
	brightchannel.convertTo(brightchannel, CV_32FC1);
	Mat ita_b, ita_g, ita_r,ita_merge;
	vector<Mat> color_ita;
	ita_r = colorRelfectMapChannels[0].mul(1 / brightchannel);
	ita_g = colorRelfectMapChannels[1].mul(1 / brightchannel);
	ita_b = colorRelfectMapChannels[2].mul(1 / brightchannel);
	color_ita.push_back(ita_b);
	color_ita.push_back(ita_g);
	color_ita.push_back(ita_r);
	merge(color_ita, ita_merge);
	normalize(ita_merge, ita_merge, 0, 1, NORM_MINMAX);
	src.convertTo(src, CV_32FC3);
	normalize(src, src, 0, 1, NORM_MINMAX, -1, Mat());
	guidedFilter(src, ita_merge, ita_merge, 32, 1e-2);
	/*to get the color ambient illumination free src image */
	Mat src_colorRefelectFree = src.mul(1 / ita_merge);
	normalize(src_colorRefelectFree, src_colorRefelectFree, 0, 255, NORM_MINMAX);
	src_colorRefelectFree.convertTo(src_colorRefelectFree, CV_8UC3);
#else
	Mat src_colorRefelectFree = src;
#endif /* COLORREFLECTFREE */
	
	/* caluculate src_colorRefelctFree for bbchannel */
	Mat light_brightchannel, light_darkchannel;
	calcDarkChannel(light_darkchannel, light_brightchannel, src_colorRefelectFree, radius);
	guidedFilter(src, light_brightchannel, light_brightchannel, 32, 1e-2);
	//imshow("airlight", light_brightchannel);
	//imwrite("airlight.jpg", light_brightchannel);
//#define TRANSESTIMATION_ESTI
#ifdef TRANSESTIMATION_ESTI
	Mat transmission;
	Mat darkchannel_light_brightchannel, brightchannel_light_brightchannel;
	calcDarkChannel(darkchannel_light_brightchannel, brightchannel_light_brightchannel, light_brightchannel, radius);
	light_darkchannel.convertTo(light_darkchannel, CV_32FC1);
	//light_brightchannel.convertTo(light_brightchannel, CV_32FC1);

	darkchannel_light_brightchannel.convertTo(darkchannel_light_brightchannel, CV_32FC1);
	transmission =1-light_darkchannel.mul(1 / darkchannel_light_brightchannel);

	guidedFilter(src, transmission,transmission, 32, 1e-2);
	/*normalize(transmission, transmission, 0, 255, NORM_MINMAX);
	transmission.convertTo(transmission, CV_8UC3);
	imshow("transmission", transmission);
	imwrite("transmission.jpg", transmission);*/
#else
	/*vector<Mat> bgr_channels;
	split(src_colorRefelectFree, bgr_channels);*/
	Mat transmission = colorRelfectMapChannels[2];
	transmission.convertTo(transmission, CV_32FC1);
	normalize(transmission, transmission, 0.1, 0.6, NORM_MINMAX);
	//imshow("transmission", transmission);
#endif //TRANSESTIMATION_ESTI
#ifdef POSITIONPARAM
	double dis_camera_constant = 1.5;//m
	double dis_b = pow(NRER_BLUE,  dis_camera_constant);
	double dis_g= pow(NRER_GREEN, dis_camera_constant);
	double dis_r = pow(NRER_RED,  dis_camera_constant);
	double dis_led_constant =dis_camera_constant + 196;//mm
	double view_width = dis_camera_constant / FOCUS_LENGTH*SENOR_WIDTH;
	double view_height = dis_camera_constant / FOCUS_LENGTH*SENSOR_HEIGHT;
	double pixel_width = view_width / width;
	double pixel_height = view_height / height;
	Point center(width / 2, height / 2);
#endif //POSITIONPARAM

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			
			Vec3b src_colorFree = src_colorRefelectFree.at<Vec3b>(i, j);

			int air_light = light_brightchannel.at<uchar>(i, j);
			float trans = MAX(transmission.at<float>(i, j),(float)0.3);
			float dst_0 = ((float)(src_colorFree[0] / 255.) - (float)(air_light / 255.)) / (trans*1.1) + (float)(air_light / 255.);
			float dst_1 = ((float)(src_colorFree[1] / 255.) - (float)(air_light / 255.)) / (trans*1.1) + (float)(air_light / 255.);
			float dst_2 = ((float)(src_colorFree[2] / 255.) - (float)(air_light / 255.)) / trans + (float)(air_light / 255.);
			/*float dst_0 = ((float)(src_colorFree[0] / 255.) - air_light[0]) / trans + air_light[0];
			float dst_1 = ((float)(src_colorFree[1] / 255.) - air_light[1])/ trans + air_light[1];
			float dst_2 = ((float)(src_colorFree[2] / 255.) - air_light[2]) / trans + air_light[2];*/
			dst_0 = dst_0*255.;
			dst_1 = dst_1*255.;
			dst_2 = dst_2*255.;

			dst_0 = dst_0 > 255 ? 255 : (dst_0 < 0 ? 0 : dst_0);
			dst_1 = dst_1 > 255 ? 255 : (dst_1 < 0 ? 0 : dst_1);
			dst_2 = dst_2 > 255 ? 255 : (dst_2 < 0 ? 0 : dst_2);
			dst.at<Vec3b>(i, j)[0] = cvRound(dst_0);
			dst.at<Vec3b>(i, j)[1] = cvRound(dst_1);
			dst.at<Vec3b>(i, j)[2] = cvRound(dst_2);
			//cout << "the src is " << src.at<Vec3f>(i, j)<<endl;
			//cout << "the dst is " << dst.at<Vec3f>(i, j) << endl;
		}
		
	}
	
}