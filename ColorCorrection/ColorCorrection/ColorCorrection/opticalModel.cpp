#include "opticalModel.h"
#include "param.h"
#include "core.h"
#include <opencv2/highgui/highgui.hpp>

//#define TRANSESTIMATION
//#define COLORREFLECTFREE
void opticalModelCorrect(Mat& src, Mat& dst) {
	int width = src.cols;
	int height = src.rows;
	int radius = cvRound(MIN(width, height)*0.015);
	Mat bbchannel,darkchannel,colorRelfectMap,brightchannel;
	vector<Mat> colorRelfectMapChannels;

	/*caculate bright birightness channel according to brightness channel*/
	calcBrightBrightChannel(src, bbchannel, radius);
	guidedFilter(src, bbchannel, bbchannel, 32, 1e-2);
	bbchannel.convertTo(bbchannel, CV_32FC1);
	normalize(bbchannel, bbchannel, 0, 1, NORM_MINMAX, -1, Mat());
	double minVal, maxVal;
	minMaxIdx(bbchannel, &minVal, &maxVal);
	cout << "min " << minVal << " max " << maxVal << endl;
	/*caculate bright channel of RGB */
	calcDarkChannel(darkchannel, brightchannel, src, radius);
	guidedFilter(src, darkchannel, darkchannel, 32, 1e-2);
	darkchannel.convertTo(darkchannel, CV_32FC1);
	normalize(darkchannel, darkchannel, 0, 1, NORM_MINMAX, -1, Mat());
	imshow("darkchannel", darkchannel);

	calcMaxReflectChannelColorMap(src, colorRelfectMap, radius);
	//guidedFilter(src, colorRelfectMap, colorRelfectMap, 32, 1e-2);
	colorRelfectMap.convertTo(colorRelfectMap, CV_32FC3);
	normalize(colorRelfectMap, colorRelfectMap, 0, 1, NORM_MINMAX, -1, Mat());

	src.convertTo(src, CV_32FC3);
	normalize(src, src, 0, 1, NORM_MINMAX, -1, Mat());
#ifdef COLORREFLECTFREE
	/*caculate bright R,G,B channel*/
	calcMaxReflectChannelColorMap(src, colorRelfectMap, radius);
	//guidedFilter(src, colorRelfectMap, colorRelfectMap, 32, 1e-2);
	colorRelfectMap.convertTo(colorRelfectMap, CV_32FC3);
	normalize(colorRelfectMap, colorRelfectMap, 0, 1, NORM_MINMAX, -1, Mat());

#ifdef TRANSESTIMATION
	/*need to caculate darkchannel to estimate transimission */
	calcDarkChannel( darkchannel, brightchannel,src, radius);
#endif //TRANSESTIMATION
	
	/*caculate color of ambient illumination*/
	split(colorRelfectMap, colorRelfectMapChannels);
	Mat ita_b, ita_g, ita_r;
	ita_b = colorRelfectMapChannels[0].mul(1 / bbchannel);
	ita_g = colorRelfectMapChannels[1].mul(1 / bbchannel);
	ita_r = colorRelfectMapChannels[2].mul(1 / bbchannel);

	/*to get the color ambient illumination free src image */
	vector<Mat> bgr_channels;
	Mat src_colorRefelectFree;
	
	split(src, bgr_channels);
	bgr_channels[0] = bgr_channels[0].mul(1 / ita_b);
	bgr_channels[1] = bgr_channels[1].mul(1 / ita_g);
	bgr_channels[2] = bgr_channels[2].mul(1 / ita_r);
	merge(bgr_channels, src_colorRefelectFree);
	normalize(src_colorRefelectFree, src_colorRefelectFree, 0, 255, NORM_MINMAX);
	src_colorRefelectFree.convertTo(src_colorRefelectFree, CV_8UC3);
	imshow("src_colorRefelectFree", src_colorRefelectFree);
	/* caluculate src_colorRefelctFree for bbchannel */
	Mat bb_channel_colorRelFree;
	calcBrightBrightChannel(src_colorRefelectFree, bb_channel_colorRelFree, 7);
	guidedFilter(src, bb_channel_colorRelFree, bb_channel_colorRelFree, 32, 1e-2);
	bb_channel_colorRelFree.convertTo(bb_channel_colorRelFree, CV_32FC3);
	normalize(bb_channel_colorRelFree, bb_channel_colorRelFree, 0, 1, NORM_MINMAX);
#endif //COLORREFLECTFREE
	Mat airLight_mat = bbchannel;
	imshow("airlight", airLight_mat);
	dst.create(src.size(), CV_32FC3);
	
#ifdef TRANSESTIMATION
	Mat darkchannel_src_colorReflectFree, brightchannel_src_colorReflectFree;
	calcDarkChannel(darkchannel_src_colorReflectFree, brightchannel_src_colorReflectFree, src_colorRefelectFree, 7);
	Mat darkchannel_l, brightchannel_l;
	calcDarkChannel(darkchannel_l, brightchannel_l, brightchannel_src_colorReflectFree, 7);
	darkchannel_src_colorReflectFree.convertTo(darkchannel_src_colorReflectFree, CV_32FC1);
	darkchannel_l.convertTo(darkchannel_l, CV_32FC1);
	normalize(darkchannel_l, darkchannel_l, 0, 1, NORM_MINMAX, -1, Mat());
	normalize(darkchannel_src_colorReflectFree, darkchannel_src_colorReflectFree, 0, 1, NORM_MINMAX);
	Mat trans = 1 - darkchannel_src_colorReflectFree.mul(1/darkchannel_l);
	guidedFilter(src, trans, trans, 32, 1e-2);
#endif //TRANSESTIMATION
	
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
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			
			float dis = bbchannel.at<float>(i, j);
			Vec3f air_light;
			/*air_light= colorRelfectMap.at<Vec3f>(i, j);*/
			air_light[0] = darkchannel.at<float>(i, j);
			air_light[1] = darkchannel.at<float>(i, j);
			air_light[2] = darkchannel.at<float>(i, j);
			/*air_light = max(air_light, (float)0.1);
			air_light = min(air_light, (float)1);*/
			
			
			
#ifdef TRANSESTIMATION
			float t_b = trans.at<float>(i, j);
			float t_g = trans.at<float>(i, j);
			float t_r = trans.at<float>(i, j);
#else 
			float t_b = dis_b*dis / maxVal;
			float t_g = dis_g*dis / maxVal;
			float t_r = dis_r*dis / maxVal;
			//cout << "trans " << t_b << " " << t_g << " " << t_r << endl;
#endif //TRANSESTIMATION
			
				dst.at<Vec3f>(i, j)[0] = (src.at<Vec3f>(i, j)[0] - air_light[0]) / max(t_b,(float)0.3) + air_light[0];
				dst.at<Vec3f>(i, j)[1] = (src.at<Vec3f>(i, j)[1] - air_light[1]) / max(t_g, (float)0.3) + air_light[1];
				dst.at<Vec3f>(i, j)[2] = (src.at<Vec3f>(i, j)[2] - air_light[2]) / max(t_r, (float)0.3) + air_light[2];
				/*dst.at<Vec3f>(i, j)[0] = (src.at<Vec3f>(i, j)[0]) / t_b;
				dst.at<Vec3f>(i, j)[1] = (src.at<Vec3f>(i, j)[1]) / t_g;
				dst.at<Vec3f>(i, j)[2] = (src.at<Vec3f>(i, j)[2]) / t_r; */
			
			

			
			//cout << "the src is " << src.at<Vec3f>(i, j)<<endl;
			//cout << "the dst is " << dst.at<Vec3f>(i, j) << endl;
		}
		
	}
normalize(dst, dst, 0, 255, NORM_MINMAX, -1, Mat());
	dst.convertTo(dst, CV_8UC3);
	


}