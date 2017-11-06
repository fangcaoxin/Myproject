#include "opticalModel.h"
#include "param.h"
#include "core.h"
#include <opencv2/highgui/highgui.hpp>

void opticalModelCorrect(Mat& src, Mat& dst) {
	int width = src.cols;
	int height = src.rows;
	Mat bbchannel,darkchannel,colorRelfectMap;
	vector<Mat> colorRelfectMapChannels;
	//calcBrightBrightChannel(src, bbchannel, 7);
	calcMaxReflectChannelColorMap(src, colorRelfectMap, 7);
	calcDarkChannel( darkchannel, bbchannel,src, 7);
	guidedFilter(src, bbchannel, bbchannel, 32, 1e-2);
	guidedFilter(src, colorRelfectMap, colorRelfectMap, 32, 1e-2);
	bbchannel.convertTo(bbchannel, CV_32FC1);
	normalize(bbchannel, bbchannel, 0, 1, NORM_MINMAX, -1, Mat());
	colorRelfectMap.convertTo(colorRelfectMap, CV_32FC3);
	normalize(colorRelfectMap, colorRelfectMap, 0, 1, NORM_MINMAX, -1, Mat());
	split(colorRelfectMap, colorRelfectMapChannels);
	Mat ita_b, ita_g, ita_r;
	ita_b = colorRelfectMapChannels[0].mul(1 / bbchannel);
	ita_g = colorRelfectMapChannels[1].mul(1 / bbchannel);
	ita_r = colorRelfectMapChannels[2].mul(1 / bbchannel);
	double minVal, maxVal;
	/*imshow("ita_b", ita_b);
	imshow("ita_g", ita_g);
	imshow("ita_r", ita_r);*/
	minMaxIdx(bbchannel, &minVal, &maxVal);
	cout << "min " << minVal << " max " << maxVal << endl;
	vector<Mat> bgr_channels;
	Mat src_colorRefelectFree;
	Mat airLight_mat = bbchannel;
	Mat cb_dis(src.size(), CV_32FC1);
	Mat lb_ist(src.size(), CV_32FC1);
	dst.create(src.size(), CV_32FC3);
	src.convertTo(src, CV_32FC3);
	normalize(src, src, 0, 1, NORM_MINMAX, -1, Mat());
	split(src, bgr_channels);
	bgr_channels[0] = bgr_channels[0].mul(1/ita_b);
	bgr_channels[1] = bgr_channels[1].mul(1 / ita_g);
	bgr_channels[2] = bgr_channels[2].mul(1 / ita_r);
	/*imshow("b_new", bgr_channels[0]);
	imshow("g_new", bgr_channels[1]);
	imshow("r_new", bgr_channels[2]);*/
	merge(bgr_channels, src_colorRefelectFree);
	normalize(src_colorRefelectFree, src_colorRefelectFree, 0, 255, NORM_MINMAX);
	src_colorRefelectFree.convertTo(src_colorRefelectFree, CV_8UC3);
	imshow("src_colorRefelectFree", src_colorRefelectFree);
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
	/*Mat bb_channel_colorRelFree;
	calcBrightBrightChannel(src_colorRefelectFree, bb_channel_colorRelFree, 7);
	guidedFilter(src, bb_channel_colorRelFree, bb_channel_colorRelFree, 32, 1e-2);
	vector<Mat> bgr;
	bb_channel_colorRelFree.convertTo(bb_channel_colorRelFree, CV_32FC3);
	normalize(bb_channel_colorRelFree, bb_channel_colorRelFree, 0, 1, NORM_MINMAX);
	imshow("bbchannleColorRelFree", bb_channel_colorRelFree);*/
	
	double dis_camera_constant = 1;//m
	double dis_b = pow(NRER_BLUE,  dis_camera_constant);
	double dis_g= pow(NRER_GREEN, dis_camera_constant);
	double dis_r = pow(NRER_GREEN,  dis_camera_constant);
	double dis_led_constant =dis_camera_constant + 196;//mm
	double view_width = dis_camera_constant / FOCUS_LENGTH*SENOR_WIDTH;
	double view_height = dis_camera_constant / FOCUS_LENGTH*SENSOR_HEIGHT;
	double pixel_width = view_width / width;
	double pixel_height = view_height / height;

	Point center(width / 2, height / 2);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			/*Point3d cb_dis_cur,lb_dis_cur;
			cb_dis_cur.x = (j - center.x)*pixel_width;
			cb_dis_cur.y = (i - center.y)*pixel_height;
			cb_dis_cur.z = dis_camera_constant;
			cb_dis.at<float>(i, j) = norm(cb_dis_cur);*/
			float air_light = darkchannel_l.at<float>(i, j);
			air_light = max(air_light, (float)0.1);
			air_light = min(air_light, (float)1);
			//float air_light = 3.3;
			
			/*double t_b = dis_b*air_light / maxVal;
			double t_g = dis_g*air_light / maxVal;
			double t_r = dis_r*air_light / maxVal;*/
			float t_b = trans.at<float>(i, j);
			float t_g = trans.at<float>(i, j);
			float t_r = trans.at<float>(i, j);
			//cout << "trans " << t_b << " " << t_g << " " << t_r << endl;
			
				dst.at<Vec3f>(i, j)[0] = (src.at<Vec3f>(i, j)[0] - air_light) / max(t_b,(float)0.75) + air_light;
				dst.at<Vec3f>(i, j)[1] = (src.at<Vec3f>(i, j)[1] - air_light) / max(t_g, (float)0.75) + air_light;
				dst.at<Vec3f>(i, j)[2] = (src.at<Vec3f>(i, j)[2] - air_light) / max(t_r, (float)0.75) + air_light;
				/*dst.at<Vec3f>(i, j)[0] = (src.at<Vec3f>(i, j)[0]) / t_b;
				dst.at<Vec3f>(i, j)[1] = (src.at<Vec3f>(i, j)[1]) / t_g;
				dst.at<Vec3f>(i, j)[2] = (src.at<Vec3f>(i, j)[2]) / t_r; */
			
			

			
			/*cout << "the dst is " << dst.at<Vec3f>(i, j)[0]<<
				dst.at<Vec3f>(i, j)[1]<<
				dst.at<Vec3f>(i, j)[2] << endl;*/
		}
		
	}
	normalize(dst, dst, 0, 1, NORM_MINMAX, -1, Mat());
	/*dst.convertTo(dst, CV_8UC3);*/
	


}