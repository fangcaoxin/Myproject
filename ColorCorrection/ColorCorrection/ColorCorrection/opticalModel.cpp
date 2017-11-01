#include "opticalModel.h"
#include "param.h"

void opticalModelCorrect(Mat& src, Mat& dst) {
	int width = src.cols;
	int height = src.rows;

	Mat cb_dis(src.size(), CV_32FC1);
	Mat lb_ist(src.size(), CV_32FC1);
	dst.create(src.size(), CV_32FC3);
	src.convertTo(src, CV_32FC3);
	normalize(src, src, 0, 1, NORM_MINMAX, -1, Mat());
	double dis_camera_constant = 500;//mm
	double dis_led_constant =dis_camera_constant + 196;//mm
	double view_width = dis_camera_constant / FOCUS_LENGTH*SENOR_WIDTH;
	double view_height = dis_camera_constant / FOCUS_LENGTH*SENSOR_HEIGHT;
	double pixel_width = view_width / width;
	double pixel_height = view_height / height;

	Point center(width / 2, height / 2);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			Point3d cb_dis_cur,lb_dis_cur;
			cb_dis_cur.x = (j - center.x)*pixel_width;
			cb_dis_cur.y = (i - center.y)*pixel_height;
			cb_dis_cur.z = dis_camera_constant;
			cb_dis.at<float>(i, j) = norm(cb_dis_cur);
			dst.at<Vec3f>(i, j)[0] = src.at<Vec3f>(i, j)[0] / pow(NRER_BLUE, 2*norm(cb_dis_cur) / 1000);
			dst.at<Vec3f>(i, j)[1] = src.at<Vec3f>(i, j)[1] / pow(NRER_GREEN, 2*norm(cb_dis_cur) / 1000);
			dst.at<Vec3f>(i, j)[2] = src.at<Vec3f>(i, j)[2] / pow(NRER_RED, 2*norm(cb_dis_cur) / 1000);
			//cout << "the dst is " << dst.at<Vec3f>(i, j)[2] << endl;
		}
		
	}
	normalize(dst, dst, 0, 255, NORM_MINMAX, -1, Mat());
	dst.convertTo(dst, CV_8UC3);
	


}