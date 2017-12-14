#include "SfM.h"
#define OK 0;
int sfm_add_image(sfm_program * const sfm, Mat& p_input_image) {
	sfm->input_images.push_back(p_input_image);
	return OK;
}

int sfm_super_pixel(sfm_program * const sfm,
	const Mat& img_1) {
	Ptr<ximgproc::SuperpixelSLIC> superpixel = ximgproc::createSuperpixelSLIC(img_1,100,10,10.0);
	superpixel->getLabels(sfm->super_pixel_label_out);
	return OK;
}

int sfm_optical_flow(sfm_program * const sfm) {
	Mat pregray, gray;
	Mat img_1, img_2;
	img_1 = sfm->input_images[0];
	img_2 = sfm->input_images[1];
	if (!img_1.empty() && !img_2.empty()) {
		cvtColor(img_1, pregray, CV_BGR2GRAY);
		cvtColor(img_2, gray, CV_BGR2GRAY);

		calcOpticalFlowFarneback(pregray, gray, sfm->u_flow, 0.5, 1, 15, 3, 5, 1.2, 0);
	}
	return OK;
}


int sfm_drawOptFlowMap(sfm_program * const sfm) {
	int step = 16;
	Mat base_image;
	double line_width = 1.5;
	Scalar color(0, 255, 0);
	sfm->input_images[0].copyTo(base_image);
	for (int y = 0; y < base_image.rows; y += step) {
		for (int x = 0; x < base_image.cols; x += step) {
			const Point2f& fxy = sfm->u_flow.at<Point2f>(y, x);
			line(base_image, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
			circle(base_image, Point(x, y), 2, color, -1);
		}
	}
	sfm->c_flow_map = base_image;
	return OK;
}