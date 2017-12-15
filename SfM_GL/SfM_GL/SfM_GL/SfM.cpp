#include "SfM.h"
#define OK 0;
enum {
	SLIC = 100,
	SLICO = 101,
	MSLIC = 102,
};

enum {
	OPTICAL_FLOW = 1,
	GMS = 2,
};

struct super_pixel_vertex {
	int min_x;
	int min_y;
	int max_x;
	int max_y;
};

int sfm_add_image(sfm_program * const sfm, Mat& p_input_image) {
	sfm->input_images.push_back(p_input_image);
	return OK;
}

int sfm_super_pixel(sfm_program * const sfm) {
	int min_element_size = 20;
	int region_size = 20;
	int num_iterations = 3;
	int ruler = 10;
	Mat img_1 = sfm->input_images[0];
	Mat converted;
	cvtColor(img_1, converted, CV_BGR2HSV);
	Ptr<ximgproc::SuperpixelSLIC> superpixel = ximgproc::createSuperpixelSLIC(converted, SLICO,region_size,(float)ruler);
	superpixel->iterate(num_iterations);
	if (min_element_size > 0)
		superpixel->enforceLabelConnectivity(min_element_size);
	superpixel->getLabels(sfm->super_pixel_label);
	superpixel->getLabelContourMask(sfm->contour_mask, true);
	sfm->num_superpixel= superpixel->getNumberOfSuperpixels();
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

		calcOpticalFlowFarneback(pregray, gray, sfm->u_flow, 0.5, 3, 15, 3, 5, 1.2, 0);
	}
	return OK;
}


int sfm_drawOptFlowMap(sfm_program * const sfm, Scalar color) {
	int step = 16;
	Mat base_image;
	double line_width = 1.5;
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

int sfm_superpixel_image(sfm_program *const sfm, Scalar color) {
	Mat mask = sfm->contour_mask;
	Mat refer_image;
	sfm->input_images[0].copyTo(refer_image);
	refer_image.setTo(color, mask);
	sfm->super_pixel_image = refer_image;
	return OK;
}

static int get_superpixel_vertex(sfm_program * const sfm, vector<KeyPoint>& pt1s) {
	int num = sfm->num_superpixel + 1;
	vector<super_pixel_vertex> pts(num);
	Mat label = sfm->super_pixel_label;
	for (int i = 0; i < label.rows; i++) {
		for (int j = 0; j < label.cols; j++) {
			int label_val = label.at<int>(i, j);

		}
	}

}
int sfm_get_keyPoints(sfm_program *const sfm, int method) {
	switch (method)
	{
	case OPTICAL_FLOW:
	{

	}
	break;
	case GMS:
	{

	}
	break;
	default:
		break;
	}
}