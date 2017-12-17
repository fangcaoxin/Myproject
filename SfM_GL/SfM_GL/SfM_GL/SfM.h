#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ximgproc/slic.hpp>
#include <opencv2/video/tracking.hpp>
#include "sfm_triangulation.h"

using namespace cv;
using namespace std;

struct CloudPoint {
	Point3d pt;
	vector<int> imgpt_for_img;
	double reprojection_error;
};

struct sfm_program {
	vector<Mat> input_images;
	Matx33d internal_matrix;
	Mat u_flow;
	Mat super_pixel_label;
	Mat c_flow_map;
	Mat color_flow;
	Mat contour_mask;
	Mat super_pixel_image;
	Mat discoeff;
	Mat depth_map;
	int num_superpixel;
	vector<KeyPoint> keypts1;
	vector<KeyPoint> keypts2;
	vector<KeyPoint> keypts1_good;
	vector<KeyPoint> Keypts2_good;
	vector<KeyPoint> correspImgPt;
	vector<DMatch> matches;
	vector<double> depths;
	Matx34d external_martix;
	vector<CloudPoint> pointcloud;
};

int sfm_add_image(sfm_program * const sfm, Mat &p_input_image);

int sfm_optical_flow(sfm_program * const sfm);

int sfm_super_pixel(sfm_program * const sfm);

int sfm_drawOptFlowMap(sfm_program * const sfm, Scalar color);

int sfm_motion_to_color(sfm_program *const sfm);

int sfm_superpixel_image(sfm_program *const sfm, Scalar color);

int sfm_get_keyPoints(sfm_program *const sfm, int method);

int sfm_drawOptflowKps(sfm_program *const sfm);

int sfm_set_internal_matrix(sfm_program *const sfm, double f, double cx, double cy);

int sfm_get_external_matrix(sfm_program *const sfm);

int sfm_triangulatePoints(sfm_program *const sfm);

int sfm_drawDepths(sfm_program *const sfm);