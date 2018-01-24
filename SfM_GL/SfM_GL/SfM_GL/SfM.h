#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ximgproc/slic.hpp>
#include <opencv2/video/tracking.hpp>
#include "sfm_triangulation.h"

//#pragma comment(lib,"ceres.lib")

using namespace cv;
using namespace std;
/* method */
enum {
	OPTICAL_FLOW_PATCH = 1,
	GMS = 2,
	OPTICAL_FLOW_POINT = 3,
};
#define METHOD OPTICAL_FLOW_PATCH

#define DSP_MIN 1e-7
#define DSP_MAX 1e-2
#define DSP_LVL 100

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
	Mat base_image;
	Mat src_image;
	int num_superpixel;
	vector<KeyPoint> keypts1;
	vector<KeyPoint> keypts2;
	vector<KeyPoint> keypts1_good;
	vector<KeyPoint> keypts2_good;
	vector<KeyPoint> correspImgPt;
	vector<DMatch> matches;
	vector<double> depths;
	vector<Matx34d> external_martix;
	vector<Point3d> Pt1;
	vector<Point3d> Pt2;
	vector<CloudPoint> pointcloud;
	vector<uchar> status;
	vector<vector<Point>> label_points;
};

int sfm_add_image(sfm_program * const sfm, Mat &p_input_image);

//int sfm_optical_flow(sfm_program * const sfm);

int sfm_super_pixel(sfm_program * const sfm);

int sfm_drawOptFlowMap(sfm_program * const sfm, Scalar color);

int sfm_motion_to_color(sfm_program *const sfm);

int sfm_superpixel_image(sfm_program *const sfm, Scalar color);

int sfm_get_keyPoints(sfm_program *const sfm, int frame_num);

int sfm_drawOptflowKps(sfm_program *const sfm);

int sfm_set_internal_matrix(sfm_program *const sfm, double f, double cx, double cy);

int sfm_get_external_matrix(sfm_program *const sfm);

int sfm_set_base_external_matrix(sfm_program *const sfm);

double sfm_triangulatePoints(sfm_program *const sfm, int frame_num);

Mat sfm_drawDepths(sfm_program *const sfm);

Mat sfm_draw_gms_matches(sfm_program *const sfm, Scalar color, int type);

Point3d sfm_reproj4Bundler(Point2f point_2d, Matx33d K, Matx34d external_martix, double d);

Point2f sfm_proj4Bundler(Point3d X, Matx33d K, Matx34d external_matrix);

int sfm_set_base_src_image(sfm_program *const sfm, int frame_num);

int sfm_photoconsistency_optimazation(sfm_program *const sfm);