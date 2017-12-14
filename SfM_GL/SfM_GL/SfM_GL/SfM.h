#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ximgproc/slic.hpp>
#include <opencv2/video/tracking.hpp>

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
	Mat super_pixel_label_out;
	Mat c_flow_map;
	int a;
	
};

int sfm_add_image(sfm_program * const sfm, Mat &p_input_image);

int sfm_feature_matching(sfm_program * const sfm, 
	const Mat& img_1,
	const Mat& img_2,
	vector<KeyPoint>& keypts1,
	vector<KeyPoint>& keypts2,
	vector<DMatch>* mathes,
	int method);

int sfm_optical_flow(sfm_program * const sfm);

int sfm_super_pixel(sfm_program * const sfm, 
	const Mat& img_1);

int sfm_drawOptFlowMap(sfm_program * const sfm);