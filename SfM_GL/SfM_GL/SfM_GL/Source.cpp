#include <windows.h>
#include <GL/GL.h>
#include <GL/glut.h>
#include "SfM.h"
#include <glog/logging.h>
#include <ceres/ceres.h>
#define SET_NUM 10

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

void disp(void) {
	glClear(GL_COLOR_BUFFER_BIT);
	glBegin(GL_POLYGON);
	glColor3d(1.0, 0.0, 0.0);
	glVertex2d(-0.9, -0.9);
	glColor3d(0.0, 1.0, 0.0);
	glVertex2d(0.9, -0.9);
	glColor3d(0.0, 0.0, 1.0);
	glVertex2d(0.9, 0.9);
	glColor3d(1.0, 1.0, 0.0);
	glVertex2d(-0.9, 0.9);
	glEnd();
	glFlush();
}
struct CostFunctor {
	template <typename T>
	bool operator()(const T* const x, T* residual) const {
		residual[0] = T(10.0) - x[0];
		return true;
	}
};

int main(int argc, char ** argv) {
	//sfm_program p_sfm;
	//int method = 1;
	//Scalar color(0, 0, 255);
	////string img_file1 = "eval-data//urban//frame10.png";
	////string img_file2 = "eval-data//urban//frame11.png";
	//for (int k = 0; k < SET_NUM; k++) {
	//	string image_name = to_string(k) + ".jpg";
	//	Mat img = imread(image_name);
	//	sfm_add_image(&p_sfm, img);
	//}
	//sfm_set_base_image(&p_sfm);
	//sfm_super_pixel(&p_sfm);
	////sfm_superpixel_image(&p_sfm, color);
	//sfm_get_keyPoints(&p_sfm, method);
	//sfm_set_internal_matrix(&p_sfm, 1057.14, p_sfm.base_image.cols/2,p_sfm.base_image.rows/2);
	//sfm_get_external_matrix(&p_sfm);
	//double pro_error = sfm_triangulatePoints(&p_sfm);
	//Mat depth_map = sfm_drawDepths(&p_sfm,method);
	////Mat gms_match = sfm_draw_gms_matches(&p_sfm, color, 1);
	////sfm_drawOptflowKps(&p_sfm);
	////sfm_drawOptFlowMap(&p_sfm,color);
	////sfm_motion_to_color(&p_sfm);
	////imshow("optical flow", p_sfm.color_flow); 
	////imshow("superpixel", p_sfm.super_pixel_image);
	////imshow("gms_match", gms_match);
	////imshow("flow map", p_sfm.c_flow_map);
	//imshow("depth", depth_map);
	//waitKey(0);
	google::InitGoogleLogging(argv[0]);

	// The variable to solve for with its initial value.
	double initial_x = 5.0;
	double x = initial_x;

	// Build the problem.
	Problem problem;

	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).
	CostFunction* cost_function =
		new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
	problem.AddResidualBlock(cost_function, NULL, &x);

	// Run the solver!
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << "\n";
	std::cout << "x : " << initial_x
		<< " -> " << x << "\n";
	return 0;
}