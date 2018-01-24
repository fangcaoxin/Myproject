#include <windows.h>
#include <GL/GL.h>
#include <GL/glut.h>
#include "SfM.h"
#include <iostream>

#define SET_NUM 2
#define SRC_FRAME 1

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


int main(int argc, char ** argv) {

	sfm_program p_sfm;
	int method = 1;
	Scalar color(0, 0, 255);
	//string img_file1 = "eval-data//urban//frame10.png";
	//string img_file2 = "eval-data//urban//frame11.png";
	//string folder = "eval-data//urban//frame";
	string folder = "eval-data//fuku//";
	int beg_num = 10;
	//for (int k = beg_num; k < SET_NUM + beg_num; k++) {
		//string image_name = folder + to_string(k) + ".png";
	string image_1 = folder + "0.jpg";
	string image_2 = folder + "2.jpg";
		Mat img_1 = imread(image_1);
		sfm_add_image(&p_sfm, img_1);
		Mat img_2 = imread(image_2);
		sfm_add_image(&p_sfm, img_2);
	//}
	sfm_set_base_src_image(&p_sfm, 1);
	if (METHOD == OPTICAL_FLOW_PATCH) {
		sfm_super_pixel(&p_sfm);
	}
	sfm_superpixel_image(&p_sfm, color);
	sfm_get_keyPoints(&p_sfm, SRC_FRAME);
	sfm_set_internal_matrix(&p_sfm, 1057.14, p_sfm.base_image.cols/2,p_sfm.base_image.rows/2);
	cout << "Kinv" << Mat(p_sfm.internal_matrix).inv() << endl;
	sfm_set_base_external_matrix(&p_sfm);
	sfm_get_external_matrix(&p_sfm);
	double pro_error = sfm_triangulatePoints(&p_sfm,1);
	//sfm_photoconsistency_optimazation(&p_sfm);
	Mat depth_map = sfm_drawDepths(&p_sfm);
	//Mat gms_match = sfm_draw_gms_matches(&p_sfm, color, 1);
	//sfm_drawOptflowKps(&p_sfm);
	sfm_drawOptFlowMap(&p_sfm,color);
	sfm_motion_to_color(&p_sfm);
	//imshow("optical flow", p_sfm.color_flow); 
	//imshow("superpixel", p_sfm.super_pixel_image);
	//imshow("gms_match", gms_match);
	//imshow("flow map", p_sfm.c_flow_map);
	imshow("depth", depth_map);
	waitKey(0);
	

}