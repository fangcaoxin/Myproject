#include <windows.h>
#include <GL/GL.h>
#include <GL/glut.h>
#include "SfM.h"

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
	sfm_program *p_sfm = NULL;
	p_sfm = (sfm_program*)malloc(sizeof(sfm_program));
	string img_file1 = "image//frame_0049.png";
	string img_file2 = "image//frame_0050.png";
	Mat img_1 = imread(img_file1);
	Mat img_2 = imread(img_file2);
	p_sfm->a = 7;
	sfm_add_image(p_sfm, img_1);
	sfm_add_image(p_sfm, img_2);
	sfm_optical_flow(p_sfm);
	sfm_drawOptFlowMap(p_sfm);
	imshow("optical flow", p_sfm->c_flow_map);
}