#include "SfM.h"
#include "Header.h"
#include "gms_matcher.h"
#include <iostream>
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
	Point2f point_accum;
	int num;
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

//int sfm_optical_flow(sfm_program * const sfm) {
//	Mat pregray, gray;
//	Mat img_1, img_2;
//	img_1 = sfm->input_images[0];
//	img_2 = sfm->input_images[1];
//	if (!img_1.empty() && !img_2.empty()) {
//		cvtColor(img_1, pregray, CV_BGR2GRAY);
//		cvtColor(img_2, gray, CV_BGR2GRAY);
//
//		calcOpticalFlowFarneback(pregray, gray, sfm->u_flow, 0.5, 3, 15, 3, 5, 1.2, 0);
//	}
//	return OK;
//}


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

static int get_superpixel_vertex(sfm_program * const sfm, vector<Point2f>& pt1s) {
	int num = sfm->num_superpixel + 1;
	vector<super_pixel_vertex> pts(num);
	Mat label = sfm->super_pixel_label;
	for (int i = 0; i < label.rows; i++) {
		for (int j = 0; j < label.cols; j++) {
			int label_val = label.at<int>(i, j);
			pts[label_val].point_accum += Point2f(j, i);
			pts[label_val].num++;
		}
	}
	for (int k = 0; k < num; k++) {
		pts[k].point_accum /= pts[k].num;
		pt1s.push_back(pts[k].point_accum);
	}

}
int sfm_get_keyPoints(sfm_program *const sfm, int method) {
	switch (method)
	{
	case OPTICAL_FLOW:
	{
		Mat pregray, gray;
		Mat img_1, img_2;
		img_1 = sfm->input_images[0];
		img_2 = sfm->input_images[1];
		if (!img_1.empty() && !img_2.empty()) {
			cvtColor(img_1, pregray, CV_BGR2GRAY);
			cvtColor(img_2, gray, CV_BGR2GRAY);
			calcOpticalFlowFarneback(pregray, gray, sfm->u_flow, 0.5, 3, 15, 3, 5, 1.2, 0);
		}
		vector<Point2f> pt1s, pt2s;
		int num = sfm->num_superpixel;
		vector<super_pixel_vertex> pts(num);
		Mat label = sfm->super_pixel_label;
		for (int i = 0; i < label.rows; i++) {
			for (int j = 0; j < label.cols; j++) {
				int label_val = label.at<int>(i, j);
				pts[label_val].point_accum += Point2f(j, i);
				pts[label_val].num++;
			}
		}
		for (int k = 0; k < num; k++) {
			pts[k].point_accum /= pts[k].num;
			pt1s.push_back(pts[k].point_accum);
			pt2s.push_back(pts[k].point_accum + sfm->u_flow.at<Point2f>(cvRound(pts[k].point_accum.y), cvRound(pts[k].point_accum.x)));
		}
		KeyPoint::convert(pt1s, sfm->keypts1);
		KeyPoint::convert(pt2s, sfm->keypts2);
	}
	break;
	case GMS:
	{
		int num_inliers = 0;
		sfm->matches.clear();
		sfm->keypts1_good.clear();
		sfm->keypts2_good.clear();
		vector<bool> vbInliers;
		Mat d1, d2;
		vector<KeyPoint> kp1, kp2;
		vector<DMatch> matches_all, matches_gms;
		Mat img1 = sfm->input_images[0];
		Mat img2 = sfm->input_images[1];
		Ptr<ORB> orb = ORB::create(10000);
		orb->setFastThreshold(0);
		orb->detectAndCompute(img1, Mat(), kp1, d1);
		orb->detectAndCompute(img2, Mat(), kp2, d2);
		BFMatcher matcher(NORM_HAMMING);
		matcher.match(d1, d2, matches_all);
		gms_matcher gms(kp1, img1.size(), kp2, img2.size(), matches_all);
		num_inliers = gms.GetInlierMask(vbInliers, false, false);
		for (size_t i = 0; i < vbInliers.size(); ++i)
		{
			if (vbInliers[i] == true)
			{
				matches_gms.push_back(matches_all[i]);
				sfm->keypts1_good.push_back(kp1[i]);
				sfm->keypts2_good.push_back(kp2[i]);
			}
		}
		sfm->keypts1 = kp1;
		sfm->keypts2 = kp2;
		sfm->matches = matches_gms;
	}
	break;
	default:
		break;
	}
	return OK;
}

int sfm_drawOptflowKps(sfm_program *const sfm) {
	vector<Point2f> centers;
	KeyPoint::convert(sfm->keypts1, centers);
	for (int i = 0; i < sfm->keypts1.size(); ++i) {
		circle(sfm->super_pixel_image, Point(cvRound(centers[i].x),cvRound(centers[i].y)), 2, Scalar(0, 255, 0));
	}
	return OK;
}

int sfm_set_internal_matrix(sfm_program *const sfm, double f, double cx, double cy) {
	double a[] = {
		f,0,cx,
		0,f,cy,
		0,0,1
	};
	sfm->internal_matrix = Mat(3,3,CV_64F,a);
	
	return OK;
}

int sfm_get_external_matrix(sfm_program *const sfm) {
	vector<Point2f> keypts1, keypts2;
	vector<uchar> status;
	KeyPoint::convert(sfm->keypts1, keypts1);
	KeyPoint::convert(sfm->keypts2, keypts2);
	if (keypts1.size() != keypts2.size()) {
		KeyPoint::convert(sfm->keypts1_good, keypts1);
		KeyPoint::convert(sfm->keypts2_good, keypts2);
	}
	Matx33d K(3, 3, CV_64F);
	K= sfm->internal_matrix;
	cout << "internal" << K << endl;
#define OPENCV
#ifdef OPENCV
	double f = K(0,0);
	Point2d pp(K(0,2), K(1,2));
	Mat E = findEssentialMat(keypts1, keypts2,f, pp, RANSAC, 0.999, 1.0, status);
#else
	Mat F = findFundamentalMat(keypts1, keypts2, FM_RANSAC, 3.0, 0.99, status);
	cout << "fundamental " <<F << endl;
	Mat_<double> E = K.t()*F* K;
#endif //OPENCV
	cout << "Essential " << E << endl;
	if (fabsf(determinant(E)) > 1e-05) {
		cout << "det(E) != 0 : " << determinant(E) << "\n";
		return false;
	}
	for (int i = 0; i < status.size(); i++) {
		if (status[i]) {
			sfm->keypts1_good[i]=sfm->keypts1[i];
			sfm->keypts2_good[i]=sfm->keypts2[i];
		}
	}

	Mat_<double> R(3, 3);
	Mat_<double> t(1, 3);
	SVD svd(E);
	Matx33d W(0, -1, 0,
		       1, 0, 0,
		       0, 0, 1);
	R = svd.u*Mat(W)*svd.vt;
	t = svd.u.col(2)*5;
	sfm->external_martix = Matx34d(R(0, 0), R(0, 1), R(0, 2), t(0),
		R(1, 0), R(1, 1), R(1, 2), t(1),
		R(2, 0), R(2, 1), R(2, 2), t(2));
	cout << "external " << Mat(sfm->external_martix) << endl;
	return OK;
}

int sfm_triangulatePoints(sfm_program *const sfm) {

	sfm->pointcloud.clear();
	sfm->correspImgPt.clear();
	sfm->depths.clear();
	Matx34d P = Matx34d(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	Matx34d P1 = sfm->external_martix;
	Matx44d P1_(P1(0, 0), P1(0, 1), P1(0, 2), P1(0, 3),
		P1(1, 0), P1(1, 1), P1(1, 2), P1(1, 3),
		P1(2, 0), P1(2, 1), P1(2, 2), P1(2, 3),
		0, 0, 0, 1);
	Matx44d P1inv(P1_.inv());
	cout << "Triangluating Now...";
	vector<double> reproj_error;
	int pts_size = sfm->keypts1_good.size();
	Mat_<double> KP1 = Mat(sfm->internal_matrix)*Mat(P1);
#pragma omp parallel for num_threads(1)
	for (int i = 0; i < pts_size; i++) {
		Point2f kp = sfm->keypts1_good[i].pt;
		Point3d u(kp.x, kp.y, 1.0);
		Mat_<double> um = Mat(sfm->internal_matrix).inv()*Mat_<double>(u);
		u.x = um(0); u.y = um(1); u.z = um(2);

		Point2f kp1 = sfm->keypts2_good[i].pt;
		Point3d u1(kp1.x, kp1.y, 1.0);
		Mat_<double> um1 = Mat(sfm->internal_matrix).inv()*Mat_<double>(u1);
		u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

		Mat_<double> X = IterativeLinearLSTriangulation(u, P, u1, P1);
		Mat_<double> xPt_img = KP1 * X;
		Point2f xPt_img_(xPt_img(0) / xPt_img(2), xPt_img(1) / xPt_img(2));
#pragma omp critical
		{
			double reprj_err = norm(xPt_img_ - kp1);
			reproj_error.push_back(reprj_err);

			CloudPoint cp;
			cp.pt = Point3d(X(0), X(1), X(2));
			cp.reprojection_error = reprj_err;

			sfm->pointcloud.push_back(cp);
			sfm->correspImgPt.push_back(sfm->keypts1_good[i]);
			sfm->depths.push_back(X(2));
		
		}
	}
	Scalar mse = mean(reproj_error);
	return OK;
}

Mat sfm_drawDepths(sfm_program *const sfm, int method) {
	double minVal, maxVal;
	vector<double> depths = sfm->depths;
	minMaxLoc(depths, &minVal, &maxVal);
	int width = sfm->input_images[0].cols;
	int height = sfm->input_images[0].rows;
	for (int i = 0; i < sfm->depths.size(); i++) {
		double _d = MAX(MIN((depths[i] - minVal) / (maxVal - minVal), 1.0), 0.0);
		Scalar color(255 * (1.0 - (_d)), 255, 255);

	}
	switch (method)
	{
	case OPTICAL_FLOW: {
		Mat tmp(height, width, CV_8UC1, Scalar(0, 0, 0));
		
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				int index = sfm->super_pixel_label.at<int>(i, j);
				double _d = MAX(MIN((depths[index] - minVal) / (maxVal - minVal), 1.0), 0.0);
				unsigned char color = (255 * (1.0 - _d));
				tmp.at<uchar>(i, j) = color;
			}
		}
		return tmp;
	}
	break;
	case GMS: {
		Mat tmp(height, width, CV_8UC3, Scalar(0, 0, 0));
		vector<Point2f> keypts;
		KeyPoint::convert(sfm->keypts1_good, keypts);
		for (int i = 0; i < keypts.size(); i++) {
			double _d = MAX(MIN((depths[i] - minVal) / (maxVal - minVal), 1.0), 0.0);
			Vec3b color(255 * (1.0 - (_d)), 255, 255);
			tmp.at<Vec3b>(keypts[i]) = color;
		}
		cvtColor(tmp, tmp, CV_HSV2BGR);
		return tmp;
	}
	break;
	default:
		break;
	}
	
}

Mat sfm_draw_gms_matches(sfm_program *const sfm, Scalar color, int type) {
	Mat src1 = sfm->input_images[0];
	Mat src2 = sfm->input_images[1];
	vector<DMatch> inlier = sfm->matches;
	vector<KeyPoint> kpt1 = sfm->keypts1;
	vector<KeyPoint> kpt2 = sfm->keypts2;
	const int height = max(src1.rows, src2.rows);
	const int width = src1.cols + src2.cols;
	Mat output(height, width, CV_8UC3, Scalar(0, 0, 0));
	src1.copyTo(output(Rect(0, 0, src1.cols, src1.rows)));
	src2.copyTo(output(Rect(src1.cols, 0, src2.cols, src2.rows)));

	if (type == 1)
	{
		for (size_t i = 0; i < inlier.size(); i = i + 10)
		{
			Point2f left = kpt1[inlier[i].queryIdx].pt;
			Point2f right = (kpt2[inlier[i].trainIdx].pt + Point2f((float)src1.cols, 0.f));
			line(output, left, right, color);
		}
	}
	else if (type == 2)
	{
		for (size_t i = 0; i < inlier.size(); i++)
		{
			Point2f left = kpt1[inlier[i].queryIdx].pt;
			Point2f right = (kpt2[inlier[i].trainIdx].pt + Point2f((float)src1.cols, 0.f));
			line(output, left, right, color);
		}

		for (size_t i = 0; i < inlier.size(); i++)
		{
			Point2f left = kpt1[inlier[i].queryIdx].pt;
			Point2f right = (kpt2[inlier[i].trainIdx].pt + Point2f((float)src1.cols, 0.f));
			circle(output, left, 1, Scalar(0, 255, 255), 2);
			circle(output, right, 1, Scalar(0, 255, 0), 2);
		}
	}

	return output;
}