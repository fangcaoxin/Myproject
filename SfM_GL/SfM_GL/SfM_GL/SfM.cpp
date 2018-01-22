#include "SfM.h"
#include "Header.h"
#include "gms_matcher.h"
#include "GCMex/GCoptimization.h"
#include <iostream>
#include <math.h>
#define OK 0;

enum {
	SLIC = 100,
	SLICO = 101,
	MSLIC = 102,
};





static int set_dsp_table(double *dsp_table) {
	dsp_table[0] = DSP_MIN;
	for (int i = 1; i < DSP_LVL; i++) {
		dsp_table[i] = dsp_table[i - 1] + (double)(DSP_MAX - DSP_MIN) / DSP_LVL;
	}
	return OK;
}


struct super_pixel_vertex {
	Point2f point_accum;
	int num;
};

bool MotionFromEssentialAndCorrespondence(const Matx33d &E,
	const Matx33d &K1,
	const Point2f x1,
	const Point2f x2,
	Matx33d *R,
	Matx31d *t);

int sfm_add_image(sfm_program * const sfm, Mat& p_input_image) {
	sfm->input_images.push_back(p_input_image);
	return OK;
}

int sfm_super_pixel(sfm_program * const sfm) {
	int min_element_size = 20;
	int region_size = 20;
	int num_iterations = 3;
	int ruler = 10;
	Mat img_1 = sfm->base_image;
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
int sfm_get_keyPoints(sfm_program *const sfm, int frame_num) {
	switch (METHOD)
	{
	case OPTICAL_FLOW:
	{
		Mat pregray, gray;
		Mat img_1, img_2;
		img_1 = sfm->base_image;
		img_2 = sfm->src_image;
		if (!img_1.empty() && !img_2.empty()) {
			cvtColor(img_1, pregray, CV_BGR2GRAY);
			cvtColor(img_2, gray, CV_BGR2GRAY);
			calcOpticalFlowFarneback(pregray, gray, sfm->u_flow, 0.5, 3, 15, 3, 5, 1.2, 0);
		}
		vector<Point2f> pt1s, pt2s;
		int num = sfm->num_superpixel;
		vector<super_pixel_vertex> pts(num);
		vector<vector<Point>> label_points(num);
		Mat label = sfm->super_pixel_label;
		for (int i = 0; i < label.rows; i++) {
			for (int j = 0; j < label.cols; j++) {
				int label_val = label.at<int>(i, j);
				label_points[label_val].push_back(Point(j, i));
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
		sfm->label_points = label_points;
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
		Mat img1 = sfm->base_image;
		Mat img2 = sfm->input_images[frame_num];
		Ptr<ORB> orb = ORB::create(10000);
		orb->setFastThreshold(0);
		orb->detectAndCompute(img1, Mat(), kp1, d1);
		orb->detectAndCompute(img2, Mat(), kp2, d2);
		BFMatcher matcher(NORM_HAMMING);
		matcher.match(d1, d2, matches_all);
		gms_matcher gms(kp1, img1.size(), kp2, img2.size(), matches_all);
		num_inliers = gms.GetInlierMask(vbInliers, false, false);
		for (size_t i = 0; i < vbInliers.size(); i++)
		{
			if (vbInliers[i] == true)
			{
				matches_gms.push_back(matches_all[i]);
				sfm->keypts1_good.push_back(kp1[matches_all[i].queryIdx]);
				sfm->keypts2_good.push_back(kp2[matches_all[i].trainIdx]);
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

int sfm_set_base_external_matrix(sfm_program *const sfm)
{
	Matx34d P = Matx34d(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	sfm->external_martix.push_back(P);
	return OK;
}
/*determined by the keypoints of the frame num */
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
	std::cout << "internal" << K << endl;
#define OPENCV
#ifdef OPENCV
	double f = K(0,0);
	Point2d pp(K(0,2), K(1,2));
	Mat E = findEssentialMat(keypts1, keypts2,f, pp, RANSAC, 0.999, 1.0, status);
	sfm->status = status;
#else
	Mat F = findFundamentalMat(keypts1, keypts2, FM_RANSAC, 3.0, 0.99, status);
	std::cout << "fundamental " <<F << endl;
	Mat KM(K);
	Mat_<double> E = KM.t()*F* KM;
#endif //OPENCV
	std::cout << "Essential " << E << endl;
	if (fabsf(determinant(E)) > 1e-05) {
		std::cout << "det(E) != 0 : " << determinant(E) << "\n";
		return false;
	}
	sfm->keypts1_good.clear();
	sfm->keypts2_good.clear();
	for (int i = 0; i < status.size(); i++) {
		if (status[i]) {
			sfm->keypts1_good.push_back(sfm->keypts1[i]);
			sfm->keypts2_good.push_back(sfm->keypts2[i]);
		}
	}
	Point2f x1(sfm->keypts1[0].pt);
	Point2f x2(sfm->keypts2[0].pt);
	Matx33d R;
	Matx31d t;
	Matx34d P;
	if (MotionFromEssentialAndCorrespondence(E, K, x1, x2, &R, &t)) {

		P= Matx34d(R(0, 0), R(0, 1), R(0, 2), t(0, 0),
			R(1, 0), R(1, 1), R(1, 2), t(1, 0),
			R(2, 0), R(2, 1), R(2, 2), t(2, 0));
		sfm->external_martix.push_back(P);
		cout << "R " << R << "t " << t << endl;
	}
	else {
		cout << "Failed to compute R and t from E and K" << endl;
	}
	
	std::cout << "external " << Mat(sfm->external_martix) << endl;
	return OK;
}




double sfm_triangulatePoints(sfm_program *const sfm , int frame_num) {

	sfm->pointcloud.clear();
	sfm->correspImgPt.clear();
	sfm->depths.clear();
//#define OPENCV_TRIANGULATION
#ifdef OPENCV_TRIANGULATION
	Matx34d P = Matx34d(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	Matx34d P1 = sfm->external_martix;
	vector<Point2f> pts;
	vector<Point2f> pts1;
	
	KeyPoint::convert(sfm->keypts1, pts);
	KeyPoint::convert(sfm->keypts2, pts1);
	Mat pts_rec_homo_1,pts_rec_homo_2;
	Mat pts_rec_1, pts_rec_2,pts_rec_reshape_1;
	triangulatePoints(P, P1, pts, pts1, pts_rec_homo_1);
	triangulatePoints(P1, P, pts1, pts, pts_rec_homo_2);
	Mat th = pts_rec_homo_1.reshape(4,1);
	Mat th2 = pts_rec_homo_2.reshape(4, 1);
	convertPointsFromHomogeneous(th, pts_rec_1);
	convertPointsFromHomogeneous(th2, pts_rec_2);
	pts_rec_reshape_1 = pts_rec_1.reshape(1, 3);

	sfm->Pt1 = pts_rec_1;
	sfm->Pt2 = pts_rec_2;
	/*for (int i = 0; i < 20; i++) {
		cout << pts_rec_reshape.col(i) << endl;
	}*/
	sfm->depths = pts_rec_reshape_1.row(2);
	
#else

	Matx34d P = sfm->internal_matrix*sfm->external_martix[0];
	Matx34d P1 = sfm->internal_matrix*sfm->external_martix[frame_num];
	Matx44d P1_(P1(0, 0), P1(0, 1), P1(0, 2), P1(0, 3),
		P1(1, 0), P1(1, 1), P1(1, 2), P1(1, 3),
		P1(2, 0), P1(2, 1), P1(2, 2), P1(2, 3),
		0, 0, 0, 1);
	Matx44d P1inv(P1_.inv());
	std::cout << "Triangluating Now...";
	vector<double> reproj_error;
	int pts_size = sfm->keypts1.size();
	Mat_<double> KP1 = Mat(sfm->internal_matrix)*Mat(P1);
#pragma omp parallel for num_threads(1)
	for (int i = 0; i < pts_size; i++) {
		Point2f kp = sfm->keypts1[i].pt;
		Point3d u(kp.x, kp.y, 1.0);
		Mat_<double> um = Mat(sfm->internal_matrix).inv()*Mat_<double>(u);
		u.x = um(0); u.y = um(1); u.z = um(2);

		Point2f kp1 = sfm->keypts2[i].pt;
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
			sfm->correspImgPt.push_back(sfm->keypts1[i]);
			sfm->depths.push_back(X(2));
		
		}
	}
	Scalar mse = mean(reproj_error);
#endif //OPENCV
	return mse(0);
}

static void MotionFromEssential(const Matx33d E, vector<Matx33d> &Rs, vector<Matx31d>& ts) {
	SVD svd(E);
	if (determinant(svd.u) < 0) {
		svd.u.col(2) *= -1;
	}

	if (determinant(svd.vt) < 0) {
		svd.vt.row(2) *= -1;
	}

	Matx33d W(0, -1, 0,
		1, 0, 0,
		0, 0, 1);
	Mat_<double> R1(3, 3);
	Mat_<double> R2(3, 3);
	Mat_<double> t1(3, 1);
	Mat_<double> t2(3, 1);
	R1 = svd.u*Mat(W)*svd.vt;
	R2 = svd.u*Mat(W).t()*svd.vt;
	t1 = svd.u.col(2);
	t2 = -svd.u.col(2);
	Rs.push_back(Matx33d(R1));
	Rs.push_back(Matx33d(R1));
	Rs.push_back(Matx33d(R2));
	Rs.push_back(Matx33d(R2));

	ts.push_back(Matx31d(t1));
	ts.push_back(Matx31d(t2));
	ts.push_back(Matx31d(t1));
	ts.push_back(Matx31d(t2));

}
static Matx34d P_from_KRt(Matx33d K, Matx33d R, Matx31d t) {
	Matx34d P(R(0, 0), R(0, 1), R(0, 2), t(0, 0),
		R(1, 0), R(1, 1), R(1, 2), t(1, 0),
		R(2, 0), R(2, 1), R(2, 2), t(2, 0));
	P = K * P;
	return P;
}

static int MotionFromEssentialChooseSolution(const vector<Matx33d> &Rs,
	const vector<Matx31d> &ts,
	const Matx33d K,
	const Point2f x1,
	const Point2f x2
) 
{
	Matx33d R1(1, 0, 0,
		0, 1, 0,
		0, 0, 1);
	Matx31d t1(0, 0,0);
	Matx34d P1(1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	Matx34d P2;
	Point3d u(x1.x, x1.y, 1.0);
	Mat_<double> um = Mat(K).inv()*Mat_<double>(u);
	u.x = um(0); u.y = um(1); u.z = um(2);

	Point3d u1(x2.x, x2.y, 1.0);
	Mat_<double> um1 = Mat(K).inv()*Mat_<double>(u1);
	u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);
	for (int i = 0; i < 4; i++) {
		P2 = P_from_KRt(K, Rs[i], ts[i]);
		Mat_ <double> X = IterativeLinearLSTriangulation(u, P1, u1, P2);
		Point3d Xe(X(0), X(1), X(2));
		Mat_<double> d_r = Mat(R1)*Mat_<double>(Xe);
		Mat_<double> d1_r = Mat(Rs[i])*Mat_<double>(Xe);
		double d1 = d_r(2) + t1(2, 0);
		double d2 = d1_r(2) + ts[i](2, 0);
		if (d1 > 0 && d2 > 0) {
			return i;
		}
	}
	return -1;
}

bool MotionFromEssentialAndCorrespondence(const Matx33d &E,
	const Matx33d &K1,
	const Point2f x1,
	const Point2f x2,
	Matx33d *R,
	Matx31d *t) 
{
	std::vector<Matx33d> Rs;
	std::vector<Matx31d> ts;
	MotionFromEssential(E, Rs, ts);
	int solution = MotionFromEssentialChooseSolution(Rs, ts, K1, x1, x2);
	if (solution >= 0) {
		*R = Rs[solution];
		*t = ts[solution];
		return true;
	}
	else {
		return false;
	}
}

Mat sfm_drawDepths(sfm_program *const sfm) 
{
	double minVal, maxVal;
	vector<double> depths = sfm->depths;
	minMaxLoc(depths, &minVal, &maxVal);
	int width = sfm->base_image.cols;
	int height = sfm->base_image.rows;
	for (int i = 0; i < sfm->depths.size(); i++) {
		double _d = MAX(MIN((depths[i] - minVal) / (maxVal - minVal), 1.0), 0.0);
		Scalar color(255 * (1.0 - (_d)), 255, 255);

	}
	switch (METHOD)
	{
	case OPTICAL_FLOW: {
		Mat tmp(height, width, CV_8UC1, Scalar(0, 0, 0));
		
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				int index = sfm->super_pixel_label.at<int>(i, j);
				//if (!sfm->status[index]) {
				//	continue;
				//}
				//else
				{
					double _d = MAX(MIN((depths[index] - minVal) / (maxVal - minVal), 1.0), 0.0);
					unsigned char color = (255 * (1.0 - _d));
					tmp.at<uchar>(i, j) = color;
				}
				
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

Mat sfm_draw_gms_matches(sfm_program *const sfm, Scalar color, int type)
{
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
/* ith camera pixel -> 3D points at different depth */
static Point3d sfm_reproj4Bundler(Point2f point_2d, Matx33d K, Matx34d external_martix, double d)
{
	double cx = K(0, 2);
	double cy = K(1, 2);
	double f = K(0, 0);
	double z =  1.0/d;
	Point3d u(point_2d.x, point_2d.y, 1.0);
	Mat_<double> um = Mat(K).inv()*Mat_<double>(u);
	Point3d uw(um(0)*z, um(1)*z, z);
	//cout << uw;
	//Mat_<double> um = Mat(K).inv()*Mat_<double>(u);
	Matx33d R(external_martix(0, 0), external_martix(0, 1), external_martix(0, 2),
		external_martix(1, 0), external_martix(1, 1), external_martix(1, 2),
		external_martix(2, 0), external_martix(2, 1), external_martix(2, 2));
	Mat_<double> t(external_martix(0, 3), external_martix(1, 3), external_martix(2, 3));
	Mat_<double> u2 = Mat(R).t()*(Mat_<double>(uw) - t);

	return Point3d(u2(0), u2(1), u2(2));
}

static Point2f sfm_proj4Bundler(Point3d X, Matx33d K, Matx34d external_martix) {
	Matx33d R(external_martix(0, 0), external_martix(0, 1), external_martix(0, 2),
		external_martix(1, 0), external_martix(1, 1), external_martix(1, 2),
		external_martix(2, 0), external_martix(2, 1), external_martix(2, 2));
	Matx31d t(external_martix(0, 3), external_martix(1, 3), external_martix(2, 3));
	
	//cout << "R"<<R<< "t " << t << endl;
	Mat_<double> u = Mat(K)*(Mat(R)*Mat_<double>(X)+Mat(t));
	//cout << " R*X +t " << Mat(R)*Mat_<double>(X)+Mat(t) << endl;

	return Point2f(u(0)/u(2), u(1)/u(2));
}

int sfm_set_base_src_image(sfm_program *const sfm, int frame_num) {
	
	CV_Assert(sfm->input_images[0].cols > 0, "NO input images");
	
	sfm->input_images[0].copyTo(sfm->base_image);
	sfm->input_images[frame_num].copyTo(sfm->src_image);
	return OK;
}

int sfm_photoconsistency_optimazation(sfm_program *const sfm)
{
	Mat img_1,img_2, img_1n,img_2n;
	cvtColor(sfm->base_image,img_1,CV_BGR2GRAY);
	cvtColor(sfm->src_image, img_2, CV_BGR2GRAY);
	img_1.convertTo(img_1, CV_32FC1);
	img_2.convertTo(img_2, CV_32FC1);
	normalize(img_1, img_1n, 0, 1, NORM_MINMAX);
	normalize(img_2, img_2n, 0, 1, NORM_MINMAX);
	double depth_table[DSP_LVL];
	set_dsp_table(depth_table);
	if (METHOD == OPTICAL_FLOW) {
		vector<Point2f> base_points;
		vector<Point2f> src_points;
		KeyPoint::convert(sfm->keypts1, base_points);
		KeyPoint::convert(sfm->keypts2, src_points);
		Matx33d K = sfm->internal_matrix;
		Matx34d External_base = sfm->external_martix[0];
		Matx34d External_src = sfm->external_martix[1];
		CV_Assert(base_points.size() == src_points.size(), "Not equal keypoints in base image and src image");
		double p_error = 0.;
		vector<double> p_err_at_detph(DSP_LVL);
		vector<vector<int>> neighbours(base_points.size());
		//int *result = new int[base_points.size()];
		try {
			GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph((int)(base_points.size()),DSP_LVL);
			
			for (int k = 0; k < base_points.size(); k++) {
				for (int t = 0; t < base_points.size(); t++) {
					if (k == t) continue;
					if (norm(base_points[k] - base_points[t]) < 25) {
						gc->setNeighbors(k, t, k*t);
						neighbours[k].push_back(t);
						//cout << "neighbours" << k <<" "<< t << endl;
					}
				}
			}
			float min = 10000;
			vector<int> what_label;
			   for (int i = 0; i < base_points.size(); i++) {
				   int index;
				   for (int j = 0; j < DSP_LVL; j++) {
				
					Point3d X;
					Point2f x_proj;
					float p_err = 0.0;
				
					//for (int k = 0; k < sfm->label_points[i].size(); k++) {
						X = sfm_reproj4Bundler(base_points[i], K, External_base, depth_table[j]);
						x_proj = sfm_proj4Bundler(X, K, External_src);
						//Point2f src_points;
						//src_points = Point2f(sfm->label_points[i][k]) + sfm->u_flow.at<Point2f>(sfm->label_points[i][k]);
						p_err = norm(src_points[i] - x_proj);
					//}
					
					//cout <<x_proj << endl;
					Point x_proj_round(x_proj.x, x_proj.y);
					x_proj_round.x = x_proj_round.x < 0 ? 0 : x_proj_round.x;
					x_proj_round.y = x_proj_round.y < 0 ? 0 : x_proj_round.y;
					x_proj_round.x = x_proj_round.x > img_1.cols - 1 ? img_1.cols - 1 : x_proj_round.x;
					x_proj_round.y = x_proj_round.y > img_1.rows - 1 ? img_1.rows - 1 : x_proj_round.y;
					//cout << Point(base_points[i]) << " " << x_proj_round << endl;
					//float p_err = norm(src_points[i] - x_proj);
					float g_err = abs(img_1n.at<float>(Point(base_points[i])) - img_2n.at<float>(x_proj_round));
					float t_err = 0.05*p_err + g_err;
					
					//cout << "t_err" <<" "<<j<<" "<< p_err << endl;
					gc->setDataCost(i, j, t_err);
					if (t_err < min) {
						min = t_err;
						index = j;
					}
				}
				   what_label.push_back(index);
			}
			for (int l1 = 0; l1 < DSP_LVL; l1++) {
				for (int l2 = 0; l2 < DSP_LVL; l2++) {
					int cost = (l1 - l2)*(l1 - l2) <= 100 ? (l1 - l2)*(l1 - l2) : 100;
					gc->setSmoothCost(l1, l2, cost);
				}
			}

			

			cout << "Before optimization energy is " << gc->compute_energy() << endl;
			gc->expansion(5);
			cout << " After optimization energy is " << gc->compute_energy() << endl;
			sfm->depths.clear();
			for (int k = 0; k < base_points.size(); k++) {
				//int result = gc->whatLabel(k);
				int result = what_label[k];
				cout << "result" <<result<< endl;
				sfm->depths.push_back(1.0/depth_table[result]);
			}
			delete gc;
		}
		catch (GCException e) {
			e.Report();
		}
		
	}
	else if (METHOD == GMS) {
		int width = img_1.cols;
		int height = img_1.rows;
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {

			}
		}
	}
	return OK;
}