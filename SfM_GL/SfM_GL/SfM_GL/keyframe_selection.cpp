#include "keyframe_selection.h"

Matx33d IntrinsicsNormalizationMatrix(const CameraIntrinsics &intrinsics) {
	Matx33d T(1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, 1.0);
	Matx33d S(1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, 1.0);

	T(0, 2) = -intrinsics.principal_point_x();
	T(1, 2) = -intrinsics.principal_point_y();

	S(0, 0) /= intrinsics.focal_length_x();
	S(1, 1) /= intrinsics.focal_length_y();

	return S * T;
}

// P.H.S. Torr
// Geometric Motion Segmentation and Model Selection
//
// http://reference.kfupm.edu.sa/content/g/e/geometric_motion_segmentation_and_model__126445.pdf
//
// d is the number of dimensions modeled
//     (d = 3 for a fundamental matrix or 2 for a homography)
// k is the number of degrees of freedom in the model
//     (k = 7 for a fundamental matrix or 8 for a homography)
// r is the dimension of the data
//     (r = 4 for 2D correspondences between two frames)

double GRIC(const vector<double>& e, int d, int k, int r) {
	int n = e.size();
	double lambda1 = log(static_cast<double>(r));
	double lambda2 = log(static_cast<double>(r*n));
	double lambda3 = 2.0;
	double sigma2 = 0.01;
	double gric = 0.0;
	for (int i = 0; i < n; i++) {
		gric += std::min(e[i] * e[i] / sigma2, lambda3 * (r - d));
	}
	gric += lambda1 * d * n;
	gric += lambda2 * k;
	return gric;
}
