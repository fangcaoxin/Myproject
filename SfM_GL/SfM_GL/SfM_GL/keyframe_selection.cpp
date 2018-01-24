#include "keyframe_selection.h"
#include <Eigen/Eigenvalues>
#include "SfM.h"
using namespace std;





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

	double GRIC(const vector<float> &e, int d, int k, int r) {
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

	Mat PseudoInverseWithClampedEigenvalues(const Mat &matrix,
		int num_eigenvalues_to_clamp) {
		Eigen::EigenSolver<Mat> eigen_solver(matrix);
		Mat D = eigen_solver.pseudoEigenvalueMatrix();
		Mat V = eigen_solver.pseudoEigenvectors();

		// Clamp too-small singular values to zero to prevent numeric blowup.
		double epsilon = std::numeric_limits<double>::epsilon();
		for (int i = 0; i < D.cols(); ++i) {
			if (D(i, i) > epsilon) {
				D(i, i) = 1.0 / D(i, i);
			}
			else {
				D(i, i) = 0.0;
			}
		}

		// Apply the clamp.
		for (int i = D.cols() - num_eigenvalues_to_clamp; i < D.cols(); ++i) {
			D(i, i) = 0.0;
		}
		return V * D * V.inverse();
	}

	void FilterZeroWeightMarkersFromTracks(const Tracks &tracks,
		Tracks *filtered_tracks) {
		vector<Marker> all_markers = tracks.AllMarkers();

		for (int i = 0; i < all_markers.size(); ++i) {
			Marker &marker = all_markers[i];
			if (marker.weight != 0.0) {
				filtered_tracks->Insert(marker.image,
					marker.track,
					marker.x,
					marker.y,
					marker.weight);
			}
		}
	}

	void SelectKeyframesBasedOnGRICAndVariance(const sfm_program *p_sfm,
		const Matx33d &intrinsics,
		vector<int> &keyframes)
	{
		Tracks filtered_tracks;
		FilterZeroWeightMarkersFromTracks(_tracks, &filtered_tracks);
		int max_image = filtered_tracks.MaxImage();
		int next_keyframe = 1;
		int number_keyframes = 0;
		// Limit correspondence ratio from both sides.
		// On the one hand if number of correspondent features is too low,
		// triangulation will suffer.
		// On the other hand high correspondence likely means short baseline.
		// which also will affect om accuracy
		const double Tmin = 0.8;
		const double Tmax = 1.0;

		Matx33d N = intrinsics;
		Matx33d N_inverse = N.inv();

		double Sc_best = std::numeric_limits<double>::max();
		double success_intersects_factor_best = 0.0f;
		while (next_keyframe != -1) {
			int currect_keyframe = next_keyframe;
			double Sc_best_candidate = std::numeric_limits<double>::max();
			number_keyframes++;
			next_keyframe = -1;

			for (int candidate_image = currect_keyframe + 1;
				candidate_image <= max_image;
				candidate_image++) {
				vector<Marker> all_markers = filtered_tracks.MarkersInBothImages(currect_keyframe, candidate_image);
				vector<Marker> tracked_markers =
					filtered_tracks.MarkersForTracksInBothImages(currect_keyframe, candidate_image);
				vector<Point2f> x1, x2;
				CoordinatesForMarkersInImage(tracked_markers, currect_keyframe, &x1);
				CoordinatesForMarkersInImage(tracked_markers, candidate_image, &x2);
				if (x1.cols() < 8 || x2.cols() < 8)
					continue;
				int Tc = tracked_markers.size();
				int Tf = all_markers.size();
				double Rc = static_cast<double>(Tc) / Tf;
				if (Rc < Tmin || Rc > Tmax)
					continue;
				Mat3 H, F;
			
				Mat H = findHomography(x1, x2, Mat(), 0, 3.0);
				// Convert homography to original pixel space.
				H = N_inverse * H * Mat(N);
				Mat F = findFundamentalMat(x1, x2, CV_FM_RANSAC, 3.0, 0.99, Mat());
				F = N_inverse * F * Mat(N);
				Vec H_e, F_e;
				H_e.resize(x1.cols());
				F_e.resize(x1.cols());
				for (int i = 0; i < x1.size(); i++) {
					Point2f current_x1, current_x2;

					intrinsics.NormalizedToImageSpace(x1(0, i), x1(1, i),
						&current_x1(0), &current_x1(1));

					intrinsics.NormalizedToImageSpace(x2(0, i), x2(1, i),
						&current_x2(0), &current_x2(1));

					H_e(i) = SymmetricGeometricDistance(H, current_x1, current_x2);
					F_e(i) = SymmetricEpipolarDistance(F, current_x1, current_x2);
					// Degeneracy constraint
					double GRIC_H = GRIC(H_e, 2, 8, 4);
					double GRIC_F = GRIC(F_e, 3, 7, 4);
					if (GRIC_H <= GRIC_F)
						continue;
					EuclideanReconstruction reconstruction;

					// The F matrix should be an E matrix, but squash it just to be sure

					// Reconstruction should happen using normalized fundamental matrix
					Mat3 F_normal = N * F * N_inverse;

					Mat3 E;
					FundamentalToEssential(F_normal, &E);

					// Recover motion between the two images. Since this function assumes a
					// calibrated camera, use the identity for K
					Mat3 R;
					Vec3 t;
					Mat3 K = Mat3::Identity();
				}

			}
		}
	}


