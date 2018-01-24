#include <opencv2/core/core.hpp>
using namespace cv;

	void SelectKeyframesBasedOnGRICAndVariance(
		const Tracks &tracks,
		const Matx33d &intrinsics,
		vector<int> &keyframes);
