#include "tracks.h"
#include "camera_intrinsics.h"
#include  "vector.h"
namespace libmv {
	void SelectKeyframesBasedOnGRICAndVariance(
		const Tracks &tracks,
		const CameraIntrinsics &intrinsics,
		vector<int> &keyframes);
}
