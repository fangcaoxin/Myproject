#include "tracks.h"
#include "camera_intrinsics.h"

void SelectKeyframesBasedOnGRICAndVariance(
	const Tracks &tracks,
	const CameraIntrinsics &intrinsics,
	vector<int> &keyframes);