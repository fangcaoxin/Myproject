#include <opencv2/core/core.hpp>

using namespace cv;

class CameraIntrinsics {
public:
	CameraIntrinsics();
	CameraIntrinsics(const CameraIntrinsics &from);
	virtual ~CameraIntrinsics() {}

	//virtual DistortionModelType GetDistortionModelType() const = 0;

	int image_width()  const { return image_width_; }
	int image_height() const { return image_height_; }

	const Matx33d &K() const { return K_; }

	double focal_length()      const { return K_(0, 0); }
	double focal_length_x()    const { return K_(0, 0); }
	double focal_length_y()    const { return K_(1, 1); }

	double principal_point_x() const { return K_(0, 2); }
	double principal_point_y() const { return K_(1, 2); }

	virtual int num_distortion_parameters() const = 0;
	virtual double *distortion_parameters() = 0;
	virtual const double *distortion_parameters() const = 0;

	// Set the image size in pixels.
	// Image is the size of image camera intrinsics were calibrated with.
	void SetImageSize(int width, int height);

	// Set the entire calibration matrix at once.
	void SetK(const Matx33d new_k);

	// Set both x and y focal length in pixels.
	void SetFocalLength(double focal_x, double focal_y);

	// Set principal point in pixels.
	void SetPrincipalPoint(double cx, double cy);

	// Set number of threads used for threaded buffer distortion/undistortion.
	void SetThreads(int threads);

	// Convert image space coordinates to normalized.
	void ImageSpaceToNormalized(double image_x,
		double image_y,
		double *normalized_x,
		double *normalized_y) const;

	// Convert normalized coordinates to image space.
	void NormalizedToImageSpace(double normalized_x,
		double normalized_y,
		double *image_x,
		double *image_y) const;

	// Apply camera intrinsics to the normalized point to get image coordinates.
	//
	// This applies the lens distortion to a point which is in normalized
	// camera coordinates (i.e. the principal point is at (0, 0)) to get image
	// coordinates in pixels.
	virtual void ApplyIntrinsics(double normalized_x,
		double normalized_y,
		double *image_x,
		double *image_y) const = 0;

	// Invert camera intrinsics on the image point to get normalized coordinates.
	//
	// This reverses the effect of lens distortion on a point which is in image
	// coordinates to get normalized camera coordinates.
	virtual void InvertIntrinsics(double image_x,
		double image_y,
		double *normalized_x,
		double *normalized_y) const = 0;

	// Distort an image using the current camera instrinsics
	//
	// The distorted image is computed in output_buffer using samples from
	// input_buffer. Both buffers should be width x height x channels sized.
	//
	// Overscan is a percentage value of how much overcan the image have.
	// For example overscal value of 0.2 means 20% of overscan in the
	// buffers.
	//
	// Overscan is usually used in cases when one need to distort an image
	// and don't have a barrel in the distorted buffer. For example, when
	// one need to render properly distorted FullHD frame without barrel
	// visible. For such cases renderers usually renders bigger images and
	// crops them after the distortion.
	//
	// This method is templated to be able to distort byte and float buffers
	// without having separate methods for this two types. So basically only
	//
	// But in fact PixelType might be any type for which multiplication by
	// a scalar and addition are implemented. For example PixelType might be
	// Vec3 as well.
	template<typename PixelType>
	void DistortBuffer(const PixelType *input_buffer,
		int width,
		int height,
		double overscan,
		int channels,
		PixelType *output_buffer);

	// Undistort an image using the current camera instrinsics
	//
	// The undistorted image is computed in output_buffer using samples from
	// input_buffer. Both buffers should be width x height x channels sized.
	//
	// Overscan is a percentage value of how much overcan the image have.
	// For example overscal value of 0.2 means 20% of overscan in the
	// buffers.
	//
	// Overscan is usually used in cases when one need to distort an image
	// and don't have a barrel in the distorted buffer. For example, when
	// one need to render properly distorted FullHD frame without barrel
	// visible. For such cases renderers usually renders bigger images and
	// crops them after the distortion.
	//
	// This method is templated to be able to distort byte and float buffers
	// without having separate methods for this two types. So basically only
	//
	// But in fact PixelType might be any type for which multiplication by
	// a scalar and addition are implemented. For example PixelType might be
	// Vec3 as well.
	template<typename PixelType>
	void UndistortBuffer(const PixelType *input_buffer,
		int width,
		int height,
		double overscan,
		int channels,
		PixelType *output_buffer);

private:
	// This is the size of the image. This is necessary to, for example, handle
	// the case of processing a scaled image.
	int image_width_;
	int image_height_;

	// The traditional intrinsics matrix from x = K[R|t]X.
	Matx33d K_;

	// Coordinate lookup grids for distortion and undistortion.
	//internal::LookupWarpGrid distort_;
	//internal::LookupWarpGrid undistort_;

protected:
	// Reset lookup grids after changing the distortion model.
	void ResetLookupGrids();
};