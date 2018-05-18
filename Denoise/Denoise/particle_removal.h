#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <opencv2/ml.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>

void calcDarkChannel(cv::Mat& darkchannel, cv::Mat& brightchannel, cv::Mat& input, int radius);
void calcPyrLKflow(std::vector<cv::Mat>& imageList_gray, cv::Mat& object_area, std::vector<cv::Mat>& camera_motion);
void check_FB(const std::vector<cv::Mat>& oldImagePyr, const std::vector<cv::Mat>& newImagePyr,
	const std::vector<cv::Point2f>& oldPoints, const std::vector<cv::Point2f>& newPoints, std::vector<bool>& status);
void check_NCC(const cv::Mat& oldImage, const cv::Mat& newImage,
	const std::vector<cv::Point2f>& oldPoints, const std::vector<cv::Point2f>& newPoints, std::vector<bool>& status);
template<typename T>
T getMedian(const std::vector<T>& values);
template<typename T>
T getMedianAndDoPartition(std::vector<T>& values);
cv::Mat getPatch(cv::Mat image, cv::Size patch_size, cv::Point2f patch_center);
void imageListCompensation(std::vector<cv::Mat>& image_list, 
	std::vector<cv::Mat>& image_list_compensation, 
	std::vector<cv::Mat>& camera_motion);
void imageListGrayCompensation(std::vector<cv::Mat>& image_list_gray, 
	std::vector<cv::Mat>& image_list_gray_compensation,
	std::vector<cv::Mat>& camera_motion);
void FrameRelativeDiff(std::vector<cv::Mat>& image_list_gray, std::vector<cv::Mat>& diff);
void diffByThreshold(std::vector<cv::Mat>& diff, std::vector<cv::Mat>& diff_wb, int threshold_wb);
void rgbStdDev(cv::Mat& image, cv::Mat& labels, cv::Mat& stats, cv::Mat& normalize_std, int size);
void getMaskFromStd(cv::Mat& mask, cv::Mat& normalize_std);
int sumAreaByRadius(cv::Mat& label_pre, cv::Mat& label_next, cv::Mat& centroids_pre, cv::Mat& centroids_next, cv::Mat& sum1, int radius);
void imageClosing(cv::Mat& src, cv::Mat& output, int kenel_size);
void floatingAreaRestoration(std::vector<cv::Mat>& image_list, std::vector<cv::Mat>& image_list_gray, cv::Mat& stats, cv::Mat& label, cv::Mat& output);