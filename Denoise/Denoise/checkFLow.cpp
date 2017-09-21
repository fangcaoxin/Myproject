#include "core.h"
#include <opencv2/video/tracking.hpp>

static float getAverage(const std::vector<float>& values);

void check_FB(const std::vector<Mat>& oldImagePyr, const std::vector<Mat>& newImagePyr,
	const std::vector<Point2f>& oldPoints, const std::vector<Point2f>& newPoints, std::vector<bool>& status)
{
	if (status.empty()) {
		status = std::vector<bool>(oldPoints.size(), true);
	}
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	const int MAX_COUNT = 500;
	Mat gray, preGray, image, frame;
	Size subPixWinSize(10, 10), winSize(11, 11);
	int maxLevel = 4;

	std::vector<uchar> LKstatus(oldPoints.size());
	std::vector<float> errors(oldPoints.size());
	std::vector<float> FBerror(oldPoints.size());
	std::vector<Point2f> pointsToTrackReprojection;
	calcOpticalFlowPyrLK(newImagePyr, oldImagePyr, newPoints, pointsToTrackReprojection, LKstatus, errors,
		winSize, maxLevel, termcrit, 0);

	for (size_t i = 0; i<oldPoints.size(); i++) {
		FBerror[i] = (float)norm(oldPoints[i] - pointsToTrackReprojection[i]);
		cout << "error is " << FBerror[i] << endl;
	}
	float FBerrorMedian = getMedian(FBerror);
	//printf("point median=%f\n", FBerrorMedian);
	//printf("FBerrorMedian=%f\n", FBerrorMedian);
	//float FBerrorAverage = getAverage(FBerror);
	//printf("point average=%f\n", FBerrorAverage);
	for (size_t i = 0; i<oldPoints.size(); i++) {
		status[i] = status[i] && (FBerror[i] <=FBerrorMedian);
	}
}
void check_NCC(const Mat& oldImage, const Mat& newImage,
	const std::vector<Point2f>& oldPoints, const std::vector<Point2f>& newPoints, std::vector<bool>& status)
{
	std::vector<float> NCC(oldPoints.size(), 0.0);
	Mat p1, p2;
	Size winSizeNCC(51,51);
	for (size_t i = 0; i < oldPoints.size(); i++) {
		p1 = getPatch(oldImage, winSizeNCC, oldPoints[i]);
		p2 = getPatch(newImage, winSizeNCC, newPoints[i]);

		const int patch_area = winSizeNCC.area();
		double s1 = sum(p1)(0), s2 = sum(p2)(0);
		double n1 = norm(p1), n2 = norm(p2);
		double prod = p1.dot(p2);
		double sq1 = sqrt(n1*n1 - s1*s1 / patch_area), sq2 = sqrt(n2*n2 - s2*s2 / patch_area);
		double ares = (sq2 == 0) ? sq1 / abs(sq1) : (prod - s1*s2 / patch_area) / sq1 / sq2;

		NCC[i] = (float)ares;
	}
	float median = getMedian(NCC);
	for (size_t i = 0; i < oldPoints.size(); i++) {
		status[i] = status[i] && (NCC[i] >= median);
	}
}

template<typename T>
T getMedian(const std::vector<T>& values)
{
	std::vector<T> copy(values);
	return getMedianAndDoPartition(copy);
}


static float getAverage(const std::vector<float>& values) {

	size_t size = values.size();
	if (size == 0) return 0;
	float sum = 0.0;
	for (int i = 0; i < size; i++) {
		sum += values[i];
	}
	return sum / size;
}

template<typename T>
T getMedianAndDoPartition(std::vector<T>& values)
{
	size_t size = values.size();
	if (size % 2 == 0)
	{
		std::nth_element(values.begin(), values.begin() + size / 2 - 1, values.end());
		T firstMedian = values[size / 2 - 1];

		std::nth_element(values.begin(), values.begin() + size / 2, values.end());
		T secondMedian = values[size / 2];

		return (firstMedian + secondMedian) / (T)2;
	}
	else
	{
		size_t medianIndex = (size - 1) / 2;
		std::nth_element(values.begin(), values.begin() + medianIndex, values.end());

		return values[medianIndex];
	}
}

Mat getPatch(Mat image, Size patch_size, Point2f patch_center)
{
	Mat patch;
	Point2i roi_strat_corner(cvRound(patch_center.x - patch_size.width / 2.),
		cvRound(patch_center.y - patch_size.height / 2.));

	Rect2i patch_rect(roi_strat_corner, patch_size);

	if (patch_rect == (patch_rect & Rect2i(0, 0, image.cols, image.rows)))
	{
		patch = image(patch_rect);
	}
	else
	{
		getRectSubPix(image, patch_size,
			Point2f((float)(patch_rect.x + patch_size.width / 2.),
			(float)(patch_rect.y + patch_size.height / 2.)), patch);
	}

	return patch;
}

void refineFlow(Mat& flow1, Mat& flow2,int radius) {
	int width = flow1.cols;
	int height = flow1.rows;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			float flow1_length = norm(flow1.at<Point2f>(i, j));
			float flow2_length = norm(flow2.at<Point2f>(i, j));
			if (flow1_length <1) flow1.at<Point2f>(i, j) = { 0 };
			if (flow2_length < 1) flow2.at<Point2f>(i, j) = { 0 };
			if (flow1_length * 2 < flow2_length) {
				flow1.at<Point2f>(i, j) = { 0 };
			}
			if (flow1_length > flow2_length) {
				flow1.at<Point2f>(i, j) = { 0 };
			}
			if (flow1_length >= 1 || flow2_length >= 1) {
				float flow1_direction = fastAtan2(flow1.at<Point2f>(i, j).y, flow1.at<Point2f>(i, j).x);
				float flow2_direction = fastAtan2(flow2.at<Point2f>(i, j).y, flow2.at<Point2f>(i, j).x);
				if (abs(flow1_direction - flow2_direction) > 10) {
					flow1.at<Point2f>(i, j) = { 0 };
					flow2.at<Point2f>(i, j) = { 0 };
				}
			}
			
		}
	}
}

void refineFlowTwice(Mat& flow1, int radius) {
	int width = flow1.cols;
	int height = flow1.rows;
	for (int i = 0; i < height-radius; i = i + radius) {
		for (int j = 0; j < width-radius; j = j + radius) {
			//cout << i << " " << j << endl;
			int count = 0;
			Point2f sum = { 0 };
			for (int m = 0; m < radius; m++) {
				for (int n = 0; n <  radius; n++) {
					if (norm(flow1.at<Point2f>(m+i, n+j)) > 1) {
						sum += flow1.at<Point2f>(m+i, n+j);
						count++;
					}
				}
			}
			

			for (int m = 0; m < radius; m++) {
				for (int n = 0; n < radius; n++) {
					if (count<radius) {
						flow1.at<Point2f>(m+i, n+j) = { 0 };
					}
					else {
						flow1.at<Point2f>(m+i, n+j) = sum / count;
					}
				}
			}
		}
	}
}

void getKeyPoints(Mat& flow1, Mat flow2, vector<KeyPoint>& kp1, vector<KeyPoint>& kp2) {
	int width = flow1.cols;
	int height = flow1.rows;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			
			kp1.push_back(KeyPoint(flow1.at<Point2f>(i, j), 1.0f, 0.0f));
			kp2.push_back(KeyPoint(flow2.at<Point2f>(i, j), 1.0f, 0.0f));

			}
		}
	}
