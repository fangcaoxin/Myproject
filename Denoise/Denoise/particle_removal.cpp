#include "particle_removal.h"

void check_FB(const std::vector<cv::Mat>& oldImagePyr, const std::vector<cv::Mat>& newImagePyr,
	const std::vector<cv::Point2f>& oldPoints, const std::vector<cv::Point2f>& newPoints, std::vector<bool>& status)
{
	if (status.empty()) {
		status = std::vector<bool>(oldPoints.size(), true);
	}
	cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
	const int MAX_COUNT = 500;
	cv::Mat gray, preGray, image, frame;
	cv::Size subPixWinSize(10, 10), winSize(11, 11);
	int maxLevel = 4;

	std::vector<uchar> LKstatus(oldPoints.size());
	std::vector<float> errors(oldPoints.size());
	std::vector<float> FBerror(oldPoints.size());
	std::vector < cv::Point2f > pointsToTrackReprojection;
	calcOpticalFlowPyrLK(newImagePyr, oldImagePyr, newPoints, pointsToTrackReprojection, LKstatus, errors,
		winSize, maxLevel, termcrit, 0);

	for (size_t i = 0; i<oldPoints.size(); i++) {
		FBerror[i] = (float)norm(oldPoints[i] - pointsToTrackReprojection[i]);

	}
	float FBerrorMedian = getMedian(FBerror);
	for (size_t i = 0; i<oldPoints.size(); i++) {
		status[i] = status[i] && (FBerror[i] <= FBerrorMedian);
	}
}
void check_NCC(const cv::Mat& oldImage, const cv::Mat& newImage,
	const std::vector<cv::Point2f>& oldPoints, const std::vector<cv::Point2f>& newPoints, std::vector<bool>& status)
{
	std::vector<float> NCC(oldPoints.size(), 0.0);
	cv::Mat p1, p2;
	cv::Size winSizeNCC(51, 51);
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

cv::Mat getPatch(cv::Mat image, cv::Size patch_size, cv::Point2f patch_center)
{
	cv::Mat patch;
	cv::Point2i roi_strat_corner(cvRound(patch_center.x - patch_size.width / 2.),
		cvRound(patch_center.y - patch_size.height / 2.));

	cv::Rect2i patch_rect(roi_strat_corner, patch_size);

	if (patch_rect == (patch_rect & cv::Rect2i(0, 0, image.cols, image.rows)))
	{
		patch = image(patch_rect);
	}
	else
	{
		getRectSubPix(image, patch_size,
			cv::Point2f((float)(patch_rect.x + patch_size.width / 2.),
			(float)(patch_rect.y + patch_size.height / 2.)), patch);
	}

	return patch;
}
static size_t filterPointsInVectors(std::vector<bool>& status, std::vector<cv::Point2f>& vec1, std::vector<cv::Point2f>& vec2, bool goodValue)
{
	CV_DbgAssert(status.size() == vec1.size() && status.size() == vec2.size());

	size_t first_bad_idx = 0;
	while (first_bad_idx < status.size())
	{
		if (status[first_bad_idx] != goodValue)
			break;
		first_bad_idx++;
	}

	if (first_bad_idx >= status.size())
		return first_bad_idx;

	for (size_t i = first_bad_idx + 1; i < status.size(); i++)
	{
		if (status[i] != goodValue)
			continue;

		status[first_bad_idx] = goodValue;
		vec1[first_bad_idx] = vec1[i];
		vec2[first_bad_idx] = vec2[i];
		first_bad_idx++;
	}
	vec1.erase(vec1.begin() + first_bad_idx, vec1.end());
	vec2.erase(vec2.begin() + first_bad_idx, vec2.end());
	status.erase(status.begin() + first_bad_idx, status.end());

	return first_bad_idx;
}

void calcDarkChannel(cv::Mat& darkchannel, cv::Mat& brightchannel, cv::Mat& input, int radius)
{


	int height = input.rows;
	int width = input.cols;
	darkchannel.create(height, width, CV_8UC1);
	brightchannel.create(height, width, CV_8UC1);

	int st_row, ed_row;
	int st_col, ed_col;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			st_row = i - radius, ed_row = i + radius;
			st_col = j - radius, ed_col = j + radius;

			st_row = st_row < 0 ? 0 : st_row;
			ed_row = ed_row >= height ? (height - 1) : ed_row;
			st_col = st_col < 0 ? 0 : st_col;
			ed_col = ed_col >= width ? (width - 1) : ed_col;


			int min = 300;
			int max = 0;
			for (int m = st_row; m <= ed_row; m++)
			{
				for (int n = st_col; n <= ed_col; n++)
				{
					for (int k = 0; k < 3; k++)
					{

						int cur = input.at<cv::Vec3b>(m, n)[k];
						if (cur < min)
							min = cur;

						if (cur > max)
							max = cur;
					}
				}
			}
			darkchannel.at<uchar>(i, j) = min;
			brightchannel.at<uchar>(i, j) = max;
		}
	}

}
void calcPyrLKflow(std::vector<cv::Mat>& imageList_gray, cv::Mat& object_area, std::vector<cv::Mat>& camera_motion) 
{
	std::vector<cv::Point2f> points[3];

	cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
	const int MAX_COUNT = 500;
	cv::Size subPixWinSize(10, 10), winSize(11, 11);
	int maxLevel = 4;

	std::vector<uchar> status;
	std::vector<float> err;


	cv::goodFeaturesToTrack(imageList_gray[1], points[1], MAX_COUNT, 0.01, 10, object_area, 3, 0, 0.04);
	cv::cornerSubPix(imageList_gray[1], points[1], subPixWinSize, cv::Size(-1, -1), termcrit);

	std::vector<cv::Mat> oldImagePyr, newImagePyr, newImagePyr1;
	cv::buildOpticalFlowPyramid(imageList_gray[1], oldImagePyr, winSize, maxLevel, false);
	cv::buildOpticalFlowPyramid(imageList_gray[0], newImagePyr, winSize, maxLevel, false);
	cv::buildOpticalFlowPyramid(imageList_gray[2], newImagePyr1, winSize, maxLevel, false);
	cv::calcOpticalFlowPyrLK(oldImagePyr, newImagePyr, points[1], points[0], status, err, winSize, 3, termcrit, 0, 0.001);
	cv::calcOpticalFlowPyrLK(oldImagePyr, newImagePyr1, points[1], points[2], status, err, winSize, 3, termcrit, 0, 0.001);

	std::vector<bool>filter_status(points[1].size(), true);
	std::vector<bool> filter_status1(points[1].size(), true);
	//checkValidArea(object_area, points[1], filter_status);
	check_FB(oldImagePyr, newImagePyr, points[1], points[0], filter_status);
	check_NCC(imageList_gray[1], imageList_gray[0], points[1], points[0], filter_status);
	//checkValidArea(object_area, points[1], filter_status1);
	check_FB(oldImagePyr, newImagePyr1, points[1], points[2], filter_status1);
	check_NCC(imageList_gray[1], imageList_gray[2], points[1], points[2], filter_status1);
	std::vector<cv::Point2f> points1 = points[1];

	size_t good_points_after_filter = filterPointsInVectors(filter_status, points[1], points[0], true);
	size_t good_points_after_filter1 = filterPointsInVectors(filter_status1, points1, points[2], true);

	cv::Mat homo_10 = findHomography(points[0], points[1], CV_RANSAC, 3.0);
	cv::Mat homo_12 = findHomography(points[2], points1, CV_RANSAC, 3.0);
	camera_motion.push_back(homo_10);
	camera_motion.push_back(homo_12);
	std::cout << "camera motion " << homo_10 << " " << homo_12 << std::endl;
}
void imageListCompensation(std::vector<cv::Mat>& image_list, std::vector<cv::Mat>& image_list_compensation, std::vector<cv::Mat>& camera_motion)
{

	cv::Mat image_pre_compensation;
	cv::Mat image_next_compensation;

	cv::warpPerspective(image_list[0], image_pre_compensation, camera_motion[0], image_list[0].size(), 1, cv::BORDER_REPLICATE);
	cv::warpPerspective(image_list[2], image_next_compensation, camera_motion[1], image_list[1].size(), 1, cv::BORDER_REPLICATE);

	image_list_compensation.push_back(image_pre_compensation);
	image_list_compensation.push_back(image_list[1]);
	image_list_compensation.push_back(image_next_compensation);

}
void imageListGrayCompensation(std::vector<cv::Mat>& image_list_gray, std::vector<cv::Mat>& image_list_gray_compensation, std::vector<cv::Mat>& camera_motion)
{
	cv::Mat image_pre_compensation;
	cv::Mat image_next_compensation;

	cv::warpPerspective(image_list_gray[0], image_pre_compensation, camera_motion[0], image_list_gray[0].size(), 1, cv::BORDER_REPLICATE);
	cv::warpPerspective(image_list_gray[2], image_next_compensation, camera_motion[1], image_list_gray[1].size(), 1, cv::BORDER_REPLICATE);

	image_list_gray_compensation.push_back(image_pre_compensation);
	image_list_gray_compensation.push_back(image_list_gray[1]);
	image_list_gray_compensation.push_back(image_next_compensation);
}
void FrameRelativeDiff(std::vector<cv::Mat>& image_list_gray, std::vector<cv::Mat>& diff) 
{
	int size = image_list_gray.size();
	int mid = size / 2;
	std::vector<cv::Mat> forward_diff;
	std::vector<cv::Mat> backward_diff;
	for (int i = 0; i < mid; i++) {
		forward_diff.push_back(image_list_gray[i + 1] - image_list_gray[i]);
	}

	for (int i = mid; i < size - 1; i++) {
		backward_diff.push_back(image_list_gray[i] - image_list_gray[i + 1]);
	}

	diff.insert(diff.end(), forward_diff.begin(), forward_diff.end());
	diff.insert(diff.end(), backward_diff.begin(), backward_diff.end());
}
void diffByThreshold(std::vector<cv::Mat>& diff, std::vector<cv::Mat>& diff_wb, int threshold_wb) {
	int size = diff.size();

	for (int i = 0; i < size; i++) {
		cv::Mat tmp_wb;
		threshold(diff[i], tmp_wb, threshold_wb, 255, CV_THRESH_BINARY);
		diff_wb.push_back(tmp_wb);
	}
}
void rgbStdDev(cv::Mat& image, cv::Mat& labels, cv::Mat& stats, cv::Mat& normalize_std, int size) 
{
	int width = image.cols;
	int height = image.rows;
	cv::Mat normalize_std_tmp(size, 2, CV_32FC1, cv::Scalar(0));
	cv::Mat rgb_std(size, 3, CV_32FC1, cv::Scalar(0));
	cv::Mat rgb_sum(size, 3, CV_32FC1, cv::Scalar(0));
	cv::Mat rgb_sum2(size, 3, CV_32FC1, cv::Scalar(0));

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (labels.at<int>(i, j) == 0) continue;
			for (int k = 0; k < 3; k++) {
				rgb_sum.at<float>(labels.at<int>(i, j) - 1, k) += (float)image.at<cv::Vec3b>(i, j)[k];
				rgb_sum2.at<float>(labels.at<int>(i, j) - 1, k) += (float)image.at<cv::Vec3b>(i, j)[k] * (float)image.at<cv::Vec3b>(i, j)[k];
			}
		}
	}
	for (int m = 0; m < size; m++) {
		for (int k = 0; k < 3; k++) {
			rgb_std.at<float>(m, k) = rgb_sum2.at<float>(m, k) / stats.at<int>(m + 1, 4) - (rgb_sum.at<float>(m, k) / stats.at<int>(m + 1, 4))*(rgb_sum.at<float>(m, k) / stats.at<int>(m + 1, 4));
		}
		float sum_bgr = rgb_std.at<float>(m, 0) + rgb_std.at<float>(m, 1) + rgb_std.at<float>(m, 2);

		if (stats.at<int>(m + 1, 4) < 3 || sum_bgr<0.001) {
			normalize_std_tmp.at<float>(m, 0) = 20;

		}
		else {
			float b_stddev = sqrt(rgb_std.at<float>(m, 0) / 1);
			float g_stddev = sqrt(rgb_std.at<float>(m, 1) / 1);
			float r_stddev = sqrt(rgb_std.at<float>(m, 2) / 1);


			float average_std = (b_stddev + g_stddev + r_stddev) / 3;
			float n_stddev = ((r_stddev - average_std)*(r_stddev - average_std) + \
				(g_stddev - average_std)*(g_stddev - average_std) + \
				(b_stddev - average_std)*(b_stddev - average_std)) / 3;
			/*float abs_bg = abs(sqrt(rgb_std.at<float>(m, 0) / sum_bgr) - sqrt(rgb_std.at<float>(m, 1) / sum_bgr));
			float abs_gr = abs(sqrt(rgb_std.at<float>(m, 1) / sum_bgr) - sqrt(rgb_std.at<float>(m, 2) / sum_bgr));
			normalize_std_tmp.at<float>(m, 0) = cv::abs(abs_bg - abs_gr);*/
			normalize_std_tmp.at<float>(m, 0) = sqrt(n_stddev);



		}
		//cout << "t_std " << normalize_std_tmp.at<float>(m, 0) << endl;
	}

	normalize_std = normalize_std_tmp;
}
void getMaskFromStd(cv::Mat& mask, cv::Mat& normalize_std) 
{
	int width = mask.cols;
	int height = mask.rows;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {

			int cur = mask.at<int>(i, j);
			if (cur == 0) continue;
			if (normalize_std.at<float>(cur - 1, 0) > 5) {
				mask.at<int>(i, j) = 0;
			}
		}
	}
}
void imageClosing(cv::Mat& src, cv::Mat& output, int kenel_size) {
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kenel_size, kenel_size));
	cv::morphologyEx(src, output, cv::MORPH_CLOSE, element);
}
int sumAreaByRadius(cv::Mat& label_pre, cv::Mat& label_next, cv::Mat& centroids_pre, cv::Mat& centroids_next, cv::Mat& sum1, int radius)
{
	int height = label_pre.rows;
	int width = label_pre.cols;
	cv::Mat sum(height, width, CV_8UC1, cv::Scalar(0));
	int st_row, ed_row;
	int st_col, ed_col;

	std::vector<int> list_0;
	std::vector<int> list_1;
	for (int k = 1; k < centroids_pre.rows; k++) {
		int i = centroids_pre.at<double>(k, 1);
		int j = centroids_pre.at<double>(k, 0);
		st_row = i - radius, ed_row = i + radius;
		st_col = j - radius, ed_col = j + radius;

		st_row = st_row < 0 ? 0 : st_row;
		ed_row = ed_row >= height ? (height - 1) : ed_row;
		st_col = st_col < 0 ? 0 : st_col;
		ed_col = ed_col >= width ? (width - 1) : ed_col;

		for (int m = st_row; m <= ed_row; m++)
		{
			for (int n = st_col; n <= ed_col; n++)
			{
				if (label_next.at<int>(m, n) != 0) {
					int label = label_next.at<int>(m, n);
					double dis = (centroids_next.at<double>(label, 1) - i)*(centroids_next.at<double>(label, 1) - i) + (centroids_next.at<double>(label, 0) - j)*(centroids_next.at<double>(label, 0) - j);
					if (dis < radius*radius) {

						list_0.push_back(k);
						list_1.push_back(label);
					}
				}
			}
		}
	}
	int num = 0;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int label_0 = label_pre.at<int>(i, j);
			int label_1 = label_next.at<int>(i, j);
			std::vector<int>::iterator iter_0 = find(list_0.begin(), list_0.end(), label_0);
			std::vector<int>::iterator iter_1 = find(list_1.begin(), list_1.end(), label_1);
			if (iter_0 != list_0.end()) {
				sum.at<uchar>(i, j) = 255;
				num++;
			}

			if (iter_1 != list_1.end()) {
				sum.at<uchar>(i, j) = 255;
				num++;
			}
		}
	}
	sum1 = sum;
	return num;

}
static void createSamplesFrames(std::vector<cv::Mat>& image_list_gray, cv::Mat& label, std::vector<cv::Mat>& samples, cv::Mat& stats, int size) {
	int width = label.cols;
	int height = label.rows;
	std::vector<cv::Mat> floatSource(3);
	std::vector<cv::Mat> tmp_samples(size);
	std::vector<int> idx(size);
	for (int k = 0; k < image_list_gray.size(); k++) {
		image_list_gray[k].convertTo(floatSource[k], CV_32F);
	}

	for (int m = 0; m < size; m++) {
		tmp_samples[m].create(3 * stats.at<int>(m + 1, 4), 1, CV_32FC1);
	}
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int label_num = label.at<int>(i, j);
			if (label_num > 0) {
				for (int frame = 0; frame < image_list_gray.size(); frame++) {
					tmp_samples[label_num - 1].at<float>(idx[label_num - 1]++, 0) = image_list_gray[frame].at<uchar>(i, j);
				}
			}

		}
	}
	samples = tmp_samples;
}
static void EMModel(std::vector<cv::Mat>& samples, std::vector < cv::Ptr < cv::ml::EM>> &em_models) 
{
	for (int i = 0; i < samples.size(); i++) {
		em_models[i] = cv::ml::EM::create();

		em_models[i]->setClustersNumber(2);
		em_models[i]->setCovarianceMatrixType(cv::ml::EM::COV_MAT_SPHERICAL);
		em_models[i]->setTermCriteria(cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 300, 0.1));
		em_models[i]->trainEM(samples[i], cv::noArray(), cv::noArray(), cv::noArray());
	}
}

static void restorationBaseMationBrightEM(std::vector<cv::Mat>& image_list_compensation, 
	std::vector<cv::Mat>& image_list_gray, cv::Mat& label, std::vector<cv::Ptr<cv::ml::EM>>& models, cv::Mat& output) {
	int width = image_list_compensation[0].cols;
	int height = image_list_compensation[0].rows;

	image_list_compensation[1].copyTo(output);
	struct max_probs_labels {
		cv::Point pos;
		int frame_num;
		float prob;
	};
	std::vector<int> flag;
	std::vector<max_probs_labels> label_max(models.size());
	std::vector<cv::Point> need_to_repair;
	for (int k = 0; k < models.size(); k++) {
		cv::Mat means = models[k]->getMeans();
		int flag_tmp = means.at<double>(0, 0) > means.at<double>(1, 0) ? 1 : 0;
		flag.push_back(flag_tmp);
	}

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int label_num = label.at<int>(i, j);
			if (label_num > 0) {
				float max = 0.;
				int frame_num = 1;
				label_max[label_num - 1].prob = 0.5;
				for (int k = 0; k < 3; k++) {
					std::vector<float> probs;
					models[label_num - 1]->predict2(image_list_gray[k].at<uchar>(i, j), probs);
					if (probs[flag[label_num - 1]] > max) {
						max = probs[flag[label_num - 1]];
						frame_num = k;
					}
					if (probs[flag[label_num - 1]] > label_max[label_num - 1].prob) {
						label_max[label_num - 1].frame_num = k;
						label_max[label_num - 1].pos = cv::Point(j, i);
						label_max[label_num - 1].prob = probs[flag[label_num - 1]];
					}
				}
				//cout << "probs " << max << " frame "<< frame_num<< endl;
				if (max > 0) {
					output.at<cv::Vec3b>(i, j) = image_list_compensation[frame_num].at<cv::Vec3b>(i, j);
				}
				else {
					//output.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
					need_to_repair.push_back(cv::Point(j, i));
				}
			}
		}
	}
	if (need_to_repair.size() > 0) {
		for (int k = 0; k < need_to_repair.size(); k++) {
			int label_num = label.at<int>(need_to_repair[k]);
			output.at<cv::Vec3b>(need_to_repair[k]) = image_list_compensation[label_max[label_num - 1].frame_num].at<cv::Vec3b>(label_max[label_num - 1].pos);
		}
	}


}
void floatingAreaRestoration(std::vector<cv::Mat>& image_list, std::vector<cv::Mat>& image_list_gray, cv::Mat& stats, cv::Mat& label, cv::Mat& output)
{
	int size = stats.rows - 1;
	std::vector<cv::Mat> samples;
	std::vector<cv::Ptr<cv::ml::EM>> em_models(size);
	createSamplesFrames(image_list_gray, label, samples, stats, size);
	EMModel(samples, em_models);
	restorationBaseMationBrightEM(image_list, image_list_gray, label, em_models, output);

}