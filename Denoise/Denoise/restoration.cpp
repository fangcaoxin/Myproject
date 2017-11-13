#include "restoration.h"

void floatingAreaRestoration(vector<Mat>& image_list, vector<Mat>& image_list_gray, Mat& stats, Mat& label, Mat& output) {
	int size = stats.rows - 1;
	vector<Mat> samples;
	vector<Ptr<EM>> em_models(size);
	createSamplesFrames(image_list_gray, label, samples, stats,  size);
	EMModel(samples, em_models);
	restorationBaseMationBrightEM(image_list, image_list_gray, label, em_models, output);
}