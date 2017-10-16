#include "EM.h"

//#define HUE

void EMSegmetation(Mat& img, Mat& label, int pixel_num, int num) {
	int width = img.cols;
	int height = img.rows;
	Mat img_tmp;
	Mat floatSource;
	cvtColor(img, img_tmp, CV_BGR2HSV);
	img_tmp.convertTo(floatSource, CV_32F);
	Mat samples(pixel_num, 3, CV_32FC1);
	int idx = 0;

	/* make samples*/
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			if (label.at<uchar>(y, x) == 1) {
				/*float sum = floatSource.at<Vec3f>(y, x)[0] + floatSource.at<Vec3f>(y, x)[1] + floatSource.at<Vec3f>(y, x)[2];
				Vec3f cur(floatSource.at<Vec3f>(y, x)[0] / sum, floatSource.at<Vec3f>(y, x)[1] / sum, floatSource.at<Vec3f>(y, x)[2] / sum);*/
				samples.at<Vec3f>(idx++, 0) = floatSource.at<Vec3f>(y, x);
				//cout << "floatSource" << cur << endl;
			}
		}
	}

	/*create EM*/
	Mat labels;
	Ptr<EM> em_model = EM::create();

	em_model->setClustersNumber(num);
	em_model->setCovarianceMatrixType(EM::COV_MAT_SPHERICAL);
	em_model->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
	em_model->trainEM(samples, noArray(), labels, noArray());

	Mat means = em_model->getMeans();
	Mat weights = em_model->getWeights();
	cout << "means: " << means << endl;
	cout << "weidths: " << weights << endl;
	/*Scalar mean_0, mean_1, dev_0, dev_1;
	meanStdDev(means.row(0), mean_0, dev_0);
	meanStdDev(means.row(1), mean_1, dev_1);*/
	//int flag = dev_0.val[0] > dev_1.val[0] ? 1 : 0;
	double max = 0;
	int flag = 0;
	for (int i = 0; i < num; i++) {
		if (means.at<double>(i, 0) > max) {
max = means.at<double>(i, 0);
flag = i;
		}
	}

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			int response = 0;
			if (label.at<uchar>(y, x) == 1) {
				/*float sum = floatSource.at<Vec3f>(y, x)[0] + floatSource.at<Vec3f>(y, x)[1] + floatSource.at<Vec3f>(y, x)[2];
				Vec3f cur(floatSource.at<Vec3f>(y, x)[0] / sum, floatSource.at<Vec3f>(y, x)[1] / sum, floatSource.at<Vec3f>(y, x)[2] / sum);*/
				Vec3f cur = floatSource.at<Vec3f>(y, x);
				response = em_model->predict2(cur, noArray())[1];

				if (response == flag) {
					label.at<uchar>(y, x) = 1;
				}
				else {
					label.at<uchar>(y, x) = 0;
				}
			}
		}
	}

}

void EMSegmetationSamples(Mat& img, Mat& samples, vector<int>& valid_labels, vector<float>& probs_color, int num) {
	Mat labels;
	Ptr<EM> em_model = EM::create();

	em_model->setClustersNumber(num);
	em_model->setCovarianceMatrixType(EM::COV_MAT_SPHERICAL);
	em_model->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
	em_model->trainEM(samples, noArray(), labels, noArray());

	Mat means = em_model->getMeans();
	cout << "getMeans() " << means << endl;
	int flag = 0;
#ifdef HUE
	double max = 0;


	for (int i = 0; i < num; i++) {
		if (means.at<double>(i, 0) > max) {
			max = means.at<double>(i, 0);
			flag = i;
		}
	}
#else
	double min = 255;
	for (int i = 0; i < num; i++) {
		if (means.at<double>(i, 1) < min) {
			min = means.at<double>(i, 1);
			flag = i;
		}
	}
#endif //HUE
	for (int i = 0; i < samples.rows; i++) {
		Vec3f cur = samples.row(i);
		//cout << "cur " << cur << endl;
		vector<float> probs;
		int response = 0;
		response = em_model->predict2(cur, probs)[1];
		probs_color.push_back(probs[flag]);
		if (response == flag) {
			valid_labels.push_back(i);

		}
	}
}

void createSamples(Mat& img, Mat& stats, Mat& labels, Mat& samples) {
	int width = img.cols;
	int height = img.rows;
	Mat tmp, floatSource;
	cvtColor(img, tmp, CV_BGR2HSV);
	tmp.convertTo(floatSource, CV_32F);


	Mat tmp_samples(stats.rows - 1, 3, CV_32FC1, Scalar(0));
	//vector<Vec3f> tmp_samples(stats.rows);
	for (int r = 0; r < height; r++) {
		for (int c = 0; c < width; c++) {
			int label_num = labels.at<int>(r, c);
			if (label_num == 0) continue;

			tmp_samples.at<Vec3f>(label_num - 1, 0) += floatSource.at<Vec3f>(r, c) / stats.at<int>(label_num, 4);
			//cout << "floatSource " << tmp_samples.at<Vec3f>(label_num - 1, 0) << endl;
		}
	}

	samples = tmp_samples;
	//cout << "sample " << samples.row(0) << endl;
}

void createSamplesFrames(vector<Mat>& image_list_gray, Mat& label, vector<Mat>& samples,Mat& stats,int size) {
	int width = label.cols;
	int height = label.rows;
	vector<Mat> floatSource(3);
	vector<Mat> tmp_samples(size);
	vector<int> idx(size);
	for (int k = 0; k < image_list_gray.size(); k++){
		image_list_gray[k].convertTo(floatSource[k], CV_32F);
}

	for (int m = 0; m < size; m++) {
		tmp_samples[m].create(3*stats.at<int>(m+1, 4), 1, CV_32FC1);
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

void EMModel(vector<Mat>& samples, vector<Ptr<EM>>& em_models) {
	for (int i = 0; i < samples.size(); i++) {
	     em_models[i] = EM::create();

		em_models[i]->setClustersNumber(2);
		em_models[i]->setCovarianceMatrixType(EM::COV_MAT_SPHERICAL);
		em_models[i]->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
		em_models[i]->trainEM(samples[i], noArray(), noArray(), noArray());
	}
	

}