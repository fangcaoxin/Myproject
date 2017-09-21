#include "utility.h"

void calcDenseFlow(vector<Mat>& imageList_gray, vector<Mat>& flow_list) {
	Mat uflow, uflow1;
	calcOpticalFlowFarneback(imageList_gray[1], imageList_gray[0], uflow, 0.5, 1, 50, 3, 5., 1.2, 0);
	calcOpticalFlowFarneback(imageList_gray[2], imageList_gray[0], uflow1, 0.5, 1, 50, 3, 5., 1.2, 0);
	refineFlow(uflow, uflow1, 1);
	refineFlowTwice(uflow, 200);
	refineFlowTwice(uflow1, 200);
	flow_list.push_back(uflow);
	flow_list.push_back(uflow1);
}