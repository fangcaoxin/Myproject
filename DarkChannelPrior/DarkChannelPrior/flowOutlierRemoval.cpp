#include "dcp_core.h"

void FlowOutlierRemoval(Mat& flow) {
	vector<int> label;
	struct flowType {
		Point2f val;
		Point2i pos;
	};
	for (int i = 0; i < flow.rows; i++) {
		for (int j = 0; j < flow.cols; j++) {

		}
	}
}
