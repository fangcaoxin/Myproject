#include "core.h"
#define USE_FLOW

void darkChannelFrames(const vector<Mat> &imgList, int frameNum, Mat& ouput, const vector<Mat>& flow) {
	vector<Mat> imgListGray;

	for (int i = 0; i < frameNum; i++) {
		Mat cur;
		cvtColor(imgList[i], cur, CV_BGR2GRAY);
		imgListGray.push_back(cur);
	}

	int width = imgList[0].cols;
	int height = imgList[0].rows;
	
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int darkFrameNum = 0;
			int min_value = imgListGray[0].at<uchar>(i,j);
			//int min_value = 255;
			Vec3b frameNumvalue;
		
			frameNumvalue.val[0] = 0;
			frameNumvalue.val[1] = 0;
			frameNumvalue.val[2] = 0;
		
			    for (int k = 1; k < frameNum; k++) {
				Point2i cur_pos;
				cur_pos.x = j;
				cur_pos.y = i;



#ifdef USE_FLOW
				/*int radius = 0;
				int st_row, ed_row;
				int st_col, ed_col;
				st_row = i - radius, ed_row = i + radius;
				st_col = j - radius, ed_col = j + radius;
				st_row = st_row < 0 ? 0 : st_row;
				ed_row = ed_row >= height ? (height - 1) : ed_row;
				st_col = st_col < 0 ? 0 : st_col;
				ed_col = ed_col >= width ? (width - 1) : ed_col;
				if (norm(flow[k-1].at<Point2f>(cur_pos)) <0.1 ) {
					
					int count = 0;
					for (int m = st_row; m <= ed_row; m++)
					{
						for (int n = st_col; n <= ed_col; n++)
						{
							if (norm(flow[k - 1].at<Point2f>(m, n)) >= 0.1) {
								count++;
							}
						}
					}
*/
					if (norm(flow[k - 1].at<Point2f>(cur_pos)) <0.1) {
						int cur_value = imgListGray[k].at<uchar>(cur_pos);
						if (cur_value < min_value) {
							min_value = cur_value;
							darkFrameNum = k;
						}
					}
					else {
						darkFrameNum = 0;
					}

				}
					
					

				
					
#else
				int cur_value = imgListGray[k].at<uchar>(cur_pos);
				if (cur_value < min_value) {
					min_value = cur_value;
					darkFrameNum = k;
				}
				/*cur_pos.x = j + flow[k-1].at<Point2f>(i,j).x;
				cur_pos.y = i + flow[k-1].at<Point2f>(i,j).y;
				cur_pos.x = cur_pos.x < 0 ? 0 : cur_pos.x;
				cur_pos.x = cur_pos.x > width-1 ? width-1 : cur_pos.x;
				cur_pos.y = cur_pos.y < 0 ? 0 : cur_pos.y;
				cur_pos.y = cur_pos.y > height-1 ? height-1 : cur_pos.y;*/
#endif //USE_FLOW
				
			




			/*if (darkFrameNum > 0) {
				cout << "darknum is :" << darkFrameNum << endl;
			}*/
			//frameNumvalue.val[darkFrameNum] = 255;
//			Point2i resPoint;
//			if (darkFrameNum == 0) {
//				resPoint.x = j ;
//				resPoint.y = i ;
//			}
//			else {
//				
//					resPoint.x = j;
//					resPoint.y = i;
//#ifdef USE_FLOW				
//			
//					resPoint.x = j + floor(flow[darkFrameNum - 1].at<Point2f>(i, j).x);
//					resPoint.y = i + floor(flow[darkFrameNum - 1].at<Point2f>(i, j).y);
//					resPoint.x = resPoint.x < 0 ? 0 : resPoint.x;
//					resPoint.x = resPoint.x > width - 1 ? width - 1 : resPoint.x;
//					resPoint.y = resPoint.y < 0 ? 0 : resPoint.y;
//					resPoint.y = resPoint.y > height - 1 ? height - 1 : resPoint.y;
//#endif //USE_FLOW
//				
//			}
			ouput.at<Vec3b>(i, j) = imgList[darkFrameNum].at<Vec3b>(i,j);
			
			/*cout << "point is " << i<< " " << j <<
				"cal point is "<<resPoint.y<<" "<<resPoint.x << endl;*/
		}
	}
}