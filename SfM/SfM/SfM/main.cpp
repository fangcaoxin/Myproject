/**
 * @file SURF_FlannMatcher
 * @brief SURF detector + descriptor + FLANN Matcher
 * @author A. Huaman
 */

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/core/core.hpp>

#include "FeatureMatching.h"
#include "CalculateCameraMatrix.h"
#include "Triangulation.h"
#include "Common.h"
#include "SaveXYZimages.h"


#include <stdlib.h>
//#include <windows.h>

#include <math.h>



using namespace std;
using namespace cv;


void readme();

float imgdata[2448][3264][3];
float texture[2448][3264][3];
int width=0, height=0, rx = 0, ry = 0;  
int eyex = 30, eyez = 20, atx = 100, atz = 50; 
int eyey = -15;
float scalar = 0.1;        //scalar of converting pixel color to float coordinates 
vector<CloudPoint> pointcloud;
float allx = 0.0;
float ally = 0.0;
float allz = 0.0; 
 





////Function Main
int main( int argc, char** argv )
{

	string filename1 = "C:\\Research\\OpenCVProject\\Myproject\\ColorCorrection\\image\\res_fusion_1.jpg";
	string filename2 = "C:\\Research\\OpenCVProject\\Myproject\\ColorCorrection\\image\\res_fusion_2.jpg";
	Mat img_1 = imread(filename1);
	Mat img_2 = imread(filename2);
	std::vector<KeyPoint> keypoints_1, keypoints_2,keypts1_good,keypts2_good, corr;
	std::vector< DMatch > matches;
	width = img_1.cols;
	height = img_1.rows;

	if( !img_1.data || !img_2.data )
	{ std::cout<< " --(!) Error reading images " << std::endl; return -1; } /// Read in Images

	// Start Feature Matching
	int Method = 1;
	FeatureMatching(img_1,img_2,keypoints_1,keypoints_2,keypts1_good,keypts2_good,&matches,Method); // matched featurepoints
	 // Calculate Matrices
	vector<Point2f> pts1,pts2;
	vector<uchar> status;


	vector<KeyPoint> imgpts1_tmp,imgpts1_good,imgpts2_good;
	vector<KeyPoint> imgpts2_tmp;
	GetAlignedPointsFromMatch(keypoints_1, keypoints_2, matches, imgpts1_tmp, imgpts2_tmp);
	KeyPointsToPoints(imgpts1_tmp, pts1);
	KeyPointsToPoints(imgpts2_tmp, pts2);
	double minVal,maxVal;
	cv::minMaxIdx(pts1,&minVal,&maxVal);

	Mat F = findFundamentalMat(pts1, pts2, FM_RANSAC, 0.006*maxVal, 0.99, status);
	cout << "F" << F << endl;
	double status_nz = countNonZero(status); 
	double status_sz = status.size();
	double kept_ratio = status_nz / status_sz;

	vector<DMatch> new_matches;
	cout << "F keeping " << countNonZero(status) << " / " << status.size() << endl;	
	for (unsigned int i=0; i<status.size(); i++) {
		if (status[i]) 
		{
			imgpts1_good.push_back(imgpts1_tmp[i]);
			imgpts2_good.push_back(imgpts2_tmp[i]);

			new_matches.push_back(matches[i]);

			//good_matches_.push_back(DMatch(imgpts1_good.size()-1,imgpts1_good.size()-1,1.0));
		}
	}	
	
	cout << matches.size() << " matches before, " << new_matches.size() << " new matches after Fundamental Matrix\n";
	matches = new_matches; //keep only those points who survived the fundamental matrix

	Mat img_matches;
	drawMatches( img_1, keypoints_1, img_2, keypoints_2,
		matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );		
	//-- Show detected matches
	imshow( "Feature Matches", img_matches );
	imwrite("match_fusion.jpg", img_matches);
	waitKey(0);
	destroyWindow("Feature Matches");
	


	/////////////////////
	Mat Kinv,discoeff; // Read from calibration file

	double f = 1057.14;
	double cx = 640/4;
	double cy = 480/4;
	const double K_array[3][3] = {
		{ f,0,cx },
		{ 0,f,cy },
		{ 0,0,1 }
	};
	const size_t bufsize = sizeof(double) * 3 * 3;
	
	Mat K(3, 3, CV_64FC1, Scalar(0));
	memcpy(K.data, &K_array[0][0], bufsize);
	Kinv = K.inv();
	cout << K << " " <<endl<< Kinv << endl;
	Matx34d P, P1;
	
	

	bool CM = FindCameraMatrices(K,Kinv,F,P,P1,discoeff,imgpts1_tmp,imgpts2_tmp,imgpts1_good,imgpts2_good,matches,pointcloud);
	
	// Reconstruct 3D
	//double mse = TriangulatePoints(keypts1_good,keypts2_good,K,Kinv,P,P1,pointcloud,keypts1_good,discoeff);
	vector<int> r_value;
	vector<double> depths;
	for (int k = 0; k < pointcloud.size(); k++) {
		Point point_xy = imgpts1_good[k].pt;
		r_value.push_back(img_1.at<Vec3b>(point_xy)[2]);
		depths.push_back(pointcloud[k].pt.z);
		cout << "red " << (int)img_1.at<Vec3b>(point_xy)[2] << endl;
		cout << "depth " << pointcloud[k].pt.z << endl;
	}

	double r_min, r_max;
	double d_min, d_max;
	normalize(depths, depths, 0, 1, NORM_MINMAX);
	minMaxLoc(r_value, &r_min, &r_max);
	minMaxLoc(depths, &d_min, &d_max);
	int range_r = r_max - r_min;
	double range_depth = d_max - d_min;
	// Write points to file
#ifdef SAVE
	Mat X(img_1.rows,img_1.cols,CV_32FC1);
	Mat Y(img_1.rows,img_1.cols,CV_32FC1);
	Mat Z(img_1.rows,img_1.cols,CV_32FC1);
	string filepath = "C:\\Research\\OpenCVProject\\Myproject\\SfM\\";
	saveXYZimages(img_1,pointcloud,imgpts1_good,filepath,X,Y,Z);

	double Nindex = X.rows * X.cols;


	for(int i=0;i<pointcloud.size();i++ )
	{
		allx += pointcloud[i].pt.x;
		ally += pointcloud[i].pt.y;
		allz += pointcloud[i].pt.z;
	}
	allx = 1.0 * allx/(float)pointcloud.size();
	ally = 1.0 * ally/(float)pointcloud.size();
	allz = 1.0 * allz/(float)pointcloud.size();
#endif //SAVE

	//cvWaitKey(0);

	return 0;
}

/**
 * @function readme
 */
void readme()
{ std::cout << " Usage: ./SURF_FlannMatcher <img1> <img2>" << std::endl; }