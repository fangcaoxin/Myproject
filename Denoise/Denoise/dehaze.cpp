#include "dehaze.h"

void dehaze(Mat& recover, Mat& input) {
	int height = input.rows;
	int width = input.cols;

	Mat darkchannel(height, width, CV_8UC1);
	Mat brightchannel(height, width, CV_8UC1);
	Mat transmission(height, width, CV_8UC1);
	Mat refine_transmission(height, width, CV_8UC1);

	int darkchannelradius = cvRound(MIN(width, height)*0.02);
	double Airlight[3] = { 0,0,0 };
	
	printf("CalcDarkChannel...");
	calcDarkChannel(darkchannel, brightchannel, input, darkchannelradius);

	printf("CalcAirLight...");
	calcAirLight(darkchannel, input, Airlight);

	printf("CalcTransmission...");
	calcTransmission(transmission, input, Airlight, darkchannelradius);

	printf("GuidedFilterColor...");
	guidedFilter(input, transmission, refine_transmission, 60, 1e-6);

	printf("CalcRecover...");
	calcRecover(recover, input, refine_transmission, Airlight);
}