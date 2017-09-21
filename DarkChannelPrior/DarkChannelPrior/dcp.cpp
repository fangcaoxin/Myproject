//
//  dcp.cpp
//  opencv_test
//
//  Created by xiaoruiqiao on 2017/9/3.
//  Copyright © 2017年 xiaoruiqiao. All rights reserved.
//

#include "dcp_core.h"

using namespace cv::ximgproc;

void dehaze(IplImage *recover, IplImage *input)
{
    int height = input->height;
    int width = input->width;
    
    IplImage *darkchannel = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    IplImage *transmission = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    IplImage *refine_transmission = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    
    int darkchannelradius = cvRound(MIN(width, height) * 0.02);
    double Airlight[3] = { 0.0, 0.0, 0.0 };
    
     printf("CalcDarkChannel...");
//    CalcDarkChannel(darkchannel, input, darkchannelradius);
    
    printf("CalcAirlight...");
    CalcAirlight(darkchannel, input, Airlight);
    
    
    printf("CalcTransmission...");
    CalcTransmission(transmission, input, Airlight, darkchannelradius);
   
    
   
    printf("GuidedFilterColor...");
    // GuidedFilterColor() is my own implementation of guided filter
    // guidedFilter() is the implementation included in OpenCV 3.0
    // The result is almost the same, however my implementation is much more slower
    // because I haven't spend much time optimizing the code efficiency.
    
    // See http://research.microsoft.com/en-us/um/people/kahe/eccv10/ for more details
    // ref:
    // He, Kaiming, Jian Sun, and Xiaoou Tang. "Guided image filtering."
    // Pattern Analysis and Machine Intelligence, IEEE Transactions on 35.6 (2013): 1397-1409.
    
    //GuidedFilterColor(refine_transmission, input, transmission, 1e-6, 60);
     guidedFilter(cv::cvarrToMat(input), cv::cvarrToMat(transmission), cv::cvarrToMat(refine_transmission), 60, 1e-6);
   
    
   
    printf("CalcRecover...");
    CalcRecover(recover, input, refine_transmission, Airlight);
   
    
    cvReleaseImage(&refine_transmission);
    cvReleaseImage(&transmission);
    cvReleaseImage(&darkchannel);
}
