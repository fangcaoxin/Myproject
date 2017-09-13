//
//  dftShow.cpp
//  opencv_test
//
//  Created by xiaoruiqiao on 2017/9/3.
//  Copyright © 2017年 xiaoruiqiao. All rights reserved.
//

#include "dftShow.hpp"
int dftShow(Mat img,Mat& output){
    
    if(img.channels()>1) cvtColor(img,img, CV_BGR2GRAY);
    
    Mat padded;                            //expand inputimage to optimal size
    int m = getOptimalDFTSize( img.rows );
    int n = getOptimalDFTSize( img.cols ); //on the border add zero values
    copyMakeBorder(img, padded, 0, m - img.rows,0, n - img.cols, BORDER_CONSTANT, Scalar::all(0));
    
    Mat planes[] ={Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another planewith zeros
    
    dft(complexI, complexI);            // this way the result may fit inthe source matrix
    
    // compute the magnitude and switch tologarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 +Im(DFT(I))^2))
    split(complexI, planes);                   // planes[0] = Re(DFT(I),planes[1] = Im(DFT(I))
    magnitude(planes[0], planes[1], planes[0]);//planes[0] = magnitude
    Mat magI = planes[0];
    
    magI += Scalar::all(1);                    // switch to logarithmicscale
    log(magI, magI);
    
    // crop the spectrum, if it has an oddnumber of rows or columns
    magI = magI(Rect(0, 0, magI.cols & -2,magI.rows & -2));
    
    // rearrange the quadrants of Fourierimage  so that the origin is at the imagecenter
    int cx = magI.cols/2;
    int cy = magI.rows/2;
    
    Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
    Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
    Mat q3(magI, Rect(cx, cy, cx, cy)); //Bottom-Right
    
    Mat tmp;                           // swap quadrants(Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    
    q1.copyTo(tmp);                    // swap quadrant (Top-Rightwith Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);
    
    normalize(magI, magI, 0, 1, CV_MINMAX);// Transform the matrix with float values into a
    // viewable image form (float betweenvalues 0 and 1).
    output=magI;
    return 0;
}
