//
//  darkchannel.cpp
//  opencv_test
//
//  Created by xiaoruiqiao on 2017/9/3.
//  Copyright © 2017年 xiaoruiqiao. All rights reserved.
//

#include <stdio.h>
#include "dcp_core.h"




void CalcDarkChannel(Mat& darkchannel, Mat& brightchannel, Mat&input, int radius)
{
	int height = input.rows;
	int width = input.cols;
    
    
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
            
            int cur = 0;
            int min = 300;
			int max = 0;
            for (int m = st_row; m <= ed_row; m++)
            {
                for (int n = st_col; n <= ed_col; n++)
                {
                    for (int k = 0; k < 3; k++)
                    {
						cur = input.at<Vec3b>(m, n)[k];
                        if (cur < min) min = cur;
						if (cur > max) max = cur;
                    }
                }
            }
			darkchannel.at<uchar>(i, j) = min;
			brightchannel.at<uchar>(i, j) = max;
            
        }
    }
}
