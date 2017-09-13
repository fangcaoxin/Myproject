//
//  test2.cpp
//  opencv_test
//
//  Created by xiaoruiqiao on 16/2/26.
//  Copyright (c) 2016年 xiaoruiqiao. All rights reserved.
//

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace cv;
using namespace std;

//struct Rectangle
//{
    //int x;
    //int y;
    //int width;
  //  int height;
//};
Rect rec1,rec2;
struct slope_region
{
    double slope_min;
    double slope_max;
};

slope_region slope(Rect&rec1, Rect&rec2)
{
    Point p1[4],p2[4];
    p1[0].x = rec1.x;
    p1[0].y = rec1.y;
    p1[1].x = rec1.x + rec1.width;
    p1[1].y = rec1.y;
    p1[2].x = rec1.x;
    p1[2].y = rec1.y + rec1.height;
    p1[3].x = rec1.x + rec1.width;
    p1[3].y = rec1.y + rec1.height;
    
    p2[0].x = rec2.x;
    p2[0].y = rec2.y;
    p2[1].x = rec2.x + rec2.width;
    p2[1].y = rec2.y;
    p2[2].x = rec2.x;
    p2[2].y = rec2.y + rec2.height;
    p2[3].x = rec2.x + rec2.width;
    p2[3].y = rec2.y + rec2.height;
    vector<double> slopes;
    slope_region sr;
    for(int i = 0;i<4;i++)
    {
        for (int j = 0;j<4;j++)
        {
            double  slope;
            if (p1[i].x == p2[j].x) slope = std::numeric_limits<double>::infinity();
            else slope = 1.0*(p1[i].y - p2[j].y)/(p1[i].x -p2[j].x);
            slopes.push_back(slope);
        }
    sr.slope_max = *max_element(slopes.begin(),slopes.end());
    sr.slope_min = *min_element(slopes.begin(), slopes.end());
    }
    
    //cout << sr.slope_max <<" ," << sr.slope_min << endl;
    return sr;
}
    
bool intersection(slope_region&r1,slope_region&r2)
{
    if (r1.slope_min > r2.slope_max || r2.slope_min > r1.slope_max) return false;
    else return true;
}

    
class solution
{
    public:
    int count(vector<Rect>&rectangle)
    {
        int t = rectangle.size();
       
        vector<vector<slope_region>> slopesaveall;
        slopesaveall.resize(t-1);
        slope_region sr;
        for (int i = 0; i< t-1; i++)
        {
            for(int j = i+1; j < t;j++)
            {
            sr= slope(rectangle[i],rectangle[j]);
            slopesaveall.at(i).push_back(sr);
            }
        }
       // cout<<endl;
        
        int count;
        vector<int> num;
        bool find=true; short int f=0;
        int max_num(0);
        
        for (int s = 0;s< slopesaveall.size();s++)
        {
            for(int t = 0;t<slopesaveall[s].size();t++)
            {
                //for(int m = 0;m < slopesaveall[t+1].size();m++)
                //{
                    //find = intersection(slopesaveall[s][t],slopesaveall[t+1][m]);
                    //while (find)
                    //{
                        //s = m;
                        //count++;
                        //find=false;
                    //}
                    //num.push_back(count);
                
                while (find)
                {
                    f=slopesaveall[t+1].size();
                    bool set=false;
                    for(int m = 0;m < f;m++)
                    {
                        set = intersection(slopesaveall[s][t],slopesaveall[t+1][m]);
                        f=slopesaveall[m+1].size();
                        count++;
                        if(set==true)continue;
                    }
                }
                
            }
        }
        
        max_num = *max_element(num.begin(), num.end());
        return max_num;
    }
};

int main()
{
    //int data[10]={2,3,0,3,2,5,1,3,4,2};
    
    
    vector<Rect> rectangle;
    int rectangle_data[6][4] = {{2,1,4,3},{1,10,1,3},{5,7,5,4},{8,8,3,2},{13,4,3,1},{17,1,1,14}};
    for(int i=0;i<6;i++)
    {
        Rect tmp;
        tmp.x=rectangle_data[i][0];
        tmp.y=rectangle_data[i][1];
        tmp.width=rectangle_data[i][2];
        tmp.height=rectangle_data[i][3];
        
        rectangle.push_back(tmp);
    }
    solution sherry;
    int result= sherry.count(rectangle);
    return result;
}