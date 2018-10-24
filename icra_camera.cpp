#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <math.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <math.h>
#include "serial.h"
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/types.h>
#define pi 3.14159265358979323846
using namespace std;
using namespace cv;
    Point left_center;
int mark=0,mark_y=0;
    vector<cv::Point2f> points2d;
vector<cv::Point2f> points2d2;
Rect rectangle1;
    void center_g(const vector<Point> contour,Point &center)
    {
        Moments mu;                                                 // Get the center of a contour
        mu = moments(contour,false);
        center.x=mu.m10/mu.m00;
        center.y=mu.m01/mu.m00;
    }
    void detect(Mat left,Point left_center1,vector<cv::Point2f> points2d1) {
        vector<Mat> split_left, split_right;
        split(left, split_left);
        Mat left_diff[2];
        left_diff[0] = split_left[2] - split_left[1];
        left_diff[1] = split_left[0] - split_left[1];
        Mat thresleft;
        vector<Vec4i> hierarchy_L;
        vector<Point> approx_left;
        thresleft = left_diff[0] + left_diff[1];
        threshold(thresleft, thresleft, 0, 255, THRESH_OTSU);
        vector<vector<Point> > contours_left, Triangle_left;
        findContours(thresleft, contours_left, hierarchy_L, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
        int tlsize = 0;
        for (int i = 0; i < contours_left.size(); i++) {
            approxPolyDP(contours_left[i], approx_left, 10, true);
            if (approx_left.size() == 4 && hierarchy_L[i][2] == -1 && hierarchy_L[i][3] != -1) {
                contourArea(contours_left[i]);
                if (contourArea(contours_left[i]) > tlsize) {
                    tlsize = contourArea(contours_left[i]);
                    Triangle_left.clear();
                    center_g(contours_left[i], left_center1);
                    Triangle_left.push_back(contours_left[i]);
                }
            }
        }
        for (int i = 0; i < Triangle_left.size(); i++)
        {
            if (tlsize == contourArea(Triangle_left[i]))
            {
                center_g(Triangle_left[i], left_center1);
                //rectangle1 = boundingRect(Triangle_left[i]);
		RotatedRect rectangle = minAreaRect(Triangle_left[i]);                
		Point2f vertex[4];
		mark_y=rectangle.center.x;
                rectangle.points(vertex);
		if (vertex[1].x+vertex[1].y>vertex[2].x+vertex[2].y)
		//cout<<rectangle.center.x<<endl;
{
		points2d1.push_back(vertex[1]);
                points2d1.push_back(vertex[2]);
                points2d1.push_back(vertex[3]);
                points2d1.push_back(vertex[0]);
		mark=1;
}
else
{		
		points2d1.push_back(vertex[0]);
                points2d1.push_back(vertex[1]);
                points2d1.push_back(vertex[2]);
                points2d1.push_back(vertex[3]);
		mark=2;
} 
               //points2d1.push_back(Point2f(rectangle1.x,rectangle1.y+rectangle1.height));
                //points2d1.push_back(Point2f(rectangle1.x,rectangle1.y));
                //points2d1.push_back(Point2f(rectangle1.x+rectangle1.width,rectangle1.y));
               // points2d1.push_back(Point2f(rectangle1.x+rectangle1.width,rectangle1.y+rectangle1.height));

                
            }
            points2d=points2d1;
            points2d1.clear();
//rectangle(left,rectangle1,Scalar(0,0,255),7,4,0);
circle(left, left_center1, 5, Scalar(0,255,0),-1);
                left_center=left_center1;
        drawContours(left, Triangle_left, -1, Scalar(255,0,0),3);
             }


    }
int main()
{
    float Ang_X, Ang_Y, Ang_Z;
    float X, Y, Z;
    int i = 0;
    float A[][3] = { {910.5424613696998, 0, 340.2146487423434}, {  0, 910.115106928623, 242.6898879812984 }, { 0, 0, 1 } };
    float B[] = { -0.4531410167046038, 0.7161829267340207, 0.0005925811033057943, 0.002163026084773896, -1.719829254648038};
    Mat rvecs(3, 1, CV_32F), tvecs(3, 1, CV_32F), cameraMatrix(3, 3, CV_32F), distCoeffs(1, 5, CV_32F), R(3, 3, CV_32FC1);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            cameraMatrix.at<float>(i, j) = A[i][j];
            R.at<float>(i, j) = 0;
        }
    for (int i = 0; i < 5;i++)
        distCoeffs.at<float>(0, i) = B[i];
            vector<cv::Point3f> point3d;
            point3d.push_back(Point3f(2.5,2.5, 0));
            point3d.push_back(Point3f(2.5,-2.5, 0));
            point3d.push_back(Point3f(-2.5,-2.5, 0));
            point3d.push_back(Point3f(-2.5,2.5, 0));
    VideoCapture capture1(0);
    if (!capture1.isOpened())
        return -1;
    for(;;)
    {
        Mat src1;
        capture1>>src1;
        if(src1.empty())
            break;
        detect(src1,left_center,points2d2);
        if(!points2d.empty())
        {
            solvePnP(point3d, points2d, cameraMatrix, distCoeffs, rvecs, tvecs);
            Rodrigues(rvecs, R);
cout<<points2d<<endl;
            //Ang_X = asin(R.at<double>(1, 0) / cos(asin(-R.at<double>(2, 0)))) / pi * 180;
            //Ang_Y = asin(-R.at<double>(2, 0)) / pi * 180;
            //Ang_Z = asin(R.at<double>(2, 1) / cos(asin(-R.at<double>(2, 0)))) / pi * 180;
//cout<<R<<endl;
//cout<<mark<<endl;
Ang_X=atan2(R.at<double>(2, 0),R.at<double>(0, 0))*180/pi;
Ang_Y=atan2(-R.at<double>(2, 0),sqrt(R.at<double>(2, 0)*R.at<double>(2, 0)+R.at<double>(2, 2)*R.at<double>(2, 2)))*180/pi;
Ang_Z=atan2(R.at<double>(1, 0),R.at<double>(0, 0))*180/pi;
            X = R.at<double>(0, 0) *point3d[22].x + R.at<double>(0, 1)  * point3d[22].y + R.at<double>(0,2)  * point3d[22].z + tvecs.at<double>(0,0);
            Y = R.at<double>(1, 0) *point3d[22].x + R.at<double>(1, 1)  * point3d[22].y + R.at<double>(1, 2)  * point3d[22].z + tvecs.at<double>(1, 0);
            Z = R.at<double>(2, 0) *point3d[22].x + R.at<double>(2, 1)  * point3d[22].y + R.at<double>(2, 2)  * point3d[22].z + tvecs.at<double>(2, 0);
if(Ang_X>0)
Ang_X=Ang_X-180;
else
Ang_X=Ang_X+180;
cout<<mark_y<<endl;
            cout<<"X:"<<X<<" 偏转角度为"<<Ang_X<<endl;
            cout<<"Y:"<<Y<<" 偏转角度为"<<Ang_Y<<endl;
            cout<<"Z:"<<Z<<" 偏转角度为"<<Ang_Z<<endl;
        }
        imshow("测试1",src1);
        if(waitKey(30)=='q')
            break;
    }
    return 0;
}
