#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#define pi 3.14159265358979323846
using namespace cv;
using namespace std;
 float Ang_X, Ang_Y, Ang_Z;
 float X, Y, Z;
 int i = 0;
Point center1,center2;
void center_g(const vector<Point> contour,Point &center)
{
    Moments mu;                                                 // Get the center of a contour
    mu = moments(contour,false);
    center.x=mu.m10/mu.m00;
    center.y=mu.m01/mu.m00;
}
void detect(Mat left,Point left_center1)
{
    vector<Mat> split_left;
    split(left,split_left);
    //split(right, split_right);
    Mat left_diff[2];
    left_diff[0] = split_left[2] - split_left[1];
    //right_diff[0] = split_right[2] - split_right[1];
    left_diff[1] = split_left[0] - split_left[1];
    //right_diff[1] = split_right[0] - split_right[1];
    Mat thresleft;
    vector<Vec4i> hierarchy_L;
    vector<Point> approx_left;
    thresleft = left_diff[0] + left_diff[1];
    //thresright = right_diff[0] + right_diff[1];
    threshold(thresleft,thresleft,0,255,THRESH_OTSU);
    //threshold(thresright,thresright,0,255,THRESH_OTSU);
    vector<vector<Point> > contours_left,Triangle_left;
    //Point left_center,right_center;(原来的)
    findContours(thresleft, contours_left,hierarchy_L,CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    //findContours(thresright, contours_right,hierarchy_R,CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    int tlsize = 0;
    for(int i=0;i<contours_left.size();i++){
        approxPolyDP(contours_left[i], approx_left, 10, true);
        if(approx_left.size() == 4 && hierarchy_L[i][2] == -1 && hierarchy_L[i][3] != -1){
            contourArea(contours_left[i]);
            if(contourArea(contours_left[i]) > tlsize){
                tlsize = contourArea(contours_left[i]);
                Triangle_left.clear();
                center_g(contours_left[i], left_center1);
                Triangle_left.push_back(contours_left[i]);
            }
        }
    }
    for(int i=0;i<Triangle_left.size();i++){
        if(tlsize==contourArea(Triangle_left[i])){
            center_g(Triangle_left[i], left_center1);
        }
    }
    {
        circle(left, left_center1, 5, Scalar(0,255,0),-1);
        center1=left_center1;
    }
    int trsize = 0;
    drawContours(left, Triangle_left, -1, Scalar(255,0,0),3);
}
int main() 
{
VideoCapture cap(1);
Mat src,src1,src2;
float A[][3] = { { 1340.315237021477, 0, 670.1301710259809}, { 0, 1343.875408835837, 336.7952602746108}, { 0, 0, 1 } };
    float B[] = {-0.4345844165904783, 0.2141409657022983, 0.004732508929288367, -0.0001699523459121179, 0.7269313399510783};
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
            point3d.push_back(Point3f(-3.6,-6.6, 0));//centmeter
            point3d.push_back(Point3f(-3.6,6.6, 0));
            point3d.push_back(Point3f(3.6,-6.6, 0));
            point3d.push_back(Point3f(3.6,6.6, 0));
vector<cv::Point2f> points2d;

Rect box,box_result;
while(10)
{
	cap>>src;
 	detect(src,center1);
imshow("src",src);
	/*Mat element=getStructuringElement(MORPH_RECT,Size(10,10));
		morphologyEx(src,src,MORPH_CLOSE,element);
		cvtColor(src,src1,COLOR_RGB2GRAY);
		threshold(src1,src1,30,255,THRESH_BINARY);
		Canny(src1,src2,3,9,3);
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(src2,contours,hierarchy,RETR_LIST,CHAIN_APPROX_NONE);
		if (contours.empty())
			cout<<"no charactisc"<<endl;
			//return 1;
		for(int cc=0;cc<contours.size();cc++)
		{
			box=boundingRect(contours[cc]);
			if (box.area()<5000||box.width<box.height/2||box.width>box.height/1.5)
				continue;
			rectangle(src,box,Scalar(255,0,0),3);
			box_result=box;
			cout<<"box.x="<<box.x<<"box.y="<<box.y<<"width="<<box.width<<"height="<<box.height<<endl;
			break;
		}
		imshow("src",src);
		imshow("src1",src1);
		imshow("src2",src2);
		points2d.push_back(Point2f(box_result.x+box_result.width,box_result.y));
                    points2d.push_back(Point2f(box_result.x+box_result.width,box_result.y+box_result.height));
                    points2d.push_back(Point2f(box_result.x,box_result.y));
                    points2d.push_back(Point2f(box_result.x,box_result.y+box_result.height));

                    for(int cc=0;cc<points2d.size();cc++)
		{
			cout<<points2d[cc]<<endl;
		}
 //三种方法求解
	 solvePnP(point3d, points2d, cameraMatrix, distCoeffs, rvecs, tvecs);
                    Rodrigues(rvecs, R);
                    Ang_X = asin(R.at<double>(1, 0) / cos(asin(-R.at<double>(2, 0)))) / pi * 180;
                    Ang_Y = asin(-R.at<double>(2, 0)) / pi * 180;
                    Ang_Z = asin(R.at<double>(2, 1) / cos(asin(-R.at<double>(2, 0)))) / pi * 180;
                    X = R.at<double>(0, 0) *point3d[22].x + R.at<double>(0, 1)  * point3d[22].y + R.at<double>(0,2)  * point3d[22].z + tvecs.at<double>(0,0);
                    Y = R.at<double>(1, 0) *point3d[22].x + R.at<double>(1, 1)  * point3d[22].y + R.at<double>(1, 2)  * point3d[22].z + tvecs.at<double>(1, 0);
                    Z = R.at<double>(2, 0) *point3d[22].x + R.at<double>(2, 1)  * point3d[22].y + R.at<double>(2, 2)  * point3d[22].z + tvecs.at<double>(2, 0);
                    cout<<"X:"<<X<<" 偏转角度为"<<Ang_X<<endl;
                    cout<<"Y:"<<Y<<" 偏转角度为"<<Ang_Y<<endl;
                    cout<<"Z:"<<Z<<" 偏转角度为"<<Ang_Z<<endl;
                    	vector<cv::Point2f>::iterator it=points2d.begin();
	while( it<points2d.end()) 
   	 {
		points2d.erase(it);

	}
	cout<<points2d.size()<<endl;
	//double tx=Tvec.ptr<double>(0)[0];
	//double ty=Tvec.ptr<double>(0)[1];
	//double tz=Tvec.ptr<double>(0)[2];
	cout<<"x="<<tvecs.at<double>(0,0)<<"y="<<tvecs.at<double>(1, 0)<<"z="<<tvecs.at<double>(2, 0)<<endl;
*/
	if (waitKey(10)==27)
	return 1;
}
return 0;
}
