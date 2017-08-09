// Farneback dense optical flow calculate and show in Munsell system of colors  
// Author : Zouxy  
// Date   : 2013-3-15  
// HomePage : http://blog.csdn.net/zouxy09  
// Email  : zouxy09@qq.com  
  
// API calcOpticalFlowFarneback() comes from OpenCV, and this  
// 2D dense optical flow algorithm from the following paper:  
// Gunnar Farneback. "Two-Frame Motion Estimation Based on Polynomial Expansion".  
// And the OpenCV source code locate in ..\opencv2.4.3\modules\video\src\optflowgf.cpp  
      

#include <iostream>
#include <iomanip>
#include <string>
 
#include <ros/ros.h>

#include <opencv2/opencv.hpp>  
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

using namespace cv;  
using namespace std;  

#define UNKNOWN_FLOW_THRESH 1e9  
  
Mat img_sub;

ros::Subscriber mono_video_sub;

/* Mono Video */
void mono_sub_callback(sensor_msgs::Image img_msg)
{ 
	cv_bridge::CvImagePtr img_bridge_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);       
	img_bridge_ptr->image.copyTo(img_sub);       
}

// Color encoding of flow vectors from:  
// http://members.shaw.ca/quadibloc/other/colint.htm  
// This code is modified from:  
// http://vision.middlebury.edu/flow/data/  
void makecolorwheel(vector<Scalar> &colorwheel)  
{  
    int RY = 15;  
    int YG = 6;  
    int GC = 4;  
    int CB = 11;  
    int BM = 13;  
    int MR = 6;  
  
    int i;  
  
    for (i = 0; i < RY; i++) colorwheel.push_back(Scalar(255,       255*i/RY,     0));  
    for (i = 0; i < YG; i++) colorwheel.push_back(Scalar(255-255*i/YG, 255,       0));  
    for (i = 0; i < GC; i++) colorwheel.push_back(Scalar(0,         255,      255*i/GC));  
    for (i = 0; i < CB; i++) colorwheel.push_back(Scalar(0,         255-255*i/CB, 255));  
    for (i = 0; i < BM; i++) colorwheel.push_back(Scalar(255*i/BM,      0,        255));  
    for (i = 0; i < MR; i++) colorwheel.push_back(Scalar(255,       0,        255-255*i/MR));  
}  
  
void motionToColor(Mat flow, Mat &color)  
{  
    if (color.empty())  
        color.create(flow.rows, flow.cols, CV_8UC3);  
  
    static vector<Scalar> colorwheel; //Scalar r,g,b  
    if (colorwheel.empty())  
        makecolorwheel(colorwheel);  
  
    // determine motion range:  
    float maxrad = -1;  
  
    // Find max flow to normalize fx and fy  
    for (int i= 0; i < flow.rows; ++i)   
    {  
        for (int j = 0; j < flow.cols; ++j)   
        {  
            Vec2f flow_at_point = flow.at<Vec2f>(i, j);  
            float fx = flow_at_point[0];  
            float fy = flow_at_point[1];  
            if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
                continue;  
            float rad = sqrt(fx * fx + fy * fy);  
            maxrad = maxrad > rad ? maxrad : rad;  
        }  
    }  
  
    for (int i= 0; i < flow.rows; ++i)   
    {  
        for (int j = 0; j < flow.cols; ++j)   
        {  
            uchar *data = color.data + color.step[0] * i + color.step[1] * j;  
            Vec2f flow_at_point = flow.at<Vec2f>(i, j);  
  
            float fx = flow_at_point[0] / maxrad;  
            float fy = flow_at_point[1] / maxrad;  
            if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
            {  
                data[0] = data[1] = data[2] = 0;  
                continue;  
            }  
            float rad = sqrt(fx * fx + fy * fy);  
  
            float angle = atan2(-fy, -fx) / CV_PI;  
            float fk = (angle + 1.0) / 2.0 * (colorwheel.size()-1);  
            int k0 = (int)fk;  
            int k1 = (k0 + 1) % colorwheel.size();  
            float f = fk - k0;  
            //f = 0; // uncomment to see original color wheel  
  
            for (int b = 0; b < 3; b++)   
            {  
                float col0 = colorwheel[k0][b] / 255.0;  
                float col1 = colorwheel[k1][b] / 255.0;  
                float col = (1 - f) * col0 + f * col1;  
                if (rad <= 1)  
                    col = 1 - rad * (1 - col); // increase saturation with radius  
                else  
                    col *= .75; // out of range  
                data[2 - b] = (int)(255.0 * col);  
            }  
        }  
    }  
}  
  
int main(int argc, char** argv)
{  
	ros::init(argc,argv,"optical_flow_detection");
	ros::NodeHandle my_node;
	
	/* Topic: mono_video -- Type: sensor_msgs::Image */
	mono_video_sub = my_node.subscribe<sensor_msgs::Image>("/iarc007/mono_video",2,mono_sub_callback);
	
	ros::Rate rate(30);
	
    VideoCapture cap;  
    cap.open(7);  
    //cap.open("test_02.wmv");  
  
    if( !cap.isOpened() )  
        return -1;  
  
    Mat prevgray, gray, flow, cflow, frame;  
    namedWindow("flow", 1);  
  
    Mat motion2color;  
  
    while(ros::ok())  
    {  
        double t = (double)cvGetTickCount();  
  
        cap >> frame;  
        cvtColor(frame, gray, CV_BGR2GRAY);  
        imshow("original", frame);  
  
        if( prevgray.data )  
        {  
            calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);  
            motionToColor(flow, motion2color);  
            imshow("flow", motion2color);  
        }  
        if(waitKey(10)>=0)  
            break;  
        std::swap(prevgray, gray);  
  
        t = (double)cvGetTickCount() - t;  
        cout << "cost time: " << t / ((double)cvGetTickFrequency()*1000.) << endl;  
    }  
    
    return 0;  
}  
/*
int main(int argc, char** argv)
{

	ros::init(argc,argv,"mono_video_sub");
	ros::NodeHandle my_node;
	
	/* Topic: mono_video -- Type: sensor_msgs::Image */
/*	mono_video_sub = my_node.subscribe<sensor_msgs::Image>("/iarc007/mono_video",2,mono_sub_callback);
	
	ros::Rate rate(30);
	
	while(ros::ok())
	{
		if(img_sub.data)//确定有图像
		{
			//imshow("mono_video_sub",img_sub);
			*img = IplImage(img_sub);
	
			
		}	
	}
	
	return 0;	
}
*/
