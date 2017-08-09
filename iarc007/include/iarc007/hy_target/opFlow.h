#ifndef OPFLOW_H_
#define OPFLOW_H_

// Farneback dense optical flow calculate and show in Munsell system of colors  
// Author : Zouxy  
// Date   : 2013-3-15  
// HomePage : http://blog.csdn.net/zouxy09  
// Email  : zouxy09@qq.com  

// API calcOpticalFlowFarneback() comes from OpenCV, and this  
// 2D dense optical flow algorithm from the following paper:  
// Gunnar Farneback. "Two-Frame Motion Estimation Based on Polynomial Expansion".  
// And the OpenCV source code locate in ..\opencv2.4.3\modules\video\src\optflowgf.cpp  
// Color encoding of flow vectors from:  
// http://members.shaw.ca/quadibloc/other/colint.htm  
// This code is modified from:  
// http://vision.middlebury.edu/flow/data/  

#include <ros/ros.h>

#include <iostream>  
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>

#include <dji_sdk/AttitudeQuaternion.h>

using namespace cv;  
using namespace std;  

#define UNKNOWN_FLOW_THRESH 1e9  

class opFlow
{
public:
	opFlow(float frame_rate,float fx,float fy,float cx,float cy)
	{
		FRAME_RATE = frame_rate;
		
		cu = cx;		
		cv = cy;

		fu = fx;
		fv = fy;
		
		vo_3d = Mat(3,1,CV_32FC1,Scalar::all(0));
		w_3d =  Mat(3,1,CV_32FC1,Scalar::all(0));
		
		d = 0.;

		updateFlag = false;
		time = 0.;

		vo.resize(2);
	}
	void voFromOpFlow();
	void callback(const sensor_msgs::ImageConstPtr& left_img,const dji_sdk::AttitudeQuaternionConstPtr& f_att,const \
	sensor_msgs::LaserScanConstPtr& g_ul,const geometry_msgs::Vector3StampedConstPtr& g_vo);
	void motionToColor(Mat flow,Mat& color);
	void voEstimate();
	void motionEstimate();
private:
	void makecolorwheel(vector<Scalar> &colorwheel);
public:
	/* file */	
	ofstream outfile;

	/* intrinsic parameters */
	float FRAME_RATE = 20.;
	float fu = 250;
	float fv = 250;
	float cu = 151;
	float cv = 116;

	/* color panel Munsell Color System*/
	vector<Scalar> colorwheel;
	/* color image of optical flow image */
	Mat color;

	/* source image */
	Mat img_sub;
	/* source image */
	Mat gray;
	/* previous image */
	Mat prevgray;

	/* optical flow image */
	Mat flow;

	//std::mutex mutexCall;

	/* data source update flag */
	bool updateFlag;

	/* timestamp */
	double prevTimeStamp;
	double nextTimeStamp;

	/* period (ms)*/
	double time;

	/* linear and angular velocity */
	Mat vo_3d;
	Mat w_3d;
	/* height */
	float d;

	//std::mutex mutexFuse;

	/* motion flow */
	Mat mFlowT;

	/* color image of translational flow */
	Mat colorT;

	/* linear vo from mono */
	vector<float> vo;

	//std::mutex mutexOp; 
};

#endif
