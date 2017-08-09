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
#include <thread>
#include <mutex>
#include <cstdlib>

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

#include "iarc007/hy_target/opFlow.h"

using namespace cv;  
using namespace std;  
using namespace message_filters;

int main(int argc, char** argv)  
{  
	ros::init(argc,argv,"optical_flow");
	ros::NodeHandle nh;

	std::string cam_topic;
	nh.getParam("/optical_flow/cam_topic",cam_topic);

	float fx,fy,cx,cy,rate;
	nh.getParam("/optical_flow/cam_fx",fx);
	nh.getParam("/optical_flow/cam_fy",fy);
	nh.getParam("/optical_flow/cam_cx",cx);
	nh.getParam("/optical_flow/cam_cy",cy);
	nh.getParam("/optical_flow/cam_rate",rate);

	opFlow vFlow(rate,fx,fy,cx,cy);

	std::cout << "rate " << rate << endl \
	     << "fx "   << fx << endl \
             << "fy "   << fy << endl \
             << "cx "   << cx << endl \
             << "cy "   << cy << endl \
		<< "topic_cam " << cam_topic << endl;

	message_filters::Subscriber<sensor_msgs::Image> mono_camera_sub(nh,cam_topic,1);
	message_filters::Subscriber<dji_sdk::AttitudeQuaternion> attitude_sub(nh,"/dji_sdk/attitude_quaternion",1);
	message_filters::Subscriber<geometry_msgs::Vector3Stamped> vo_sub(nh,"/guidance/velocity",1);
	message_filters::Subscriber<sensor_msgs::LaserScan> sonic_sub(nh,"/guidance/ultrasonic",1);

	typedef sync_policies::ApproximateTime<sensor_msgs::Image,dji_sdk::AttitudeQuaternion,sensor_msgs::LaserScan,\
	geometry_msgs::Vector3Stamped> mySyncPolicy;

	Synchronizer<mySyncPolicy> sync(mySyncPolicy(10),mono_camera_sub,attitude_sub,sonic_sub,vo_sub);

	sync.registerCallback(boost::bind(&opFlow::callback,&vFlow,_1,_2,_3,_4));	

	vFlow.outfile.open("/home/hitcsc/oflow.txt");
	
	//std::thread* thread_motion = new thread(&opFlow::motionEstimate,&vFlow);

	while(ros::ok())  
	{ 
		vFlow.motionEstimate();

		//vFlow.gray.copyTo(vFlow.colorT);
		if(vFlow.updateFlag){
			//for(int y = 0; y < vFlow.mFlowT.rows; y += 5){
				/*for(int x = 0; x < vFlow.mFlowT.cols; x += 5)
				{
					const Point2f& fxy = vFlow.mFlowT.at<Point2f>(y, x);
					line(vFlow.colorT, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
						 Scalar(255,255,255));
					circle(vFlow.colorT, Point(x,y), 2, Scalar(255,255,255), -1);
				}*/
			//}
		
			//imshow("graph",vFlow.colorT);
			//waitKey(1);  <---->
		}
		
		ros::spinOnce();
	}  
	
	ROS_INFO("System stop!");

	vFlow.outfile.close();

	//delete thread_motion;

	return 0;  
}  
