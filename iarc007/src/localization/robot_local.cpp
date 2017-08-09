// Farneback dense optical flow calculate and show in Munsell system of colors  
// Author : Zouxy  
// Date   : 2013-3-15  
// HomePage : http://blog.csdn.net/zouxy09  
// Email  : zouxy09@qq.com  
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
#include <dji_sdk/Acceleration.h>

#include <localization/localization.h>
#include <localization/iposInfo.h>

using namespace cv;
using namespace std;
using namespace message_filters;

int main(int argc,char** argv)
{
	ros::init(argc,argv,"robot_local");
	ros::NodeHandle nh;

	std::string cam_topic;
	nh.getParam("/robot_local/cam_topic",cam_topic);

	float fx,fy,cx,cy;
	nh.getParam("/robot_local/cam_fx",fx);
	nh.getParam("/robot_local/cam_fy",fy);
	nh.getParam("/robot_local/cam_cx",cx);
	nh.getParam("/robot_local/cam_cy",cy);

	iposInfo robotLocalization(fx,fy,cx,cy);

	message_filters::Subscriber<sensor_msgs::Image> mono_camera_sub(nh,cam_topic,1);
	message_filters::Subscriber<dji_sdk::AttitudeQuaternion> attitude_sub(nh,"/dji_sdk/attitude_quaternion",1);
	message_filters::Subscriber<dji_sdk::Acceleration> acc_sub(nh,"/dji_sdk/acceleration",1);
	message_filters::Subscriber<geometry_msgs::Vector3Stamped> pos_sub(nh,"/guidance/position",1);
	message_filters::Subscriber<geometry_msgs::Vector3Stamped> vo_sub(nh,"/guidance/velocity",1);
	message_filters::Subscriber<sensor_msgs::LaserScan> sonic_sub(nh,"/guidance/ultrasonic",1);

	typedef sync_policies::ApproximateTime<dji_sdk::Acceleration,dji_sdk::AttitudeQuaternion,sensor_msgs::LaserScan,\
	geometry_msgs::Vector3Stamped,geometry_msgs::Vector3Stamped> mySyncPolicy;

	Synchronizer<mySyncPolicy> sync(mySyncPolicy(10),acc_sub,attitude_sub,sonic_sub,pos_sub,vo_sub);

	sync.registerCallback(boost::bind(&iposInfo::callback,&robotLocalization,_1,_2,_3,_4,_5));
	mono_camera_sub.registerCallback(&iposInfo::image_callback,&robotLocalization);

	ofstream outfile;
	outfile.open("/home/hitcsc/position.txt");

	ros::Rate r(60);

	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}

	outfile.close();

	ROS_INFO("!!Process Ends!!");

	return 0;
}
