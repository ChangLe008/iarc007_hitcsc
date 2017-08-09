#include <ros/ros.h>

#include <sys/time.h>
#include <unistd.h>
#include <time.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <cv.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>

#include <dji_sdk/Velocity.h>
#include <iarc/raw_position.h>

using namespace std;
using namespace cv;

vector<float> VelocityX;
vector<float> VelocityY;
vector<float> VelocityZ;

vector<uint64> TimeStamp;

ros::Subscriber velocity_sub;

ros::Publisher position_pub;

void velocity_callback(const dji_sdk::Velocity& g_vo)
{
    VelocityX.push_back(g_vo.vx);
    VelocityY.push_back(g_vo.vy);
    VelocityZ.push_back(g_vo.vz);
    TimeStamp.push_back(g_vo.header.stamp.toNSec());
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"guidance_position");
    ros::NodeHandle my_node;
    
    velocity_sub = my_node.subscribe("/dji_sdk/velocity", 100, velocity_callback);
     
     double LastTime = 0;
     double CurrentTime = 0;
     double TimeInterval = 0;
     
     float CurrentVelocityX = 0;
     float CurrentVelocityY = 0;
     float CurrentVelocityZ = 0;
     
     Mat CurrentPosition = Mat(3, 1, CV_32FC1,Scalar(0));
     
     position_pub = my_node.advertise<iarc::raw_position>("/iarc/raw_position", 50);
     
     iarc::raw_position position;
     
     ros::Rate rate(50);
     
     while(ros::ok())
     {
	 ros::spinOnce();
	 rate.sleep();
	 
	 for(int i = 0; i < TimeStamp.size(); i++)
	 {
	     CurrentTime = (double)TimeStamp[i];
	     
	     TimeInterval = (CurrentTime - LastTime) / 1e9;
	     CurrentPosition.at<float>(0, 0) += CurrentVelocityX * TimeInterval;
	     CurrentPosition.at<float>(1, 0) += CurrentVelocityY * TimeInterval;
	     CurrentPosition.at<float>(2, 0) += CurrentVelocityZ * TimeInterval;
	     
	     CurrentVelocityX = VelocityX[i];
	     CurrentVelocityY = VelocityY[i];
	     CurrentVelocityZ = VelocityZ[i];
	     LastTime = CurrentTime;
	 }
	 position.x = CurrentPosition.at<float>(0,0);
	 position.y = CurrentPosition.at<float>(1, 0);
	 position.z = CurrentPosition.at<float>(2, 0);
	 position_pub.publish(position);
	 VelocityX.clear();
	 VelocityY.clear();
	 VelocityZ.clear();
	 TimeStamp.clear();
    }
    return 0;
}