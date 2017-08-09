#ifndef IPOSINFO_H_
#define IPOSINFO_H_

#include <ros/ros.h>

#include <iostream>  
#include <fstream>
#include <vector>
#include <string>
#include <algorithm> 
#include <cassert>
#include <thread>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>

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

using namespace cv;  
using namespace std;  
using namespace Eigen;

#define UNKNOWN_FLOW_THRESH 1e9  

class iposInfo
{
public:
	iposInfo(float fx,float fy,float cx,float cy)
	{
		mmCamera << fx,0.,cx,0.,fy,cy,0.,0.,1.;
		updateFlag = false;
		mvTSpeed = mvTPos = Vector3d::Zero();
		mvTPos(2) = -2.0;
		mmWPos << 0.,1.,0.,-1.,0.,0.,0.,0.,1.;
		mapLoading("/home/hitcsc/catkin_ws/src/iarc007/doc/map.txt");
	}

	void callback(const dji_sdk::AccelerationConstPtr& f_acc,const dji_sdk::AttitudeQuaternionConstPtr& f_att,const \
	sensor_msgs::LaserScanConstPtr& g_ul,const geometry_msgs::Vector3StampedConstPtr& g_pos,const \
	geometry_msgs::Vector3StampedConstPtr& g_vo);
	
	void image_callback(const sensor_msgs::ImageConstPtr& left_img);

	void featursSorting(float maxCombineDistance);
	void featureMerging(float maxCombineDistance);
	vector<vector<std::pair<float,float>>> featureQuad(float maxCombineDistance);

	void mapLoading(string s);
	void mapProjection(Matrix3d mmHomography);
	std::pair<float,float> mapReprojection(Matrix3d mmHomography,std::pair<float,float>& Ptsrc);
	void mapSorting(float maxCombineDistance);
	void mapMerging(float maxCombineDistance);
	vector<vector<std::pair<float,float>>> mapQuad(float maxCombineDistance);

	void PoseEstimation(std::vector<cv::Point3f> objPts,std::vector<cv::Point2f> imgPts,int method=1);

	void motionEstimate();
private:
	//! intrinsic parameters
	Matrix3d mmCamera;
	//! source image
	Mat gray;
	//! previous image
	Mat prevgray;

	Matrix3d m3;

	//! data source update flag
	bool updateFlag;

	//! linear and angular velocity and position of quadrotor
	Vector3d mvTSpeed;
	Vector3d mvTPos; //! camera pos
	Matrix3d mmWPos; //! from camera to geography
	Matrix3d mmWPosInv;

	//! liner and angular velocity and position measured by Guidance
	Vector3d mvTSpeedG;
	Vector3d mvTAccG;
	Vector3d mvWSpeedG;
	Vector3d mvTPosG; //!
	Matrix3d mmWPosG; //! from body to geography

	float mvTHeightG;
	
	//! feature
	vector<std::pair<float,float>> points;
	vector<std::pair<float,float>> pointsFilter;
	//! map
	vector<std::pair<float,float>> map;
	//! map pos in image and in world
	vector<std::pair<float,float>> mapPoints;
	vector<std::pair<float,float>> mapPointsInPixels;
	//vector<std::pair<float,float>> mapPointsInWorld;
};

#endif
