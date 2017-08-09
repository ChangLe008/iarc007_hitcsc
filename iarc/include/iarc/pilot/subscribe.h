#include <dji_sdk/dji_sdk.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <string>
#include <geometry_msgs/TransformStamped.h>     //IMU
#include <geometry_msgs/Vector3Stamped.h>       //velocity
#include <sensor_msgs/LaserScan.h>              //obstacle distance && ultrasonic
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iarc/model_choose.h>
#include "iarc/pilot/leo_math.h"
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/GlobalPosition.h>
#include <iarc/object.h>
#include <iarc/target.h>
#include <iarc/obstacle.h>
#include <iarc/target_full.h>
#include <iarc/position.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <network_client/Optitrack_data.h>
#ifndef _INIT_ATT
#define _INIT_ATT

using namespace cv;
using namespace std;



#endif

class LEODrone
{
private:
	ros::Subscriber ultrasonic_subscriber;
	ros::Subscriber acceleration_subscriber;
	ros::Subscriber attitude_quaternion_subscriber;
    ros::Subscriber rc_channels_subscriber;
    ros::Subscriber velocity_subscriber;
    ros::Subscriber activation_subscriber;
	ros::Subscriber sdk_permission_subscriber;
	ros::Subscriber time_stamp_subscriber;
	ros::Subscriber velocity_guidance_subscriber;
	ros::Subscriber l_position;
	ros::Subscriber g_position;
	ros::Subscriber target_subscriber;

	ros::Subscriber object_subscriber;
	ros::Subscriber obstacle_subscriber;
	ros::Subscriber position_sub;
	ros::Subscriber height_subscriber;
	ros::Subscriber Opti_pos;
	CvMat* distortion=(CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/Distortion.xml");

	float guidance_last = 0;
	float yaw = 0;
public:
	ros::Publisher model_choose_publisher;
	sensor_msgs::LaserScan ultrasonic;
	sensor_msgs::LaserScan sona;
	dji_sdk::Acceleration acceleration;
	dji_sdk::AttitudeQuaternion attitude_quaternion;
	dji_sdk::RCChannels rc_channels;
	dji_sdk::Velocity velocity;
    dji_sdk::TimeStamp time_stamp;
	geometry_msgs::Vector3Stamped velocity_guidance;	// this velocity is in arena coordinate
	dji_sdk::LocalPosition local_position;
	iarc::model_choose state_choose;
    bool sdk_permission_opened=true;
    bool activation = false;
	CvMat* intrinsic=(CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/Intrinsics.xml");
	Mat M=Mat(intrinsic,true);
	CvMat MM = M;

	iarc::obstacle obstacle_dynamic;
	iarc::obstacle obstacle_lidar;
	iarc::obstacle obstacle_vision;

	iarc::object object_pos_dynamic;
	iarc::object object_pos_history;
	iarc::object object_prediction;

	iarc::target_full target_dynamic;

	dji_sdk::GlobalPosition global_position;
	dji_sdk::GlobalPosition last_global_position;
	float last_height;
	iarc::position position_lf;

	network_client::Optitrack_data opti_pos;
private:
	    void ultrasonic_subscriber_callback(sensor_msgs::LaserScan g_ul);

	    void acceleration_subscriber_callback(dji_sdk::Acceleration acceleration);

		void attitude_quaternion_subscriber_callback(dji_sdk::AttitudeQuaternion attitude_quaternion);

		void rc_channels_subscriber_callback(dji_sdk::RCChannels rc_channels);

		void velocity_subscriber_callback(dji_sdk::Velocity velocity);

		void activation_subscriber_callback(std_msgs::UInt8 activation);

		void sdk_permission_subscriber_callback(std_msgs::UInt8 sdk_permission);

		void time_stamp_subscriber_callback(dji_sdk::TimeStamp time_stamp);

		void velocity_guidance_subscriber_callback(geometry_msgs::Vector3Stamped g_vo);

		void l_pose_callback(dji_sdk::LocalPosition posi);
		
		void kalman_filter(iarc::object object_pos);

		void target_callback(iarc::target target);

		CvPoint DistortionPoint(int x,int y,CvMat* distortion,CvMat* intrinsic);

		void solver(CvMat* R,CvMat* P,double h,int u,int v);

		void lidar_obstacle_callback(iarc::obstacle obs);

		void g_pose_callback(const dji_sdk::GlobalPosition g_pos);

		void position_callback(iarc::position position_lf);

		void opti_position_callback(network_client::Optitrack_data position);

		void height_callback(sensor_msgs::LaserScan ul);
		
public:
		void transformer(Point2f img_point, Point3d& world_point, Mat R, Mat M, double height);
	LEODrone(ros::NodeHandle& nh);
	~LEODrone();
};