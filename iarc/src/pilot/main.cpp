#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include "dji_sdk/LocalPosition.h"
#include "dji_sdk/AttitudeQuaternion.h"
#include <geometry_msgs/Vector3Stamped.h>
#include "Eigen/Dense"
#include "iarc/pilot/control.h"
#include "iarc/pilot/leo_math.h"
#include "iarc/pilot/parameter.h"
//#include "iarc/pilot/subscribe.h"


using namespace Eigen;
using namespace std;
using namespace DJI::onboardSDK;


ros::Subscriber guidance_height;
ros::Subscriber l_position;
ros::Subscriber g_position;
ros::Subscriber quaternion_subscriber;
ros::Subscriber guidance_velocity;


//geometry_msgs::Vector3Stamped v_guidance;
//sensor_msgs::LaserScan g_height;
//dji_sdk::LocalPosition local_position;
//dji_sdk::AttitudeQuaternion attitude_quaternion;



double ranger;
double dist = .0;

extern CvMat *q_init;
extern CvMat *att_init;




/*
double OptimalController(double a0,double k0,double tgo,double x1,double x2,double xt1,double xt2,double vm)
{
	#define sqr3(a) ((a)*(a)*(a))
	#define sqr2(a) ((a)*(a))
	#define e(a) exp((a))
    double ans;
    MatrixXd A(4,4);
    MatrixXd B(4,1);
    MatrixXd C(1,4);
    MatrixXd res(1,1);
    A(0,0)=2.0; A(0,1)=-1.0; A(0,2)=4*sqr3(a0); A(0,3)=-4*sqr2(a0);
    A(1,0)=-2.0; A(1,1)=-1.0; A(1,2)=0.0; A(1,3)=4*sqr2(a0);
    A(2,0)=2.0-2*a0*tgo; A(2,1)=-e(a0*tgo); A(2,2)=4*sqr3(a0); A(2,3)=-4*sqr2(a0)*e(-a0*tgo);
    A(3,0)=-2.0; A(3,1)=-e(a0*tgo); A(3,2)=0.0; A(3,3)=4*sqr2(a0)*e(-a0*tgo);
    B(0,0)=4*sqr3(a0)*x1;
    B(1,0)=4*sqr2(a0)*x2;
    B(2,0)=4*sqr3(a0)*xt1;
    B(3,0)=4*sqr2(a0)*xt2;
    C(0,0)=-1.0/2.0/a0; C(0,1)=-1.0/2.0/a0; C(0,2)=0.0; C(0,3)=0.0;
	res=1/k0*C*A.inverse()*B;

     ans=res(0,0);
	 ans=isnan(ans)?0.0:ans>vm?vm:ans<-vm?-vm:ans;
      return ans;

#undef sqr3
#undef sqr2
#undef e
}
*/



int main(int argc, char **argv)
{

    int main_operate_code = 0;
    int temp32;
    int timer=0;
	double time_0 = tic();
	ofstream init_log("/home/hitcsc/catkin_ws/log/test_client/descend.txt");
	
    ros::init(argc, argv, "attitude_control_test");
    ROS_INFO("IARC pilot test");
    ros::NodeHandle nh;

	DJIDrone* drone		= new DJIDrone(nh);
	
	/*****
	1. ativation and take off

	****/
	while(drone->activation==0)
	{
		ros::spinOnce();
        sleep(1);
		drone->activate();
		ROS_INFO("activation error %d",drone->activation);
	}
	ROS_INFO("activation success");

	while((abs(drone->rc_channels.mode+8000.0)<1)||abs(drone->rc_channels.mode)<1)//F or P mode
	{
		ros::spinOnce();
		drone->request_sdk_permission_control();
		ROS_INFO("Request permission error, quad mode is %f",drone->rc_channels.mode);
		sleep(1);
	}
	
	ROS_INFO("Control acessed");
/*
	if(drone->takeoff())
	{
		ROS_INFO("Takeoff command is sent");
	}
	else
	{
		ROS_INFO("Unable to take off");
	}
	sleep(5);*/

	sleep(2);
	


	float angle_x=0.0,angle_y=0.0;
	float expect_vx=0.0,expect_vy=0.0,velocity_h=0.0;
	int a=0;
	int b=0;
	float c=0.0;
	float v_x=0.0,v_y=0.0,v_h=0.0;
	int freq =10;
	int _rate=50;
	int descend_flag=0;
	double x_error=1.,x_error_1=0.,x_error_2;
	double h_error=0.,h_error_1=0.,h_error_2;
	double height_error=0.;
	float Error_vx =0, Error_vy = 0;
	drone-> release_sdk_permission_control();
	// 2. goto contorl function
	control();
/////////////////////////////////////////////////////////////////////////////
 

/////////////////////////////////////////////////////////////////////////////
	init_log.close();
	
	return 0;
}

