#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sys/time.h>
#include<iarc007/jrh_pilot/pilotmain.h>
#include <iarc007/jrh_pilot/JRH_math.h>
#include <iarc007/jrh_pilot/control.h>
//#include <iarc007/jrh_pilot/subscribe.h>
#define PI 3.1416

extern FILE *jrh_q;

extern CvMat *q_init;
//extern CvMat *R_init;  //R_init是赛场坐标系和地理坐标系之间的转换
//extern CvMat *att_init;


using namespace DJI::onboardSDK;

CvMat *q_init=cvCreateMat(4,1,CV_32FC1);
CvMat *R_init=cvCreateMat(3,3,CV_32FC1);
CvMat *att_init=cvCreateMat(3,1,CV_32FC1);
FILE *yawwww_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/yawwww.txt","w");
FILE *jrh_q=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/Q_INIT.txt","r");
//FILE *ultrasonic_original=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/ultrasonic_original.txt","w");
int main(int argc, char **argv)
{

    int main_operate_code = 0;
    int temp32;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "pilotmain");
    ROS_INFO("pilotmain test");

    ros::NodeHandle nh;

    DJIDrone* drone = new DJIDrone(nh);
	while(drone->activation==0)
	{
		ros::spinOnce();
        sleep(1);
		drone->activate();
		ROS_INFO("active error    %d",drone->activation);
		//ROS_INFO("active error");
	}
	ROS_INFO("active success ok !!");
	while((abs(drone->rc_channels.mode+8000.0)<1)||abs(drone->rc_channels.mode)<1)//遥控未切换档位
	{
	ros::spinOnce();
	drone->request_sdk_permission_control();
	ROS_INFO("obtain  error !!! %f",drone->rc_channels.mode);
    sleep(1);
	}
	ROS_INFO("obtain control success!!!");

	drone->takeoff();
	sleep(8);
	ROS_INFO("takeoff command send ok!!!");
//now we should initialize the matrix form ground to body

///////////////////////////////////////////////////////////////
	if(jrh_q==NULL)
	{
				printf("File open failed ! \n");
				return 0;
	}

	float q0,q1,q2,q3;
	fscanf(jrh_q,"%f%f%f%f\n",&q0,&q1,&q2,&q3);
	fclose(jrh_q);

    cvmSet(q_init, 0, 0, q0);
	cvmSet(q_init, 1, 0, q1);
	cvmSet(q_init, 2, 0, q2);
	cvmSet(q_init, 3, 0, q3);
	Quaternion_To_Euler(q_init,att_init);
//printf("%f	%f	%f\n",cvmGet(att_init,0,0),cvmGet(att_init,1,0),cvmGet(att_init,2,0));
	Euler_To_Matrix(cvmGet(att_init,0,0),cvmGet(att_init,1,0),cvmGet(att_init,2,0),R_init);
	printf("Q q0:%f, q1:%f, q2:%f,q3:%f\n",q0,q1,q2,q3);

///////////////////////////////////////////////////////////////




	{

		control();

	}

	cvReleaseMat(&att_init);
	cvReleaseMat(&R_init);
	cvReleaseMat(&q_init);

    return 0;
}
