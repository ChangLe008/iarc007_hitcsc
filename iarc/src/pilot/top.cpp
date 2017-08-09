#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <sys/time.h>
#include <fstream>
#include <geometry_msgs/Vector3Stamped.h>
#include <iarc/pilot/control.h>
#include <iarc/pilot/subscribe.h>
#include <iarc/object.h>
#include "iarc/pilot/rrt.h"
#include <visualization_msgs/Marker.h>
#include <random>
#include <iarc/path.h>
using namespace std;


float x = 0.;
float y = 0.;


extern CvMat *att_init;
extern CvMat *R_init;
extern CvMat *R_init_i;

extern CvMat *q_right_now;
extern CvMat *att_right_now;
extern CvMat *R_now_i;

extern CvMat *q;
extern CvMat *atti;

extern CvMat *q_init;

ros::Publisher rviz_pub;
ros::Publisher target_pub;
ros::Subscriber guidance_velocity;
ros::Subscriber object_subscriber;
iarc::object target;
iarc::object object_pos_dynamic;

visualization_msgs::Marker quad_marker;
visualization_msgs::Marker target_marker;
iarc::path path_rrt;

int map[40][30];
int route[2][20]={0};
rrt r;



int main(int argc, char **argv)
{

    int main_operate_code = 0;
    int temp32;
    bool valid_flag = false;
    bool err_flag = false;
    int timer;
	double time_0 = tic();

	ofstream init_log("/home/hitcsc/catkin_ws/log/iarc/lidar.txt");
	if(init_log.is_open())
	{
		cout << "Log initation success"<< endl;
	}
	else
	{
		cout << "Log error"<< endl;
        return 0;
	}

	rrt pathplan;

    ros::init(argc, argv, "top");
    ROS_INFO("top test");
    ros::NodeHandle top;
	visualization_msgs::Marker line_maker;

    LEODrone* top_drone=new LEODrone(top);
	rviz_pub = top.advertise<visualization_msgs::Marker>("/rviz/quad", 1000);
	target_pub = top.advertise<visualization_msgs::Marker>("/rviz/target", 1000);
	int _rate=10;
	ros::Rate rate(_rate);
	while(ros::ok())
	{		
		int obstacle_num	= top_drone->obstacle_dynamic.num;
		top_drone->obstacle_dynamic.o_x.resize(obstacle_num);
		top_drone->obstacle_dynamic.o_y.resize(obstacle_num);
		// init_log << obstacle_num<<"\t";
		// for(int i=0;i<obstacle_num;i++)
		// {
		// 	cout << top_drone->obstacle_dynamic.o_x[i] <<"\t"<< top_drone->obstacle_dynamic.o_y[i]<<"\t";
		// 	init_log << top_drone->obstacle_dynamic.o_x[i] <<"\t"<< top_drone->obstacle_dynamic.o_y[i]<<"\t";
		// 	if(i==obstacle_num-1)
		// 	{
		// 		cout << endl;
		// 		init_log << endl;
		// 	}
		// }
		// if(obstacle_num==0)
		// {
		// 	cout << "0"<<endl;
		// 	init_log <<"0"<<endl;  
		// }
		ros::spinOnce();
		rate.sleep();
	}
	init_log.close();
	return 0;
}
