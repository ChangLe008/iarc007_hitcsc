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



/// target
float x = 0.;
float y = 0.;

/// target


// extern CvMat *R_init;
ros::Publisher marker_pub;
ros::Publisher rviz_pub;
ros::Publisher target_pub;


iarc::object target;
iarc::object object_pos_dynamic;

visualization_msgs::Marker target_maker;
visualization_msgs::Marker quad_maker;
visualization_msgs::Marker line_maker;

iarc::path path_rrt;

int map[40][30];
int route[2][20]={0};
rrt r;

int main(int argc, char **argv)
{
    float target_x=0.;
    float target_y=0.;
    float obs_x[4]={0,0,0,0};
    float obs_y[4]={0,0,0,0};
    int target_number = 0;


    int main_operate_code = 0;
    int temp32;
    bool valid_flag = false;
    bool err_flag = false;
    int timer;
	double time_0 = tic();

	ofstream init_log("/home/hitcsc/catkin_ws/log/iarc/simulation.txt");
    fstream cfg("/home/hitcsc/catkin_ws/src/iarc/cfg/simu.cfg");  

	if(init_log.is_open())
	{
		cout << "Log initation success"<< endl;
	}
	else
	{
		cout << "Log error"<< endl;
        return 0;
	}

    if(cfg.is_open())
    {
        cout<< "Configeration loaded"<<endl;
    }
    else
    {
        cout<< "CANNOT Load Configuration"<<endl;
    }
    cfg >> target_x;
    cfg >> target_y;
    
    while (!cfg.eof())  
    {  
          cfg >> obs_x[target_number];
          cfg >> obs_y[target_number];
          target_number++;
    }

 	cout << target_x <<"\t"<< target_y <<"\t"<< obs_x[0]<<"\t"<< obs_y[0]<<endl;
	rrt pathplan;

    ros::init(argc, argv, "top");
    ROS_INFO("top test");
    ros::NodeHandle top;
	



	rviz_pub    = top.advertise<visualization_msgs::Marker>("/rviz/quad", 1000);
	target_pub  = top.advertise<visualization_msgs::Marker>("/rviz/target", 1000);
	marker_pub  = top.advertise<visualization_msgs::Marker>("/rviz/obstacle", 10);
	int _rate=10;
	ros::Rate rate(_rate);
	target_maker.pose.position.x = 0;
	target_maker.pose.position.y = 0;
	while(ros::ok())
	{		
		int target_number = 1;
		target_maker.header.frame_id = "base_link";
		target_maker.header.stamp = ros::Time();
		target_maker.ns = "tartget";
		target_maker.id = 0;
		target_maker.type = visualization_msgs::Marker::CYLINDER;
		target_maker.action = 3;
		target_maker.action = visualization_msgs::Marker::ADD;
		
		if(target_number>=1)
		{
			target_maker.pose.position.x = x;
			target_maker.pose.position.y = y;
		}
		else
		{
			;
		}
		if(target_number>=1 )
		{
			
			// path_rrt=pathplan.output(target_x,target_y,obs_x,obs_y,target_number);
            path_rrt=pathplan.output(target_x,target_y,obs_x[0],obs_y[0]);
			if(path_rrt.found==1)
			{
				path_rrt.x.resize(path_rrt.len);
				path_rrt.y.resize(path_rrt.len);
				int fff = path_rrt.len;
				cout << "Path found! " << fff<<"\t"<< endl;
                //<<"\t"<< path_rrt.y[fff-1]<<"\t"<<path_rrt.y[fff-1]<<endl;
                for(int j=0;j<fff;j++)
                {
                    cout << (path_rrt.x[fff-1-j]-20)*0.1<<"\t";
                }
                cout << endl;
                for(int j=0;j<fff;j++)
                {
                    cout << (path_rrt.y[fff-1-j]-15)*0.1<<"\t";
                }
                cout << endl;
			}
			else
			{
				cout << "Path not found!"<< endl;
			}
		}
		else if(target_number>=1)
		{
			path_rrt.found = 1;
			path_rrt.len = 2;
			path_rrt.x.resize(2);
			path_rrt.y.resize(2);
			path_rrt.y[0] = 15;
			path_rrt.x[0] = 20;
			path_rrt.y[1] = y;
			path_rrt.x[1] = x;
			cout << "It's a direct go"<< endl;
		}
		else
		{
			cout << "no target" << endl;
		}


		target_maker.pose.position.z = .05;
		target_maker.pose.orientation.x = 0.0;
		target_maker.pose.orientation.y = 0.0;
		target_maker.pose.orientation.z = 0.0;
		target_maker.pose.orientation.w = 1.0;
		target_maker.scale.x = 0.15;
		target_maker.scale.y = 0.15;
		target_maker.scale.z = 0.10;
		target_maker.color.a = 1.0; // Don't forget to set the alpha!
		target_maker.color.r = .0;
		target_maker.color.g = 1.0;
		target_maker.color.b = 0.0;


		quad_maker.header.frame_id = "base_link";
		quad_maker.header.stamp = ros::Time();
		quad_maker.ns = "quad";
		quad_maker.id = 0;
		quad_maker.type = visualization_msgs::Marker::MESH_RESOURCE;
    	quad_maker.mesh_resource = "package://iarc/cfg/quad.dae";
		quad_maker.action = visualization_msgs::Marker::ADD;
		quad_maker.pose.position.x = 0.;
		quad_maker.pose.position.y = 0.;
		quad_maker.pose.position.z = .5;
		quad_maker.pose.orientation.x = 0.0;
		quad_maker.pose.orientation.y = 0.0;
		quad_maker.pose.orientation.z = 0.0;
		quad_maker.pose.orientation.w = 1.0;
		quad_maker.scale.x = 0.30;
		quad_maker.scale.y = 0.30;
		quad_maker.scale.z = 0.30;
		quad_maker.color.a = 1.0; // Don't forget to set the alpha!
		quad_maker.color.r = 1.0;
		quad_maker.color.g = 0.0;
		quad_maker.color.b = 0.0;

		target_pub.publish(quad_maker);
		rviz_pub.publish(target_maker);

		ros::spinOnce();
		rate.sleep();
	}
	init_log.close();
	return 0;
}