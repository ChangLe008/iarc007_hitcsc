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

// void object_subscriber_callback(iarc::object object_pos)
// {
// 	object_pos_dynamic	= object_pos;
// 	object_pos_dynamic.target_x.resize(20);
// 	object_pos_dynamic.target_y.resize(20);
// 	object_pos_dynamic.target_dir.resize(20);
// 	object_pos_dynamic.target_vx.resize(20);
// 	object_pos_dynamic.target_vy.resize(20);
// }



int main(int argc, char **argv)
{

    int main_operate_code = 0;
    int temp32;
    bool valid_flag = false;
    bool err_flag = false;
    int timer;
	double time_0 = tic();

	//ofstream init_log("/home/hitcsc/catkin_ws/log/iarc/kf.txt");
	ofstream init_log("/home/hitcsc/catkin_ws/src/tar_obs_py/dal.txt");
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
	// object_subscriber=top.subscribe<iarc::object>("/hy_object", 1, object_subscriber_callback);
	ros::Publisher marker_pub = top.advertise<visualization_msgs::Marker>("path", 10);
	int _rate=10;
	ros::Rate rate(_rate);
	// quad_marker.pose.position.x = 0;
	// quad_marker.pose.position.y = 0;
	while(ros::ok())
	{		
		// int target_number = top_drone->object_pos_dynamic.target_num; 
		//cout <<  top_drone->target_dynamic.target.target_num;
		int target_number = top_drone->target_dynamic.target.target_num;
		top_drone->target_dynamic.target_x.resize(target_number>2?target_number:2);
		top_drone->target_dynamic.target_y.resize(target_number>2?target_number:2);
		//cout << target_number;
		// top_drone->object_pos_dynamic.target_x.resize(2);
        // top_drone->object_pos_dynamic.target_y.resize(2);
        top_drone->obstacle_dynamic.o_x.resize(2);
		top_drone->obstacle_dynamic.o_y.resize(2);
		cout << target_number;
		quad_marker.header.frame_id = "base_link";
		quad_marker.header.stamp = ros::Time();
		quad_marker.ns = "tartget";
		quad_marker.id = 0;
		quad_marker.type = visualization_msgs::Marker::CYLINDER;
		quad_marker.action = 3;
		quad_marker.action = visualization_msgs::Marker::ADD;
		
		if(target_number>=1)
		{
			// x = 0.01*object_pos_dynamic.target_x[0];
			// y = 0.01*object_pos_dynamic.target_y[0];
			x = top_drone->target_dynamic.target_x[0];
			y = top_drone->target_dynamic.target_y[0];
			quad_marker.pose.position.x = x;
			quad_marker.pose.position.y = y;
			init_log<< x <<"\t"<<y<<endl;
		}
		else
		{
			;
		}
		path_rrt=pathplan.output(-0.3,1,0.5,0.5);
		cout << "Path not found!"<< endl;
		// if(path_rrt.found==1)
		// 	{
		// 		path_rrt.x.resize(path_rrt.len);
		// 		path_rrt.y.resize(path_rrt.len);
		// 		int fff = path_rrt.len;
		// 		cout << "Path found! " <<endl;// fff<<"\t"<< path_rrt.y[fff-1]<<"\t"<<path_rrt.y[fff-1]<<endl;
		// 		for(int j=0;j<fff;j++)
		// 		{
		// 			cout << path_rrt.x[j]<<",";
		// 		}
		// 		cout << endl;
		// 		for(int j=0;j<fff;j++)
		// 		{
		// 			cout << path_rrt.y[j]<<",";
		// 		}
		// 		cout << endl;
		// 		// geometry_msgs::Point p;
		// 		// for(int i=0;i<fff;i++)
		// 		// {
		// 		// 	p.x = (path_rrt.x[i]-20)*0.1;
		// 		// 	p.y = (path_rrt.y[i]-15)*0.1;
		// 		// 	p.z = -0.4/fff*i+0.5;
		// 		// 	line_maker.points.push_back(p);
		// 		// }
		// 	}
		// 	else
		// 	{
		// 		;
		// 		// cout << "Path not found!"<< endl;
		// 	}
/*
		if(target_number>=1 && top_drone->obstacle_dynamic.num>=1)
		{
			top_drone->obstacle_dynamic.o_x.resize(1);
			top_drone->obstacle_dynamic.o_y.resize(1);
			// cout << top_drone->obstacle_dyna mic.o_x[0] << "\t" << top_drone->obstacle_dynamic.o_y[0];
			// cout <<"obs" <<top_drone->obstacle_dynamic.o_y[0];
			//path_rrt=pathplan.output(x,y,top_drone->obstacle_dynamic.o_x[0],top_drone->obstacle_dynamic.o_y[0]);
			path_rrt=pathplan.output(0.5,0,1,1);
			if(path_rrt.found==1)
			{
				path_rrt.x.resize(path_rrt.len);
				path_rrt.y.resize(path_rrt.len);
				int fff = path_rrt.len;
				cout << "Path found! " <<endl;// fff<<"\t"<< path_rrt.y[fff-1]<<"\t"<<path_rrt.y[fff-1]<<endl;
				for(int j=0;j<fff;j++)
				{
					cout << path_rrt.x[j]<<"\t";
				}
				cout << endl;
				for(int j=0;j<fff;j++)
				{
					cout << path_rrt.y[j]<<"\t";
				}
				cout << endl;
				// geometry_msgs::Point p;
				// for(int i=0;i<fff;i++)
				// {
				// 	p.x = (path_rrt.x[i]-20)*0.1;
				// 	p.y = (path_rrt.y[i]-15)*0.1;
				// 	p.z = -0.4/fff*i+0.5;
				// 	line_maker.points.push_back(p);
				// }
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
			// geometry_msgs::Point p;
			// std_msgs::ColorRGBA c;
          
			// p.x = 0;
			// p.y = 0;
			// p.z = 0.5;
			// line_maker.points.push_back(p);
			// c.r = 1;
			// c.g = 1;
			// c.b = 1;
			// c.a = 1.0;
			// line_maker.colors.push_back(c);
			// p.x = x*0.5;
			// p.y = y*0.5;
			// p.z = 0.3;
			// line_maker.points.push_back(p);
			// c.r = 1;
			// c.g = 1;
			// c.b = 1;
			// c.a = 1.0;
			// line_maker.colors.push_back(c);
			// p.x = x;
			// p.y = y;
			// p.z = 0.1;
			// line_maker.points.push_back(p);
			// c.r = 1;
			// c.g = 1;
			// c.b = 1;
			// c.a = 1.0;
			// line_maker.colors.push_back(c);
		}
		else
		{
			cout << "no target" << endl;
		}
*/
		quad_marker.pose.position.z = .05;
		quad_marker.pose.orientation.x = 0.0;
		quad_marker.pose.orientation.y = 0.0;
		quad_marker.pose.orientation.z = 0.0;
		quad_marker.pose.orientation.w = 1.0;
		quad_marker.scale.x = 0.15;
		quad_marker.scale.y = 0.15;
		quad_marker.scale.z = 0.10;
		quad_marker.color.a = 1.0; // Don't forget to set the alpha!
		quad_marker.color.r = .0;
		quad_marker.color.g = 1.0;
		quad_marker.color.b = 0.0;

		
		target_marker.header.frame_id = "base_link";
		target_marker.header.stamp = ros::Time();
		target_marker.ns = "quad";
		target_marker.id = 0;
		target_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    	target_marker.mesh_resource = "package://iarc/cfg/quad.dae";
		target_marker.action = visualization_msgs::Marker::ADD;
		target_marker.pose.position.x = 0.;
		target_marker.pose.position.y = 0.;
		target_marker.pose.position.z = .5;
		target_marker.pose.orientation.x = 0.0;
		target_marker.pose.orientation.y = 0.0;
		target_marker.pose.orientation.z = 0.0;
		target_marker.pose.orientation.w = 1.0;
		target_marker.scale.x = 0.30;
		target_marker.scale.y = 0.30;
		target_marker.scale.z = 0.30;
		target_marker.color.a = 1.0; // Don't forget to set the alpha!
		target_marker.color.r = 1.0;
		target_marker.color.g = 0.0;
		target_marker.color.b = 0.0;
		target_pub.publish(target_marker);
		rviz_pub.publish(quad_marker);
		marker_pub.publish(line_maker);
		ros::spinOnce();
		rate.sleep();
	}
	init_log.close();
	return 0;
}
