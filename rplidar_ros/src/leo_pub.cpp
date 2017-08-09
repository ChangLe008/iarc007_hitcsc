/*
 * Copyright (c) 2017, HITCSC Team
 * All rights reserved.
 *
 *
 */
/*
 *  HITCSC TEAM IARC Mission 7a
 *  RPLIDAR Obstacle detection node 
 *
 *  Copyright 2016 - 2017 HITCSC Team
 * 
 */


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <fstream>
#include "iarc/obstacle.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/flann/miniflann.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <visualization_msgs/Marker.h>
#define RAD2DEG(x) ((x)*180./M_PI)
#define pi 3.1416
using namespace std;

ros::Publisher chatter_pub;
ros::Publisher rviz_pub;
ros::Publisher rviz_range_pub;

visualization_msgs::Marker obs_marker;
visualization_msgs::Marker obs_range_marker;


float safe_range2 = 2.86;	//定义安全距离
float safe_range3 = 1.91;
float safe_range4 = 1.43;
float safe_range5 = 1.15;
float safe_range6 = 0.95;
float safe_range7 = 0.81;
static float ranges_group[370] = {0};
static float degree_group[370] = {0};
static int   is_obstacle[370]  = {0};  //柱子标记为1
static float detect_degree[3][20] = {0};
static float detect_range[3][20]  = {0};
static float detect_x[3][20] = {0};
static float detect_y[3][20] = {0};
static int   detect_num = 0;
static float velocity = 0;
static float maybe_obstacle_degree[2][20] = {0};
static float maybe_obstacle_range[2][20] = {0};
static float maybe_obstacle_x[2][20] = {0};
static float maybe_obstacle_y[2][20] = {0};
static float real_obstacle_degree[20] = {0};
static float real_obstacle_range[20] = {0};
static float real_obstacle_x[20] = {0};
static float real_obstacle_y[20] = {0};
static float last_real_obstacle_x[20] = {0};
static float last_real_obstacle_y[20] = {0};
static float mid_real_obstacle_x[20] = {0};
static float mid_real_obstacle_y[20] = {0};
static float now_real_obstacle_x[20] = {0};
static float now_real_obstacle_y[20] = {0};
static float max_degree = 0;
static int   min_degree = 0;
static int   now_num[3] = {0};
static float hz = 12.8;

geometry_msgs::Vector3Stamped body_velocity;	//读取飞行器速度
//极座标角度（角度制）和距离到直角座标的转换
float polar_to_x(float theta, float r)
{
	float x = 0;
	x = r * cos(pi*theta / 180);
	return x;
}
float polar_to_y(float theta, float r)
{
	float y =0;
	y = r * sin(pi*theta / 180);
	return y;
}
//返回飞行器速度
void g_velocity_callback(const geometry_msgs::Vector3Stamped& velocity)
{
	body_velocity = velocity;
}
//调整单个inf点，线性拟合,避免误识别人或墙壁
void isopoints_fit(float* r)
{
	for(int i = 0; i <= (360 - 3 + 10); i++)
	{
		if(r[i+1] > 12 && r[i] < 12 && r[i+2] < 12 && abs(r[i] - r[i+2]) <= 0.33)
		{
			r[i+1] = 0.5 * (r[i] + r[i+2]);
		}
	}
}
//选出满足柱子特征的点：满足对应距离的情况下，2个点视为柱子，3个点中间点最小视为柱子，4个点中间两点更小视为柱子
int choose(float* degree_obstacle, float* range_obstacle, int num, float* r, int* obstacle, float* d)
{
	for(int i = 0; i < 370; i++)
	{
		obstacle[i] = 0;
	}
	//识别两个点的柱子
	for (int i = 0; i <= 360 - 4 + 10; i++)
	{
		if( abs(r[i] - r[i+1]) > 0.33 && abs(r[i+1] - r[i+2]) < 0.08 && abs(r[i+2] - r[i+3]) > 0.33 && r[i+1] < safe_range2 )
		{obstacle[i+1] = 1; //被判定为柱子的标记为1
		obstacle[i+2] = 1;
		degree_obstacle[num] = (d[i+1] + d[i+2]) / 2;
		range_obstacle[num]  = (r[i+1] + r[i+2]) / 2;
		num++;}
	}
	//识别三个点的柱子，3个点中间点最小视为柱子
	for (int i = 0; i <= 360 - 5 + 10; i++)
	{
		if( abs(r[i] - r[i+1]) > 0.25 && abs(r[i+1] - r[i+2]) < 0.08 && abs(r[i+2] - r[i+3]) < 0.08 && abs(r[i+3] - r[i+4]) > 0.25 && r[i+2] < safe_range2 && r[i+2] <= (r[i+1]+0.008) && r[i+2] <= (r[i+3]+0.008))
		{obstacle[i+1] = 1; //被判定为柱子的标记为1
		obstacle[i+2] = 1;
		obstacle[i+3] = 1;
		degree_obstacle[num] = d[i+2];
		range_obstacle[num] = r[i+2];
		num++;}
	}
	//识别四个点的柱子
	for (int i = 0; i <= 360 - 6 + 10; i++)
	{
		if( abs(r[i] - r[i+1]) > 0.25 && abs(r[i+1] - r[i+2]) < 0.08 && abs(r[i+2] - r[i+3]) < 0.08 && abs(r[i+3] - r[i+4]) < 0.08 && abs(r[i+4] - r[i+5]) > 0.25 && (r[i+1] > r[i+2] || abs(r[i+1]-r[i+2]) < 0.008) && (r[i+4] > r[i+3] || abs(r[i+4]-r[i+3]) < 0.008) && r[i+2] < safe_range2)
		{obstacle[i+1] = 1; //被判定为柱子的标记为1
		obstacle[i+2] = 1;
		obstacle[i+3] = 1;
		obstacle[i+4] = 1;
		degree_obstacle[num] = (d[i+2] + d[i+3]) / 2;
		range_obstacle[num]  = (r[i+2] + r[i+3]) / 2;
		num++;}
	}
	//识别五个点柱子
	for (int i = 0; i <= 360 - 7 + 10; i++)
	{
		if( abs(r[i] - r[i+1]) > 0.25 && abs(r[i+1] - r[i+2]) < 0.07 && abs(r[i+2] - r[i+3]) < 0.07 && abs(r[i+3] - r[i+4]) < 0.07 && abs(r[i+4] - r[i+5]) < 0.07 && abs(r[i+5] - r[i+6]) > 0.25 && (r[i+1]+0.015)>r[i+3] && (r[i+5]+0.015) > r[i+3] && r[i+3] < (safe_range3))
		{obstacle[i+1] = 1; //被判定为柱子的标记为1
		obstacle[i+2] = 1;
		obstacle[i+3] = 1;
		obstacle[i+4] = 1;
		obstacle[i+5] = 1;
		degree_obstacle[num] = d[i+3];
		range_obstacle[num] = r[i+3];
		num++;}
	}
	//识别6个点柱子
	for (int i = 0; i <= 360 - 8 + 10; i++)
	{
		if( abs(r[i] - r[i+1]) > 0.25 && abs(r[i+1] - r[i+2]) < 0.06 && abs(r[i+2] - r[i+3]) < 0.06 && abs(r[i+3] - r[i+4]) < 0.06 && abs(r[i+4] - r[i+5]) < 0.06 && abs(r[i+5] - r[i+6]) < 0.06 && abs(r[i+6] - r[i+7]) > 0.25 && r[i+3] < safe_range3)
		{obstacle[i+1] = 1; //被判定为柱子的标记为1
		obstacle[i+2] = 1;
		obstacle[i+3] = 1;
		obstacle[i+4] = 1;
		obstacle[i+5] = 1;
		obstacle[i+6] = 1;
		degree_obstacle[num] = (d[i+3] + d[i+4]) / 2;
		range_obstacle[num]  = (r[i+3] + r[i+4]) / 2;
		num++;}
	}
		//识别7个点柱子
	for (int i = 0; i <= 360 - 9 + 10; i++)
	{
		if( abs(r[i] - r[i+1]) > 0.25 && abs(r[i+1] - r[i+2]) < 0.05 && abs(r[i+2] - r[i+3]) < 0.05 && abs(r[i+3] - r[i+4]) < 0.05 && abs(r[i+4] - r[i+5]) < 0.05 && abs(r[i+5] - r[i+6]) < 0.05 && abs(r[i+6] - r[i+7]) < 0.05 && abs(r[i+7] - r[i+8]) > 0.25 && r[i+4] < safe_range4)
		{obstacle[i+1] = 1; //被判定为柱子的标记为1
		obstacle[i+2] = 1;
		obstacle[i+3] = 1;
		obstacle[i+4] = 1;
		obstacle[i+5] = 1;
		obstacle[i+6] = 1;
		obstacle[i+7] = 1;
		degree_obstacle[num] = d[i+4];
		range_obstacle[num] = r[i+4];
		num++;}
	}
		//识别8个点柱子
	for (int i = 0; i <= 360 - 10 + 10; i++)
	{
		if( abs(r[i] - r[i+1]) > 0.25 && abs(r[i+1] - r[i+2]) < 0.04 && abs(r[i+2] - r[i+3]) < 0.04 && abs(r[i+3] - r[i+4]) < 0.04 && abs(r[i+4] - r[i+5]) < 0.04 && abs(r[i+5] - r[i+6]) < 0.04 && abs(r[i+6] - r[i+7]) < 0.04 && abs(r[i+7] - r[i+8]) < 0.04 && abs(r[i+8] - r[i+9]) > 0.25 && r[i+4] < safe_range5)
		{obstacle[i+1] = 1; //被判定为柱子的标记为1
		obstacle[i+2] = 1;
		obstacle[i+3] = 1;
		obstacle[i+4] = 1;
		obstacle[i+5] = 1;
		obstacle[i+6] = 1;
		obstacle[i+7] = 1;
		obstacle[i+8] = 1;
		degree_obstacle[num] = (d[i+4] + d[i+5]) / 2;
		range_obstacle[num]  = (r[i+4] + r[i+5]) / 2;
		num++;}
	}
		//识别9个点柱子
	for (int i = 0; i <= 360 - 11 + 10; i++)
	{
		if( abs(r[i] - r[i+1]) > 0.25 && abs(r[i+1] - r[i+2]) < 0.04 && abs(r[i+2] - r[i+3]) < 0.04 && abs(r[i+3] - r[i+4]) < 0.04 && abs(r[i+4] - r[i+5]) < 0.04 && abs(r[i+5] - r[i+6]) < 0.04 && abs(r[i+6] - r[i+7]) < 0.04 && abs(r[i+7] - r[i+8]) < 0.04 && abs(r[i+8] - r[i+9]) < 0.04 && abs(r[i+9] - r[i+10]) > 0.25 && r[i+5] < safe_range5)
		{obstacle[i+1] = 1; //被判定为柱子的标记为1
		obstacle[i+2] = 1;
		obstacle[i+3] = 1;
		obstacle[i+4] = 1;
		obstacle[i+5] = 1;
		obstacle[i+6] = 1;
		obstacle[i+7] = 1;
		obstacle[i+8] = 1;
		obstacle[i+9] = 1;
		degree_obstacle[num] = d[i+5];
		range_obstacle[num] = r[i+5];
		num++;}
	}
	return num;
}

//坐标系变换到与机体坐标系一致
void biaoding(float* degree_obstacle)
{
	for(int i = 0; i < 20; i++)
	{
		if(degree_obstacle[i] <= 0)
			degree_obstacle[i] = degree_obstacle[i] + 178;
		else if(degree_obstacle[i] > 0)
			degree_obstacle[i] = degree_obstacle[i] + 178 - 360;
		degree_obstacle[i] = -degree_obstacle[i];
	}
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
	for(int i =0; i<= (count - 1); i++)
		ranges_group[i] = scan->ranges[i];
	

	//扩充 is_obstacle 使得覆盖全角度

	float degree_obstacle[20] = {0};
	float range_obstacle[20]  = {0};
	int	num = 0;

    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
		degree_group[i] = degree;
    }
	//为扩充后数组的后10个赋值
	for(int i = count; i < 370; i++)
	{
		degree_group[i] = degree_group[i - count];
		ranges_group[i] = ranges_group[i - count];
	}
    isopoints_fit(ranges_group);
		//在主程序里应用判定障碍物算法
	num = choose(degree_obstacle, range_obstacle, num, ranges_group, is_obstacle, degree_group);
	biaoding(degree_obstacle);

	for(int i = 0; i < 20; i++)
	{
		detect_degree[detect_num][i] = 0; 
		detect_range[detect_num][i]  = 0; 
	}
	for(int i = 0; i <= num - 1; i++)
	{
		detect_degree[detect_num][i] = degree_obstacle[i]; 
		detect_range[detect_num][i]  = range_obstacle[i]; 
		detect_x[detect_num][i] = polar_to_x(detect_degree[detect_num][i], detect_range[detect_num][i]);
		detect_y[detect_num][i] = polar_to_y(detect_degree[detect_num][i], detect_range[detect_num][i]);
	}

	now_num[2] = detect_num;	//now_num[2]表示最新一次检测detect_degree的序号，比如detect_num=1,则最新数据存在detect_degree[1]里，上一次的在d..[0],再上一次d..[2]
	now_num[1] = detect_num - 1;
	if(now_num[1] == -1)
		now_num[1] = 2;
	now_num[0] = now_num[1] - 1;
	if(now_num[0] == -1)
		now_num[0] = 2;
	detect_num++;
	if(detect_num >= 3)
		detect_num = 0;

	velocity = sqrt(body_velocity.vector.x*body_velocity.vector.x + body_velocity.vector.y*body_velocity.vector.y);
	//	max_degree = (180 / pi) * (velocity + 0.33) * (1 / 12.8);	//例如飞行器速度为0.67， 则每检测一次差距最大角度为5.7度
	//	min_degree = (180 / pi) * (velocity - 0.33) * (1 / 12.8);
	max_degree = 5.7;
	//	min_degree = 0;
	//第一次，比较相邻两次测量
	bool is_range = 1;
	int maybe_num[2] = {0};
	for(int j = 0; j <= 1; j++)
	{
		for(int i = 0; i < 20; i++)
		{
			if(detect_range[now_num[j]][i] != 0)	//有效距离和角度
			{
				for(int k = 0; k < 20; k++)
				{
					float range_diff = sqrt((detect_x[now_num[j]][i] - detect_x[now_num[j+1]][k])*(detect_x[now_num[j]][i] - detect_x[now_num[j+1]][k]) + (detect_y[now_num[j]][i] - detect_y[now_num[j+1]][k])*(detect_y[now_num[j]][i] - detect_y[now_num[j+1]][k]));
					if(velocity >= 0.33)
					is_range = range_diff < ((1/hz) * (velocity + 0.33)) && range_diff >= ((1/hz) * (velocity - 0.33));
					else if(velocity < 0.33)
					is_range = range_diff < ((1/hz) * (velocity + 0.33));
					bool a = abs(detect_degree[now_num[j]][i] - detect_degree[now_num[j+1]][k]) <= max_degree;
					bool b1 = detect_degree[now_num[j]][i]>0 && detect_degree[now_num[j+1]][k]<0 && abs(180-detect_degree[now_num[j]][i])+abs(-180-detect_degree[now_num[j+1]][k])<max_degree;
					bool b2 = detect_degree[now_num[j]][i]<0 && detect_degree[now_num[j+1]][k]>0 && abs(-180-detect_degree[now_num[j]][i])+abs(180-detect_degree[now_num[j+1]][k])<max_degree;
					if( (a || b1  || b2) &&  detect_range[now_num[j+1]][k] != 0 /*&& is_range*/ )
					{
						maybe_obstacle_degree[j][maybe_num[j]] = detect_degree[now_num[j+1]][k];
						maybe_obstacle_range [j][maybe_num[j]] = detect_range [now_num[j+1]][k];
						maybe_obstacle_x[j][maybe_num[j]] = polar_to_x(maybe_obstacle_degree[j][maybe_num[j]], maybe_obstacle_range [j][maybe_num[j]]);
						maybe_obstacle_y[j][maybe_num[j]] = polar_to_y(maybe_obstacle_degree[j][maybe_num[j]], maybe_obstacle_range [j][maybe_num[j]]);
						maybe_num[j]++;
					}
				}
			}
		}
	}

	int real_num = 0;
	//第二次，比较得到的中间结果
	for(int i = 0; i < 20; i++)
	{
		if(maybe_obstacle_range[0][i] != 0)	//有效距离和角度
		{
			for(int k = 0; k < 20; k++)
			{
					float range_diff = sqrt((maybe_obstacle_x[0][i] - maybe_obstacle_x[1][k])*(maybe_obstacle_x[0][i] - maybe_obstacle_y[1][k]) + (maybe_obstacle_y[0][i] - maybe_obstacle_y[1][k])*(maybe_obstacle_y[0][i] - maybe_obstacle_y[1][k]));
					if(velocity >= 0.33)
					is_range = range_diff < ((1/hz) * (velocity + 0.33)) && range_diff >= ((1/hz) * (velocity - 0.33));
					else if(velocity < 0.33)
					is_range = range_diff < ((1/hz) * (velocity + 0.33));
					bool a = abs(maybe_obstacle_degree[0][i] - maybe_obstacle_degree[1][k]) <= max_degree;
					bool b1 = maybe_obstacle_degree[0][i]>0 && maybe_obstacle_degree[1][k]<0 && abs(180-maybe_obstacle_degree[0][i])+abs(-180-maybe_obstacle_degree[1][k])<max_degree;
					bool b2 = maybe_obstacle_degree[0][i]<0 && maybe_obstacle_degree[1][k]>0 && abs(-180-maybe_obstacle_degree[0][i])+abs(180-maybe_obstacle_degree[1][k])<max_degree;
				if( (a || b1 || b2) && maybe_obstacle_range[1][k] != 0 /*&& is_range*/)
				{
					
					real_obstacle_degree[real_num] = maybe_obstacle_degree[1][k];
					real_obstacle_range[real_num]  = maybe_obstacle_range[1][k];
					real_obstacle_x[real_num] = polar_to_x(real_obstacle_degree[real_num], real_obstacle_range[real_num]);
					real_obstacle_y[real_num] = polar_to_y(real_obstacle_degree[real_num], real_obstacle_range[real_num]);
					real_num++;
				}
			}
		}
	}

	//real_obstacle里相同的值变为0, 再重新排序（把中间有0的值去掉）
	for(int i = 0; i < 20; i++)
	{
		if(real_obstacle_range[i] != 0)
		{
			for(int k = i+1; k < 20; k++)
			{
				if(real_obstacle_degree[i] == real_obstacle_degree[k] && real_obstacle_degree[k] != 0)
				{
					real_obstacle_degree[k] = 0;
					real_obstacle_range [k] = 0;
					real_obstacle_x[k] = 0;
					real_obstacle_y[k] = 0;
				}
			}
		}
	}
	int useless_num = 0;
	for(int i = 0; i< 20; i++)
	{
		if(real_obstacle_range[i] != 0)
		{
			real_obstacle_range[useless_num] = real_obstacle_range[i];
			real_obstacle_degree[useless_num] = real_obstacle_degree[i];
			real_obstacle_x[useless_num] = real_obstacle_x[i];
			real_obstacle_y[useless_num] = real_obstacle_y[i];
			useless_num++; 
		}
	}
	for(int i = useless_num; i < 20; i++)
	{
		real_obstacle_range[i] = 0;
		real_obstacle_degree[i] = 0;
		real_obstacle_x[i] = 0;
		real_obstacle_y[i] = 0;
	}
	useless_num = 0;



	//maybe_obstacle显示并 重新赋为0
	for(int i = 0; i < 2; i++)
	{
		for(int j = 0; j < 20; j++)
		{
			maybe_obstacle_degree[i][j] = 0;
			maybe_obstacle_x[i][j] = 0;
			maybe_obstacle_y[i][j] = 0;
			maybe_obstacle_range [i][j] = 0;
		}
	}
	int real_obstacle_num = 0;
	//显示障碍物的位置
	for(int i = 0; i < 20; i++)
	{	
		if(real_obstacle_range[i] != 0)
		{
			ROS_INFO("obstacle_polar: [%f, %f]", real_obstacle_degree[i], real_obstacle_range[i]);
			real_obstacle_num++;
		}
	}
	iarc::obstacle msg; //定义消息
	msg.header.frame_id = "lidar";
	msg.header.stamp = ros::Time::now();
	msg.o_x.resize(20);
	msg.o_y.resize(20);
	msg.num = real_obstacle_num;
	for(int i = 0; i < 20; i++)
	{
		msg.o_x[i] = (float)(real_obstacle_x[i]);
		msg.o_y[i] = (float)(real_obstacle_y[i]);
	}
	chatter_pub.publish(msg);

	for(int i = 0; i < 370; i++)
	{
		ranges_group[i] = 0;
		degree_group[i] = 0;
		is_obstacle[i]  = 0; 
	}

	
	
	obs_marker.header.frame_id = "base_link";
	obs_marker.header.stamp = ros::Time::now();
	obs_marker.ns = "my_namespace";
	obs_marker.id = 0;
	obs_marker.type = visualization_msgs::Marker::CYLINDER;
	obs_marker.action = visualization_msgs::Marker::ADD;

	obs_range_marker.header.frame_id = "base_link";
	obs_range_marker.header.stamp = ros::Time::now();
	obs_range_marker.ns = "my_namespace";
	obs_range_marker.id = 0;
	obs_range_marker.type = visualization_msgs::Marker::CYLINDER;
	obs_range_marker.action = visualization_msgs::Marker::ADD;


	if(msg.num>=1)
	{
		obs_marker.pose.position.x = (float)(real_obstacle_x[0]) + (.05*rand()/(RAND_MAX+1.0)-.025);
		obs_marker.pose.position.y = (float)(real_obstacle_y[0]) + (.05*rand()/(RAND_MAX+1.0)-.025);
		
		obs_range_marker.pose.position.x = (float)(real_obstacle_x[0]) + (.02*rand()/(RAND_MAX+1.0)-.01);
		obs_range_marker.pose.position.y = (float)(real_obstacle_y[0]) + (.02*rand()/(RAND_MAX+1.0)-.01);
	}
	else
	{
		;
	}
	obs_marker.pose.position.z = 0.1;
	obs_marker.pose.orientation.x = 0.0;
	obs_marker.pose.orientation.y = 0.0;
	obs_marker.pose.orientation.z = 0.0;
	obs_marker.pose.orientation.w = 1.0;
	obs_marker.scale.x = 0.75;
	obs_marker.scale.y = 0.75;
	obs_marker.scale.z = 0.2;
	obs_marker.color.a = 0.2; // Don't forget to set the alpha!
	obs_marker.color.r = 1.0;
	obs_marker.color.g = 1.0;
	obs_marker.color.b = 1.0;

	obs_range_marker.pose.position.z = .5;
	obs_range_marker.pose.orientation.x = 0.0;
	obs_range_marker.pose.orientation.y = 0.0;
	obs_range_marker.pose.orientation.z = 0.0;
	obs_range_marker.pose.orientation.w = 1.0;
	obs_range_marker.scale.x = 0.05;
	obs_range_marker.scale.y = 0.05;
	obs_range_marker.scale.z = 1;
	obs_range_marker.color.a = 1.0; // Don't forget to set the alpha!
	obs_range_marker.color.r = 1.0;
	obs_range_marker.color.g = .1;
	obs_range_marker.color.b = .1;


	rviz_pub.publish(obs_marker);
	rviz_range_pub.publish(obs_range_marker);
		
	for(int i = 0; i < 20; i++)
	{
		real_obstacle_range [i] = 0;
		real_obstacle_degree[i] = 0;
		real_obstacle_x[i] = 0;
		real_obstacle_y[i] = 0;
	}
	real_obstacle_num = 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_obstacle");
    ros::NodeHandle n;
	chatter_pub = n.advertise<iarc::obstacle>("/obstacle/lidar", 1000);	
	rviz_pub = n.advertise<visualization_msgs::Marker>("/rviz/lidar", 1000);
	rviz_range_pub = n.advertise<visualization_msgs::Marker>("/rviz/camera", 1000);
	// rviz_range_pub = n.advertise<visualization_msgs::Marker>("/rviz/lidar_range", 1000);

	ros::Subscriber body_velocity = n.subscribe("/guidance/velocity",  10, g_velocity_callback);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10000, scanCallback);
    ros::Rate rate(10);
    
	ros::spin();
    return 0;
}