/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <fstream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/flann/miniflann.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#define RAD2DEG(x) ((x)*180./M_PI)
#define pi 3.1416
using namespace cv;
using namespace std;
//连续比较三次，最终确定柱子的位置信息
static float detect_degree[3][10] = {0};
static float detect_range[3][10] = {0};
static float detect_x[3][10] = {0};
static float detect_y[3][10] = {0};
static int detect_num = 0;
static float velocity = 0;
static float maybe_obstacle_degree[2][10] = {0};
static float maybe_obstacle_range[2][10] = {0};
static float maybe_obstacle_x[2][10] = {0};
static float maybe_obstacle_y[2][10] = {0};
static float real_obstacle_degree[10] = {0};
static float real_obstacle_range[10] = {0};
static float real_obstacle_x[10] = {0};
static float real_obstacle_y[10] = {0};
static float last_real_obstacle_x[10] = {0};
static float last_real_obstacle_y[10] = {0};
static float mid_real_obstacle_x[10] = {0};
static float mid_real_obstacle_y[10] = {0};
static float now_real_obstacle_x[10] = {0};
static float now_real_obstacle_y[10] = {0};
static int max_degree = 0;
static int min_degree = 0;
static int now_num[3] = {0};
static int right_num = 0;
static int total = 0;
static float rate_real_of_total = 0;
static int real_obstacle_num = 0;
static float hz = 12.8;

geometry_msgs::Vector3Stamped body_velocity;	//读取飞行器速度
Mat figurer(1001,1001,CV_8UC1,Scalar(0));
Mat init(1001,1001,CV_8UC1,Scalar(0));
Mat picture(1001,1001,CV_8UC3,Scalar(255,255,255));
Vec3b p(255,255,255);/*
p[0] = 255;
p[1] = 255;
p[2] = 255;*/
/*void delete_isolation_points(float* r, int count)
{
	int r1, r2, r3;
	short num[100], k;	//记录孤立点的角标
	int j = 0;
	for(int i = 0; i <= (count - 3); i++)
	{
		r1 = r[i];
		r2 = r[i + 1];
		r3 = r[i + 2];
		if(abs(r2 - r1) > 0.08 && abs(r2 - r3) > 0.08)
		{
			num[j] = i + 1;
			j++;
		}
	}
	for (int i = 0; i <= (j - 1); i++)
	{
		k = num[i];
		r[k] = 9.9;	
	}
}*/



float polar_to_x(float theta, float r)
{
	float x = 0;
	x = r * cos(theta);
	return x;
}
float polar_to_y(float theta, float r)
{
	float y =0;
	y = r * sin(theta);
	return y;
}
//调整单个inf点，线性拟合,避免误识别人或墙壁
void g_velocity_callback(const geometry_msgs::Vector3Stamped& velocity)
{
	body_velocity = velocity;
}
void isopoints_fit(float* r, int count)
{
	for(int i = 0; i <= (count - 3 + 10); i++)
	{
		if(r[i+1] > 12 && r[i] < 12 && r[i+2] < 12 && abs(r[i] - r[i+2]) <= 0.33)
		{
			r[i+1] = 0.5 * (r[i] + r[i+2]);
		}
	}
	//再把inf值变为0
	//for(int i =0; i<= (count - 1); i++)
	//{
	//	if(r[i] > 10)
	//	r[i] = 0;
	//}
}
//选出满足柱子特征的点：满足对应距离的情况下，2个点视为柱子，3个点中间点最小视为柱子，4个点中间两点更小视为柱子
int choose(float* degree_obstacle, float* range_obstacle, int num, float* r, int* obstacle, float* d, int count, float safe_range2, float safe_range3, float safe_range4, float safe_range5, float safe_range6, float safe_range7 )
{
	for(int i = 0; i < count + 10; i++)
	{
		obstacle[i] = 0;
	}
	//识别两个点的柱子
	for (int i = 0; i <= count - 4 + 10; i++)
	{
		if( abs(r[i] - r[i+1]) > 0.33 && abs(r[i+1] - r[i+2]) < 0.08 && abs(r[i+2] - r[i+3]) > 0.33 && r[i+1] < safe_range2 )
		{obstacle[i+1] = 1; //被判定为柱子的标记为1
		obstacle[i+2] = 1;
		degree_obstacle[num] = d[i+1];
		range_obstacle[num] = r[i+1];
		num++;}
	}
	//识别三个点的柱子，3个点中间点最小视为柱子
	for (int i = 0; i <= count - 5 + 10; i++)
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
	for (int i = 0; i <= count - 6 + 10; i++)
	{
		if( abs(r[i] - r[i+1]) > 0.25 && abs(r[i+1] - r[i+2]) < 0.08 && abs(r[i+2] - r[i+3]) < 0.08 && abs(r[i+3] - r[i+4]) < 0.08 && abs(r[i+4] - r[i+5]) > 0.25 && (r[i+1] > r[i+2] || abs(r[i+1]-r[i+2]) < 0.008) && (r[i+4] > r[i+3] || abs(r[i+4]-r[i+3]) < 0.008) && r[i+2] < safe_range2)
		{obstacle[i+1] = 1; //被判定为柱子的标记为1
		obstacle[i+2] = 1;
		obstacle[i+3] = 1;
		obstacle[i+4] = 1;
		degree_obstacle[num] = d[i+2];
		range_obstacle[num] = r[i+2];
		num++;}
	}
	//识别五个点柱子
	for (int i = 0; i <= count - 7 + 10; i++)
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
	for (int i = 0; i <= count - 8 + 10; i++)
	{
		if( abs(r[i] - r[i+1]) > 0.25 && abs(r[i+1] - r[i+2]) < 0.06 && abs(r[i+2] - r[i+3]) < 0.06 && abs(r[i+3] - r[i+4]) < 0.06 && abs(r[i+4] - r[i+5]) < 0.06 && abs(r[i+5] - r[i+6]) < 0.06 && abs(r[i+6] - r[i+7]) > 0.25 && r[i+3] < safe_range3)
		{obstacle[i+1] = 1; //被判定为柱子的标记为1
		obstacle[i+2] = 1;
		obstacle[i+3] = 1;
		obstacle[i+4] = 1;
		obstacle[i+5] = 1;
		obstacle[i+6] = 1;
		degree_obstacle[num] = d[i+4];
		range_obstacle[num] = r[i+4];
		num++;}
	}
		//识别7个点柱子
	for (int i = 0; i <= count - 9 + 10; i++)
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
	for (int i = 0; i <= count - 10 + 10; i++)
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
		degree_obstacle[num] = d[i+5];
		range_obstacle[num] = r[i+5];
		num++;}
	}
		//识别9个点柱子
	for (int i = 0; i <= count - 11 + 10; i++)
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
void biaoding(int count, float* degree_obstacle)
{
	for(int i = 0; i < 10; i++)
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
	ofstream fout;		//定义文件对象
    int count = scan->scan_time / scan->time_increment;
    //cout << "1"<<endl;
    float ranges_group[count+10];
	for(int i =0; i<= (count - 1); i++)
		ranges_group[i] = scan->ranges[i];
	
    float safe_range2 = 2.86;	//定义安全距离
	float safe_range3 = 1.91;
	float safe_range4 = 1.43;
	float safe_range5 = 1.15;
	float safe_range6 = 0.95;
	float safe_range7 = 0.81;
	//扩充 is_obstacle 使得覆盖全角度
	int is_obstacle[count+10]; //柱子标记为1
	//int num_obstacle = 0;
	//int points_obstacle[15];   //points_obstacle返回第i个柱子的点数，便于计算柱子角度
	float degree_obstacle[10] = {0};
	float range_obstacle[10] = {0};
	int	num = 0;

	float degree_group[count+10];
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
		degree_group[i] = degree;
    }
	for(int i = count; i < count + 10; i++)
	{
		degree_group[i] = degree_group[i - count];
		ranges_group[i] = ranges_group[i - count];
	}

    for(int i=0;i<1000;i++)
    {
    	for(int j=0;j<1000;j++)
    	{
    		figurer.at<uchar>(i,j)=0;
    	}
    }
    for(int i=0;i<1000;i++)
    {
    	for(int j=0;j<1000;j++)
    	{
            picture.at<Vec3b>(i,j)=(0,0,0);
    	}
    }
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    isopoints_fit(ranges_group, count);
	
	num = choose(degree_obstacle, range_obstacle, num, ranges_group, is_obstacle, degree_group, count, safe_range2, safe_range3, safe_range4, safe_range5, safe_range6, safe_range7);
	//在主程序里应用判定障碍物算法
	biaoding(count, degree_obstacle);
	for(int i = 0; i < 10; i++)
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


	//实时显示障碍物信息
	for(int i = 0; i < 10; i++)
	{
		if(detect_range[detect_num][i] != 0)
		ROS_INFO(": [%f, %f]", detect_degree[detect_num][i], detect_range[detect_num][i]);
	}

	now_num[2] = detect_num;	//now_num[2]表示最新一次检测detect_degree的序号，比如detect_num=1,则最新数据存在detect_degree[1]里，上一次的在d..[0],再上一次d..[2]
	now_num[1] = detect_num - 1;
	if(now_num[1] == -1)
		now_num[1] = 2;
	now_num[0] = now_num[1] - 1;
	if(now_num[0] == -1)
		now_num[0] = 2;
	//ROS_INFO("now_num[0]: [%d]", now_num[0]);
	//ROS_INFO("now_num[1]: [%d]", now_num[1]);
	//ROS_INFO("now_num[2]: [%d]", now_num[2]);
	detect_num++;
	if(detect_num >= 3)
		detect_num = 0;

	velocity = sqrt(body_velocity.vector.x*body_velocity.vector.x + body_velocity.vector.y*body_velocity.vector.y);
//	max_degree = (180 / pi) * (velocity + 0.33) * (1 / 12.8);	//例如飞行器速度为0.67， 则每检测一次差距角度为5.7度
//	min_degree = (180 / pi) * (velocity - 0.33) * (1 / 12.8);
	max_degree = 5.7;
//	min_degree = 0;
	//第一次，比较相邻两次测量
	bool is_range = 1;
	int maybe_num[2] = {0};
	for(int j = 0; j <= 1; j++)
	{
		for(int i = 0; i < 10; i++)
		{
			if(detect_range[now_num[j]][i] != 0)	//有效距离和角度
			{
				for(int k = 0; k < 10; k++)
				{
					float range_diff = sqrt((detect_x[now_num[j]][i] - detect_x[now_num[j+1]][k])*(detect_x[now_num[j]][i] - detect_x[now_num[j+1]][k]) + (detect_y[now_num[j]][i] - detect_y[now_num[j+1]][k])*(detect_y[now_num[j]][i] - detect_y[now_num[j+1]][k]));
				//	ROS_INFO("range_diff[%d, %d] is %f", j, k, range_diff);
					if(velocity >= 0.33)
					is_range = range_diff < ((1/hz) * (velocity + 0.33)) && range_diff >= ((1/hz) * (velocity - 0.33));
					else if(velocity < 0.33)
					is_range = range_diff < ((1/hz) * (velocity + 0.33));
					//ROS_INFO("0 1 is_range: [%d]", is_range);
					bool a = abs(detect_degree[now_num[j]][i] - detect_degree[now_num[j+1]][k]) <= max_degree;
					bool b1 = detect_degree[now_num[j]][i]>0 && detect_degree[now_num[j+1]][k]<0 && abs(180-detect_degree[now_num[j]][i])+abs(-180-detect_degree[now_num[j+1]][k])<max_degree;
					bool b2 = detect_degree[now_num[j]][i]<0 && detect_degree[now_num[j+1]][k]>0 && abs(-180-detect_degree[now_num[j]][i])+abs(180-detect_degree[now_num[j+1]][k])<max_degree;
					if( (a || b1  || b2) &&  detect_range[now_num[j+1]][k] != 0 /*&& is_range*/)
					{
					//	ROS_INFO("maybe0 [%d, %d, %d]", j, i, k);
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
	for(int i = 0; i < 10; i++)
	{
		if(maybe_obstacle_range[0][i] != 0)	//有效距离和角度
		{
			for(int k = 0; k < 10; k++)
			{
					float range_diff = sqrt((maybe_obstacle_x[0][i] - maybe_obstacle_x[1][k])*(maybe_obstacle_x[0][i] - maybe_obstacle_y[1][k]) + (maybe_obstacle_y[0][i] - maybe_obstacle_y[1][k])*(maybe_obstacle_y[0][i] - maybe_obstacle_y[1][k]));
					if(velocity >= 0.33)
					is_range = range_diff < ((1/hz) * (velocity + 0.33)) && range_diff >= ((1/hz) * (velocity - 0.33));
					else if(velocity < 0.33)
					is_range = range_diff < ((1/hz) * (velocity + 0.33));
				//	ROS_INFO("real is_range: [%d]", is_range);
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
	//real_obstacle里相同的值变为0
	for(int i = 0; i < 10; i++)
	{
		if(real_obstacle_range[i] != 0)
		{
			for(int k = i+1; k < 10; k++)
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
	//maybe_obstacle显示并 重新赋为0
	for(int i = 0; i < 2; i++)
	{
		for(int j = 0; j < 10; j++)
		{
	//		if(i == 0 && maybe_obstacle_range[i][j] !=0)
	//		ROS_INFO("maybe_obstacle 0: [%f, %f]", maybe_obstacle_degree[i][j], maybe_obstacle_range[i][j]);
	//		else if(maybe_obstacle_range[i][j] !=0)
	//		ROS_INFO("maybe_obstacle 1: [%f, %f]", maybe_obstacle_degree[i][j], maybe_obstacle_range[i][j]);
			maybe_obstacle_degree[i][j] = 0;
			maybe_obstacle_x[i][j] = 0;
			maybe_obstacle_y[i][j] = 0;
			maybe_obstacle_range [i][j] = 0;
		}
	}

	//将上一次和本次检测的最终障碍物信息存在两个数组里
	int velocity_flag = 0;

		for(int i = 0; i<10; i++)
		{
			last_real_obstacle_x[i] = mid_real_obstacle_x[i];
			last_real_obstacle_y[i] = mid_real_obstacle_y[i];
		}	
		for(int i = 0; i<10; i++)
		{
			now_real_obstacle_x[i] = real_obstacle_x[i];
			now_real_obstacle_y[i] = real_obstacle_y[i];
		}	
		for(int i = 0; i<10; i++)
		{
			mid_real_obstacle_x[i] = real_obstacle_x[i];
			mid_real_obstacle_y[i] = real_obstacle_y[i];
		}
	velocity_flag = ~velocity_flag;
	
	
	//障碍物速度估计
	float obstacle_vx[10] = {0};
	float obstacle_vy[10] = {0};
	for(int i = 0; i<10; i++)
	{
		if(last_real_obstacle_x[i] != 0 && last_real_obstacle_y[i] != 0)
		{
			for(int k = 0; k<10; k++)
			{
				if(now_real_obstacle_x[k] != 0 && now_real_obstacle_y[k] != 0)
				{
					//将这一次的位置换算到上一次的机体坐标系下
					float now_to_last_x = now_real_obstacle_x[k] - body_velocity.vector.x * (1/hz);
					float now_to_last_y = now_real_obstacle_y[k] - body_velocity.vector.y * (1/hz);
					float range_of_two = 0;//两次测量障碍物的距离
					range_of_two = sqrt((last_real_obstacle_x[i]-now_to_last_x)*(last_real_obstacle_x[i]-now_to_last_x) + (last_real_obstacle_y[i]-now_to_last_y)*(last_real_obstacle_y[i]-now_to_last_y));
					ROS_INFO("range_of_two[%d,%d]: [%f]",i,k, range_of_two);
					if(range_of_two <= 0.5*(1/hz))	//障碍物速度为0.33，留一定裕度设置为0.5
					{
						obstacle_vx[k] = (now_to_last_x - last_real_obstacle_x[i]) * hz;
						obstacle_vy[k] = (now_to_last_y - last_real_obstacle_y[i]) * hz;
					}	
				}
			}
		}


	}
	//显示第k个障碍物的速度
	for(int k = 0; k < 10; k++)
	{
		if(obstacle_vx[k] != 0)
		ROS_INFO("obstacle_velocity of %d: [%f, %f]", k, obstacle_vx[k], obstacle_vy[k]);
	}

	//显示障碍物的位置
	for(int i = 0; i < 10; i++)
	{	
		if(real_obstacle_range[i] != 0)
		{

			ROS_INFO("obstacle_position: [%f, %f]", real_obstacle_x[i], real_obstacle_y[i]);
			ROS_INFO("obstacle_polar: [%f, %f]", real_obstacle_degree[i], real_obstacle_range[i]);
			real_obstacle_num++;
			int x = 501+ceil(50*real_obstacle_range[i]*cos(pi * real_obstacle_degree[i] / 180));
			int y = 501+ceil(50*real_obstacle_range[i]*sin(pi * real_obstacle_degree[i] / 180));
        	//cout << x<<"\t"<<y<< endl;
        	if(x<=1000&x>=0&y<=1000&y>=0)
            {
                picture.at<Vec3b>(x,y)[0] = 255;
				picture.at<Vec3b>(x,y)[1] = 0;
				picture.at<Vec3b>(x,y)[2] = 0;
				cv::circle(picture,cvPoint(y,x),10,CV_RGB(0,255,255),2,8,0);
            }

		}
		real_obstacle_range [i] = 0;
		real_obstacle_degree[i] = 0;
		real_obstacle_x[i] = 0;
		real_obstacle_y[i] = 0;
	}

	if(real_obstacle_num == 1)
		right_num++;
	total++;
	rate_real_of_total = ((float)right_num) / ((float)total);
	ROS_INFO("right number / total number: [%d, %d], about [%f]", right_num, total, rate_real_of_total);
	real_obstacle_num = 0;

    for(int i = 0; i < count + 10; i++) {
        //float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //float degree = scan->angle_min + scan->angle_increment * i;
        //float range = scan->ranges[i];
        if(ranges_group[i]<=10)
        {
        	
        	int x = 501+ceil(50*ranges_group[i]*cos(pi * degree_group[i] / 180));
        	int y = 501+ceil(50*ranges_group[i]*sin(pi * degree_group[i] / 180));
        	//cout << x<<"\t"<<y<< endl;
        	if(x<=1000&x>=0&y<=1000&y>=0)
            {
                figurer.at<uchar>(x,y) = 250;
            }
        	//figurer=Scalar(100);
        }
        figurer.at<uchar>(501,501) = 255;
        figurer.at<uchar>(501,500) = 255;
        figurer.at<uchar>(501,502) = 255;
        figurer.at<uchar>(500,501) = 255;
        figurer.at<uchar>(500,500) = 255;
        figurer.at<uchar>(500,502) = 255;
        figurer.at<uchar>(502,501) = 255;
        figurer.at<uchar>(502,500) = 255;
        figurer.at<uchar>(502,502) = 255;
        
        picture.at<Vec3b>(501,501) = p;
        picture.at<Vec3b>(501,500) = p;
        picture.at<Vec3b>(501,502) = p;
        picture.at<Vec3b>(500,501) = p;
        picture.at<Vec3b>(500,500) = p;
        picture.at<Vec3b>(500,502) = p;
        picture.at<Vec3b>(502,501) = p;
        picture.at<Vec3b>(502,500) = p;
        picture.at<Vec3b>(502,502) = p;
    }
	for(int i = 0; i < 10; i++)
	{	
		if(real_obstacle_range[i] != 0)
		{
			int x = 501+ceil(50*real_obstacle_range[i]*cos(pi * real_obstacle_degree[i] / 180));
			int y = 501+ceil(50 *real_obstacle_range[i]*sin(pi * real_obstacle_degree[i] / 180));
        	//cout << x<<"\t"<<y<< endl;
        	if(x<=1000&x>=0&y<=1000&y>=0)
            {
                picture.at<Vec3b>(x,y)[0] = 255;
				picture.at<Vec3b>(x,y)[1] = 0;
				picture.at<Vec3b>(x,y)[2] = 0;
            }
		}
	}
		fout.open("output.txt");
        for(int i = 0; i < count + 10; i++)
        {
	        fout << degree_group[i];		//角度写入文件
	        fout << "   " << ranges_group[i];	//距离写入文件
			fout << "   " << is_obstacle[i] << '\n';
        }
	fout << flush; 
	fout.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar4");
    ros::NodeHandle n;
	ros::Subscriber body_velocity = n.subscribe("/guidance/velocity",  10, g_velocity_callback);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10000, scanCallback);
    ros::Rate rate(10);
    while(ros::ok())
    {
        if(figurer.data)
        {
            //cout << "data correct"<< endl;
            imshow("rplida1r",figurer);
            imshow("obstacle",picture);
            //cout << "wtf" <<endl;
            //int key=waitKey(27);
            if(waitKey(30)>=0)
            {
                return 0;
            }
            //cout << "wtf" <<endl;
        }
        else
        {
            cout <<"data error"<<endl;
        }
        ros::spinOnce();
        rate.sleep();
    } 
	ros::spin();
    return 0;
}
