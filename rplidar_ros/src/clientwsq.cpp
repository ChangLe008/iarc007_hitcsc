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
#include <fstream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/flann/miniflann.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <sys/time.h>

#define RAD2DEG(x) ((x)*180./M_PI)
#define pi 3.1416
using namespace cv;
using namespace std;
//连续比较三次，最终确定柱子的位置信息
static float detect_degree[3][10] = {0};
static float detect_range[3][10] = {0};
static int detect_num = 0;
static float body_velocity = 0.67;
static float maybe_obstacle_degree[2][10] = {0};
static float maybe_obstacle_range[2][10] = {0};
static float real_obstacle_degree[10] = {0};
static float real_obstacle_range[10] = {0};
static int max_degree = 0;
static int now_num[3] = {0};
static int right_num = 0;
static int total = 0;
static float rate_real_of_total = 0;
static int real_obstacle_num = 0;

ofstream init_log;

Mat figurer(1001,1001,CV_8UC1,Scalar(0));
Mat init(1001,1001,CV_8UC1,Scalar(0));
Mat picture(1001,1001,CV_8UC3,Scalar(255,255,255));
Vec3b p(0,0,0);/*

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
//调整单个inf点，线性拟合,避免误识别人或墙壁

double tic()
{
	struct timeval t;

	gettimeofday(&t,NULL);
	return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
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
	float degree_obstacle[10];
	float range_obstacle[10];
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
    		figurer.at<uchar>(i,j)=255;
    	}
    }
    for(int i=0;i<1000;i++)
    {
    	for(int j=0;j<1000;j++)
    	{
            picture.at<Vec3b>(i,j)[0]=255;
			picture.at<Vec3b>(i,j)[1]=255;
			picture.at<Vec3b>(i,j)[2]=255;
    	}
    }
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    isopoints_fit(ranges_group, count);
	
	num = choose(degree_obstacle, range_obstacle, num, ranges_group, is_obstacle, degree_group, count, safe_range2, safe_range3, safe_range4, safe_range5, safe_range6, safe_range7);
	//在主程序里应用判定障碍物算法
	for(int i = 0; i < 10; i++)
	{
		detect_degree[detect_num][i] = 0; 
		detect_range[detect_num][i]  = 0; 
	}
	for(int i = 0; i <= num - 1; i++)
	{
		detect_degree[detect_num][i] = degree_obstacle[i]; 
		detect_range[detect_num][i]  = range_obstacle[i]; 
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

	//实时显示障碍物信息
	for(int i = 0; i < 10; i++)
	{
		ROS_INFO(": [%f, %f]", degree_obstacle[i], range_obstacle[i]);
	}

	max_degree = (180 / pi) * (body_velocity + 0.33) * 0.1;	//例如飞行器速度为0.67， 则每检测一次差距角度为5.7度
	//第一次，比较相邻两次测量
	for(int j = 0; j <= 1; j++)
	{
		for(int i = 0; i < 10; i++)
		{
			if(detect_range[now_num[j]][i] != 0)	//有效距离和角度
			{
				for(int k = 0; k < 10; k++)
				{
					if((abs(detect_degree[now_num[j]][i] - detect_degree[now_num[j+1]][k]) <= max_degree || detect_degree[now_num[j]][i] + detect_degree[now_num[j+1]][k] <= max_degree) &&  detect_range[now_num[j+1]][k] != 0)
					{
						maybe_obstacle_degree[j][i] = detect_degree[now_num[j+1]][k];
						maybe_obstacle_range [j][i] = detect_range [now_num[j+1]][k];
					}
				}
			}
		}
	}
	//第二次，比较得到的中间结果
	for(int i = 0; i < 10; i++)
	{
		if(maybe_obstacle_range[0][i] != 0)	//有效距离和角度
		{
			for(int k = 0; k < 10; k++)
			{
				if((abs(maybe_obstacle_degree[0][i] - maybe_obstacle_degree[1][k]) <= max_degree || maybe_obstacle_degree[0][i] + maybe_obstacle_degree[1][k]) &&  maybe_obstacle_range[1][k] != 0)
				{
					
					real_obstacle_degree[i] = maybe_obstacle_degree[1][k];
					real_obstacle_range[i]  = maybe_obstacle_range[1][k];
				}
			}
		}
	}
	for(int i = 0; i < 2; i++)
	{
		for(int j = 0; j < 10; j++)
		{
			maybe_obstacle_degree[i][j] = 0;
			maybe_obstacle_range [i][j] = 0;
		}
	}
	for(int i = 0; i < 10; i++)
	{
		for(int k = i + 1; k < 10; k++)
		{
			if(abs(real_obstacle_degree[i] - real_obstacle_degree[k]) < 0.005 && real_obstacle_degree[i] != 0 && real_obstacle_degree[k] != 0)
			{
				real_obstacle_degree[k] = 0;
				real_obstacle_range [k] = 0;
			}
		}
	}
	//显示障碍物的位置
	for(int i = 0; i < 10; i++)
	{	
		if(real_obstacle_range[i] != 0)
		{
			ROS_INFO("obstacle_position: [%f, %f]", real_obstacle_degree[i], real_obstacle_range[i]);
			real_obstacle_num++;
			init_log<<tic()<<"\t"<<real_obstacle_degree[i]<<"\t"<<real_obstacle_range[i]<<endl;
		}
		real_obstacle_range [i] = 0;
		real_obstacle_degree[i] = 0;
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
                figurer.at<uchar>(x,y) = 0;
                picture.at<Vec3b>(x,y)[0] = 0;
				picture.at<Vec3b>(x,y)[1] = 0;
				picture.at<Vec3b>(x,y)[2] = 0;
            }
        	//figurer=Scalar(100);
        }
		if(is_obstacle[i]==1)
		{
			int x = 501+ceil(50*ranges_group[i]*cos(pi * degree_group[i] / 180));
			int y = 501+ceil(50*ranges_group[i]*sin(pi * degree_group[i] / 180));
        	//cout << x<<"\t"<<y<< endl;
        	if(x<=1000&x>=0&y<=1000&y>=0)
            {
                picture.at<Vec3b>(x,y)[0] = 255;
				picture.at<Vec3b>(x,y)[1] = 0;
				picture.at<Vec3b>(x,y)[2] = 0;
				//cv::circle(picture,cvPoint(x,y),10,CV_RGB(0,255,255),2,8,0);
            }
		}
        figurer.at<uchar>(501,501) = 0;
        figurer.at<uchar>(501,500) = 0;
        figurer.at<uchar>(501,502) = 0;
        figurer.at<uchar>(500,501) = 0;
        figurer.at<uchar>(500,500) = 0;
        figurer.at<uchar>(500,502) = 0;
        figurer.at<uchar>(502,501) = 0;
        figurer.at<uchar>(502,500) = 0;
        figurer.at<uchar>(502,502) = 0;
        
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
    init_log.open("/home/hitcsc/catkin_ws/log/test_client/lidar.txt");
	if(init_log.is_open())
	{
		cout << "Log initation success"<< endl;
	}
	else
	{
		cout << "Log error"<< endl;
	}
	
	ros::init(argc, argv, "rplidar4");
    ros::NodeHandle n;
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
                break;
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
