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
#include <rosgraph_msgs/Clock.h>
#define RAD2DEG(x) ((x)*180./M_PI)
using namespace std;
ofstream init_log;
rosgraph_msgs::Clock clocks;
ros::Time time2;
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ros::Time time = scan->header.stamp;
    ros::Time time1 = ros::Time::now();
    float dt = (time2-time).toSec();
    ROS_INFO("Time error is : %f,%d",dt,scan->header.seq);
    init_log<< dt<<"\t"<<scan->header.seq<<endl;
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    //time = 0.;
}

void clockCallback(const rosgraph_msgs::Clock time)
{
    time2 = time.clock;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;
    init_log.open("/home/hitcsc/catkin_ws/log/test_client/puredelay.txt");
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    ros::Subscriber clock_sub = n.subscribe<rosgraph_msgs::Clock>("/clock",100,clockCallback);
    ros::spin();

    return 0;
}
