#ifndef ANGLEID_H
#define ANGLEID_H

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#define PI 3.1415926

class angleID
{

private:
	int max; //the maximum number of container
	int num; //the valid number of data

	bool valid; //is the data flooding

	std::vector<Point2f> mem; //the container

	ros::Time current_time; // the time of now
	ros::Time last_time; // the time of update last time
	int no_num;

	double duration;
	int missing_num;

	ros::Time current_time_ir; // the time of now
	ros::Time last_time_ir; // the time of update last time
	bool update_flag;

	float r_min; // relativity
public:
	float angle; //the motion-direction

	float positive_ang;
	float negative_ang;

	int positive_num;
	int negative_num;
	double missing_time;

	int missing_num_max;
	int missing_num_min;

	Point2f target;

	angleID() :max(20), num(0),r_min(0.1), duration(1.0)
	{
		no_num = 0;

		mem.resize(max);
		angle = 10.;

		update_flag = false;
		//current_time_ir = last_time_ir = ros::Time::now();

		positive_ang = 0.;
		negative_ang = 0.;

		missing_time = 2.;

		positive_num = 0;
		negative_num = 0;
		missing_num = missing_num_max = 15;
		missing_num_min = 10;
	}
	~angleID(){};

	void initialize(); 
	void update(float air_pos[], Point2f point, bool flag); //to get measure data
	bool estimate(); //to estimate the angle
	void memClear(); // clear mem

	float sumPow(std::vector<Point2f>& mem);
	float sumMultiply(std::vector<Point2f>& mem);
	float sum(std::vector<Point2f>& mem, bool flag); //bool = true then sum_x && bool = false then sum_y

	float avg(std::vector<Point2f>& mem, bool flag);
	float sumDiff(std::vector<Point2f>& mem, bool flag);
	float sumDiff2(std::vector<Point2f>& mem, bool flag);

	bool filter(Point2f& point, float& theta, bool flag);
};

#endif
