#ifndef ESTIMATER_H_
#define ESTIMATER_H_

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <cassert>

using namespace std;

class estimater
{
	private:
		

	public:

		ros::Time last_time;
		ros::Time current_time;
		
		float a_x, a_y, a_z;
		float v_x, v_y, v_z;
		float dp_x, dp_y, dp_z;
		
		bool start;
	
		float vo_x, vo_y, vo_z;
		
		estimater()
		{
			start = false;
		}
	
		~estimater()
		{
	
		}
	
		void initialize(); // initialize accelerometer|velocity|position|
		void estimate(); //velocity || position estimate
		void vo_update(int method,float k);  //method: 1--believe optical flow 2--fuse by k
};


#endif
