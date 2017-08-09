#include "iarc007/mjh_pos/estimater.h"

void estimater::initialize()
{
	//last_time = current_time = ros::Time::now();
	
	a_x = a_y = a_z = 0.;
	v_x = v_y = v_z = 0.;
	dp_x = dp_y = dp_z = 0.;
	
	vo_x = vo_y = vo_z = 0.;
	
	start = true;
}

void estimater::estimate()
{
	dp_x = v_x*(current_time.toSec()-last_time.toSec()) + 0.5*a_x*(current_time.toSec()-last_time.toSec())*(current_time.toSec()-last_time.toSec());
	dp_y = v_y*(current_time.toSec()-last_time.toSec()) + 0.5*a_y*(current_time.toSec()-last_time.toSec())*(current_time.toSec()-last_time.toSec());
					
	v_x += a_x*(current_time.toSec()-last_time.toSec());	
	v_y += a_y*(current_time.toSec()-last_time.toSec());
		
	last_time = current_time;
}

void estimater::vo_update(int method,float k)
{
	assert((k >= 0) && (k <= 1) && (method > 0));
	
	//current_time = ros::Time::now();
	
	if(method == 1){
		v_x = vo_x;
		v_y = vo_y;	
	}
	else if(method == 2){
		v_x = k*vo_x + (1-k)*v_x;
		v_y = k*vo_y + (1-k)*v_y;
	}
	else{

	}
}
