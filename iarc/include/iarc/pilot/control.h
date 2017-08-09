/****************************************
*** Copyright@HITCSC Team 2017
*** Filename control.h
*** Author Leo Wang
*** Version 0.0.1
*** Created at 2017/04/03
*** Discription:    This program is mainly used to load parameters from cfg files. This program 
*** Fuction list:   
*** History:        Leo Wang 2017/04/03 0.0.1 created
*****************************************/
#ifndef CONTROL_L_
#define CONTROL_L_

void control();
int decision();
struct obstacle{};

struct _attitude_command{
    float roll;
    float pitch;
    float vh;
    float dyaw;
};

struct _velocity_command{
    float vx;
    float vy;
    float vh;
    float dyaw;
};

struct _patrol				//define patrol_parameters
{
	int i;			
	double angle;
};

struct _Expect{
    float x;
    float y;
    float h;
    float yaw;
};

struct _wnypatrol{
    //二维巡航速度
    float tar_vx;
    float tar_vy;
    float tar_velocity;
    double theta;       //方向角区间为[-π,π)，初始值0
    //飞行器速度，加速度与速度方向
    float quad_vx;
    float quad_vy;
    float quad_vz;
    float quad_vyaw;
    float quad_vx_last;
    float quad_vy_last;
    float quad_ax;
    float quad_ay;
    double quad_theta;
    //速度限幅与加速度限幅
    float quad_vx_limitation;
    float quad_vy_limitation;
    float quad_vz_limitation;
    float quad_vyaw_limitation;

};


struct _select_tar{
    float x;
    float y;
    float dirx;
    float diry;
    int color; // 1 is red, 2 is green 
};
struct cmp
{
	double d_pos;
	double dir;
	
	int id;
};
#endif