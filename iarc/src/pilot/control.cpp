#include <fstream>
#include <sstream>
#include <iostream>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk.h>
#include "iarc/pilot/control.h"
#include "iarc/obstacle.h"
#include "iarc/vehicle_pos.h"
#include "iarc/pilot/subscribe.h"
#include "iarc/pilot/pid_controller.h"
#include <ros/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iarc/pilot/XU_LADRC.h>
#include "iarc/pilot/target_processor.h"

#include <vector>

using namespace std;
using namespace DJI::onboardSDK;


/****
This file should include two function 
1. deciding whether to interact or not
2. to generate and send a serial of command [roll,pitch,yaw',z']
*****/

// RVIZ flags and variables begins



// RVIZ flags and variables ends




// Flags starts here
bool patrol_random=1;           // 1 denotes random mode which rely on no positioning imformation
                                // 0 denotes squaring mode which rely on the positioning imformation
bool attitude_control=false;    // 1 denotes attitude control
                                // 0 denotes velocity control

bool pid_flag=true;             // 1 denotes pid control
                                // 0 denotes LQR control


bool velocity_track=1;          //  1 denotes using veocity control to track (AKA h >1.0m)
                                //  0 denotes using attitude control to track (AKA h <1.0m)
                                //  A schmitt trigger should be designed here, the range is set as 0.9-1.0

bool trust_guidance=0;
int global_status = 0;          // 0 denotes beginning
                                // 1 denotes 45     degree interacting
                                // 2 denotes 180    degree intercating
                                // 3 dentoes patroling 
                                // 4 denotes asending

// Flags ends here



/*****
attitude test 
*********/

// target variables starts

uint chosed_target = 0;
int lost_target_counter = 0;        // if target is lost, continue the previous routine for 10 frames
float target_dir = 0.;

int target_number;
int obstacle_number;
float Vkf=0.,Vkf1=0.;

int choosed_target = 0;
// target variables ends


// 180 interact variables ends

float timer_180_1=0.;               // timer_1 for 180 interact
float timer_180_2=0.;               // timer_2 for 180 interact
int stage_180=0;                    // 
int counter_180_2=0;
float tar_x_180=0.01;
float tar_y_180=0.01;
float last_tar_x = 0;
float last_tar_y = 0;
// 180 interact variables ends

// 45 interact variables ends

float timer_45_1=0.;                // timer_1 for 45 interact
float timer_45_2=0.;                // timer_2 for 45 interact
int stage_45=0;                     // 
int counter_45_2=0;
float tar_x_45=0.01;
float tar_y_45=0.01;
float ul_ref = 1.2;
float bar_ref = 0;
int descend_counter = 0;
// 45 interact variables ends


// ascend variables ends

int ascend_mode=1;

// ascend variables ends


//main strategy variables ends
int mode_45_count=0;
int mode_180_count=0;

//main strategy variables ends



// patrol variables starts 

//二维巡航速度
float wny_tar_vx=0.0;
float wny_tar_vy=0.0;
float wny_tar_velocity=0.25;
double wny_theta=0.0;		//方向角区间为[-π,π)，初始值0
//飞行器速度，加速度与速度方向
float wny_quad_vx_last=0.0;
float wny_quad_vy_last=0.0;
float wny_quad_ax=0.0;
float wny_quad_ay=0.0;
double wny_quad_theta=0.0;
//速度限幅与加速度限幅
float wny_quad_vx_limitation=0.45;
float wny_quad_vy_limitation=0.45;
float wny_quad_vz_limitation=0.3;
float wny_quad_vyaw_limitation=45.0;
float wny_quad_ax_limitation=1.2;
float wny_quad_ay_limitation=1.2;
//边界检测
int wny_side_check=4;		//0表示绿边，1表示右白边，2表示红边，3表示左白边（顺时针），4表示无效标志位
int wny_side_corner=2;		//0表示在边附近，1表示在角附近，2表示既不在边附近又不在角附近
int wny_side_flag=0;		//0表示不在边界附近，1表示在边界附近(1条边界)，2表示在边界附近(2条边界)，假设在边界附近已经有距离阈值，也就是距离缩短到某一范围以内，从0变为1或2




// patrol variables ends

// height flags starts

float delta_baro= 0.;
float last_baro = 0.;
float ultra_threshold = 1.;
float fused_ul  = 0.;
float altitude=0,last_altitude=0;
float height_quad = 1.2;
float last_ul   = 0.;
// height flags ends


// test variable starts

int ascend_counter = 0;

// test variable ends

// track variable starts
int track_stage = 0;

// track variable ends



extern CvMat *att_init;
extern CvMat *R_init;
extern CvMat *R_init_i;

extern CvMat *q_right_now;
extern CvMat *att_right_now;
extern CvMat *R_now_i;

extern CvMat *q;
extern CvMat *atti;

extern iarc::obstacle store;
extern int control_mode;
extern int control_mode_previous;
int control_mode;
int control_mode_previous=-100;

_attitude_command attitude_command;
_attitude_command attitude_arena;
_attitude_command attitude_ground;

_velocity_command velocity_command;
_velocity_command velocity_arena;
_velocity_command velocity_ground;

_select_tar select_tar;

_patrol patrolr={0,0};

unsigned int period = 0;


/***
test flags
***/
float tar_x=0.,tar_y=0.;
// float tar_x1=0.,tar_y1=0.;
float img_x = 0;
float img_y = 0;
float dx=0;
float dy=0;
float dz = 0.;
float vx=0;
float vy=0;
float last_vx=0;
float last_vy=0;
float last_dx=0;
float last_dy=0;
float last_last_dx=0;
float last_last_dy=0;
float tar_dx=0;
float tar_dy=0;
float time0000=0.;
float compens_vx=0,compens_vy=0;

float world_ax=0,world_ay=0,world_az=0,world_vx=0,world_vy=0,world_vz=0;
float obs_dx = 0;
float obs_dy = 0;
bool fisrtshift=1;

int quad_flag = 0;


/*******
test flag ends
*******/


int sgn(float d){ return d<-0.?-1:d>0.; }

int decision(int distance)
{
    cout<<distance<<endl;
    return 0;
}
int patrol_stage=0;
double patrol_time=0.;
iarc::vehicle_pos position;

pid_ctrl pid_x,pid_y,pid_z,pid_yaw,pid_pitch,pid_roll,pid_relative_x,pid_relative_y;
target_processor t_p;
//pid_ctrl


// xxw target start 

int interact_select_locate1(float target_angle, float target_x)
{
/*无时钟信息，1区域，此时看到左边界，通过目标的横坐标与运动方向，判断应采取的交互方式
  interact_select = 0:巡航
					1:跟随
					2:45°
					3:180°
*/
	float pi = 3.14;
	float expected_angle;
	int interact_select;
	float t_45;
	float t_180;
	
	if ((target_angle >= -pi*3/4 -expected_angle) && (target_angle <= -pi/2 + expected_angle))
	{
		if (target_x + 0.33*t_45*cos(target_angle) <= 0)
		{
			interact_select = 0;
		}
		else
		{
			interact_select = 1;
		}
	}
	else if ((target_angle > -pi/2 +expected_angle) && (target_angle <= pi/2 - expected_angle))
	{
		interact_select = 1;
	}
	else
	{
		if (target_x + 0.33*t_180*cos(target_angle) <= 0)
		{
			interact_select = 0;
		}
		else
		{
			interact_select = 3;
		}
	}
	return interact_select;
}

int interact_select_locate2(float target_angle)
{
/*无时钟信息，2区域，此时看不到左右边界，通过目标的运动方向，判断应采取的交互方式
  interact_select = 0:巡航
					1:跟随
					2:45°
					3:180°
*/
	float pi = 3.14;
	float expected_angle;
	int interact_select;
	float t_45;
	float t_180;
	
	if ((target_angle >= -pi/2 - expected_angle) && (target_angle <= -pi/4 + expected_angle))
	{
		interact_select = 1;
	}
	else if ((target_angle < -pi/2 - expected_angle) || (target_angle > pi/2 + expected_angle))
	{
		interact_select = 2;
	}
	else
	{
		interact_select = 3;
	}
	return interact_select;
}
int interact_select_locate3(float target_angle, float target_x)
{
/*无时钟信息，3区域，此时能看到右边界，通过目标的横坐标与运动方向，判断应采取的交互方式
  interact_select = 0:巡航
	                1:跟随
	                2:45°
	                3:180°	
*/
	float pi = 3.14;
	float expected_angle;
	int interact_select;
	float t_45;
	float t_180;

	if ((target_angle >= -pi/2 - expected_angle) && (target_angle <= -pi/2 + expected_angle))
	{
		if (target_x + 0.33*t_180*cos(target_angle) >= 20)
		{
			interact_select = 0;
		}
		else 
		{
			interact_select = 1;
		}
	}
	else if ((target_angle > -pi/2 + expected_angle) && (target_angle <= pi/2 + expected_angle))
	{
		if (target_x + 0.33*t_180*cos(target_angle) >= 20)
		{
			interact_select = 0;
		}
		else
		{
			interact_select = 3;
		}
	}
	else
	{
		interact_select = 2;
	}
	return interact_select;
}

// xxw target end

//***************houy
bool comp(const cmp &a, const cmp &b)
{
	return a.d_pos<b.d_pos;
}

int hou_track(_select_tar select_tar,float testttt,Vector<_select_tar> input_track)
{

    float scale = 2;
	float distance1 = scale*1.0/20.0;//移动范围,目标位移小于1m,频率20hz

    Vector<cmp> descript1;
	cmp descript_t;
    for (int i = 0; i < input_track.size(); i++)
	{
		//d_pos.push_back((img_point[i].x - pre_pos.x)*(img_point[i].x - pre_pos.x) + (img_point[i].y - pre_pos.y)*(img_point[i].y - pre_pos.y));
		if (select_tar.color == input_track[i].color)
		{
			if ((input_track[i].x - select_tar.x)*(input_track[i].x - select_tar.x) + (input_track[i].y - select_tar.y)*(input_track[i].y - select_tar.y) < distance1)
			{
				//match1.push_back(i);
                
				descript_t.d_pos = (input_track[i].x - select_tar.x)*(input_track[i].x - select_tar.x) + (input_track[i].y - select_tar.y)*(input_track[i].y - select_tar.y);
				descript_t.dir = fabs(input_track[i].dirx * select_tar.dirx + input_track[i].diry * select_tar.diry);
				//descript_t.y_dir = y_dir[i];
				descript_t.id = i;
				descript1.push_back(descript_t);
			}
		}

	}
    if (descript1.size() > 0)
	{
		sort(descript1.begin(), descript1.end(), comp);
		//*******************
		for (int i = 0; i < descript1.size(); i++)
		{
			if (descript1[i].dir < 1 / 2)
			{
				return descript_t.id;
			}
		}
	}
	else
	{
		cout << "no track" << endl;
	}

    return -1;
}

void stop()
{
    velocity_command.vx = 0.;
    velocity_command.vy = 0.;
    velocity_command.vh = 0.;
    velocity_command.dyaw =0.;
    attitude_control = false;
    return;
}

int patrol(dji_sdk::Velocity velocity,dji_sdk::AttitudeQuaternion attitude_quaternion,iarc::vehicle_pos pose,float delta_h/**position*/)
{
    dji_sdk::Velocity velo = velocity;
    cvmSet(q, 0, 0, attitude_quaternion.q0);
	cvmSet(q, 1, 0, attitude_quaternion.q1);
	cvmSet(q, 2, 0, attitude_quaternion.q2);
	cvmSet(q, 3, 0, attitude_quaternion.q3);
    Quaternion_To_Euler(q,atti);

    int re=0;
    switch(patrol_stage)
    {
        case 0:
        {
            patrol_time=tic();
            patrol_stage=1;
            break;
        }
        case 1:
        {
            if(tic()-patrol_time>=2.)
            {
                patrol_stage = 2;
            }
            if(cvmGet(atti,2,0)>=2.82&&cvmGet(atti,2,0)<=2.80)
            {
                velocity_command.vx = 0.5;
                velocity_command.vy = 0.;
                velocity_command.vh = 0.2*delta_h;
                velocity_command.dyaw =0.;
                attitude_control = false;
                //if(velo.vx)
            }
            else
            {
                velocity_command.vx = 0.;
                velocity_command.vy = 0.;
                velocity_command.vh = 0.2*delta_h;
                velocity_command.dyaw =(2.82-cvmGet(atti,2,0))*0.4;
                attitude_control = false;
            }

            break;
        }
        case 2:
        {
            re=1;
            break;
        }
    }
    return re;
}

void patrol_r(bool side_one, bool side_two, float angle_wall, float angle_quad,float delta_h)
{
	//Integral_reset();

	float angle_error=fabs(angle_wall-angle_quad);
	if(angle_error<1.57)
	{
		angle_error=3.14-angle_error;
	}
	else
	{
	}
	
	if(side_one==1&&side_two==0)	//side_one before side_two
	{
		patrolr.i++;
		if(patrolr.i>10)
		{
			patrolr.i=0;
			patrolr.angle=patrolr.angle+1.57+0.5*angle_error;
			printf("one wall!\n");
		}
		else
		{
			printf("patrolling!\n");
		}
	}
	else if(side_one==1&&side_two==1)
	{
		patrolr.i++;
		if(patrolr.i>10)
		{
			patrolr.i=0;
			patrolr.angle=patrolr.angle+3.14;
			printf("two wall!\n");
		}
		else
		{
			printf("patrolling!\n");
		}
	}
	else
	{
		printf("patrolling!\n");
	}

    velocity_command.vh     = 0.4*delta_h;
    velocity_command.vx     = 0.25*cos(patrolr.angle);
    velocity_command.vy     = 0.25*sin(patrolr.angle);
    velocity_command.dyaw   = 0.;
    attitude_control = false;
    // TODO: yaw.kp_vx=0.;
}


void ascend( float current_height, int ascend_mode,float tar_x=0.001,float tar_y=0.001)
{
    //This fucntion is used to control the drone to a specific height
    // define ascend_mode   1: back to patrol height 
    //                      2: Ascend till the target is in sight
    //                      3: Avoid the obstacle, emergent
    //                      *4: Avoid the obstacle, normal ( not to consider)
    
    switch(ascend_mode)
    {
        case 1:
        {
            /*if(1.3-current_height>=0.5)
                velocity_command.vh = 0.3*(1.3-current_height);
            else
                velocity_command.vh = 0.4*(1.3-current_height);*/
            velocity_arena.vh     = pid_z.out(1,current_height,0);
            velocity_arena.vx     = 0.;
            velocity_arena.vy     = 0.;
            velocity_arena.dyaw   = 0.;
            attitude_control = false;
            break;
        }

        case 2:
        {
            velocity_arena.vh = 0.1*(1.3-current_height);
            if((tar_x*tar_x+tar_y*tar_y)>0.1)
            {
                velocity_arena.vx     = 0.;
                velocity_arena.vy     = 0.;
            }
            else
            {
                velocity_arena.vx     = 0.33*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y);
                velocity_arena.vy     = 0.33*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y);
            }
            velocity_arena.dyaw   = 0.;
            // cout << "vvh="<<velocity_arena.vh;
            attitude_control = false;
            break;
        }

        case 3:         // for 180 interaction
        {
            velocity_arena.vh     = pid_z.out(1.3,current_height,0);
            if((tar_x*tar_x+tar_y*tar_y)>0.1 && current_height<=0.5)
            {
                
                velocity_arena.vx     = 0.33*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y);
                velocity_arena.vy     = 0.33*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y);
            }
            else
            {
                velocity_arena.vx     = 0.;
                velocity_arena.vy     = 0.;
            }
            velocity_arena.dyaw   = 0.;
            attitude_control = false;
            break;
        }
        
        case 4:
        {
            velocity_arena.vh     = pid_z.out(0.2,current_height,0);
            velocity_arena.vx     = 0.;
            velocity_arena.vy     = 0.;
            velocity_arena.dyaw   = 0.;
            // cout << "vvh="<<velocity_arena.vh;
            attitude_control = false;
            break;
        }

        default:
        {
            cout << "error"<< endl;
            break;
        }
    }
    //cout << current_height<<"\t"<<velocity_command.vh <<endl;
}


void safe_patrol(float x,float y,float obs_x,float obs_y,float h,float h_ideal)
{

    float xd = -1*obs_y/sqrt(obs_x*obs_x+obs_y*obs_y);
    float yd = obs_x/sqrt(obs_x*obs_x+obs_y*obs_y);
    if(x*xd+y*yd<=0)
    {
        xd = -xd;
        yd = -yd;
    }
    velocity_arena.dyaw   = 0.;
    velocity_arena.vx       = 0.3* xd;
    velocity_arena.vy       = 0.3* yd;
    velocity_arena.vh       = pid_z.out(h_ideal,h,0);
    attitude_control = false;

}


void track(float x,float y,float vx_r,float vy_r,float h_now,float h_ideal,bool att_track,float tar_x=0.001,float tar_y=0.001)
{
    if(x==-20)
    {
        stop();
        return;
    }
    if(att_track) // Attitude control
    {
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        attitude_control=true;
        attitude_command.dyaw   = 0.;
        attitude_arena.vh       = pid_z.out(h_ideal,h_now,0);
        //cout << attitude_arena.vh << "\t" << h_now << "\t" << h_ideal <<endl;
        velocity_arena.vx       = pid_relative_x.out(0,x,1);//pid_x.out(0,x,1);
        velocity_arena.vy       = pid_relative_y.out(0,y,1);//pid_y.out(0,y,1); 
        attitude_arena.pitch =   pid_pitch.out(velocity_arena.vx,vx_r,0);          // velocity from vision
        attitude_arena.roll  =   pid_roll.out(velocity_arena.vy,vy_r,0);           //当前速度可以用1.guidance读取2.位置差分
        //cout << vx_r << "\t" << vy_r << "\t";
        //cout << velocity_arena.vx << "\t" << velocity_arena.vy << "\t";
    }
    else//输出速度指令
    {
        velocity_arena.dyaw   = 0.;
        if(track_stage==0)
        {
            if(abs(compens_vx)<=0.2 && abs(compens_vy)<=0.2 && sqrt(compens_vx*compens_vx+compens_vy*compens_vy)<0.20)
            {
                velocity_arena.vx       = pid_x.out(0,x,1)+0*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y);
                velocity_arena.vy       = pid_y.out(0,y,1)+0*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y);
                track_stage             = 0;            
            }
            else
            {
                velocity_arena.vx       = pid_x.out(0,x,1)+0.33*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y);
                velocity_arena.vy       = pid_y.out(0,y,1)+0.33*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y);
                track_stage             = 1;              
            }
        }
        else
        {
            if(abs(compens_vx)<=0.1 && abs(compens_vy)<=0.1 && sqrt(compens_vx*compens_vx+compens_vy*compens_vy)<0.1)
            {
                velocity_arena.vx       = pid_x.out(0,x,1)+0*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y);
                velocity_arena.vy       = pid_y.out(0,y,1)+0*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y);
                track_stage             = 0;            
            }
            else
            {
                velocity_arena.vx       = pid_x.out(0,x,1)+0.33*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y);
                velocity_arena.vy       = pid_y.out(0,y,1)+0.33*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y);
                track_stage             = 1;              
            }
        }
        

        velocity_arena.vh       = pid_z.out(h_ideal,h_now,0);
        attitude_control = false;
    }
     
}


void inter_45(float x,float y,float vx_r,float vy_r,float h_now,float h_ideal,bool att_track,float tar_x,float tar_y)
{
    if(x==-20)
            return;
    if(att_track && target_number>=1) // Attitude control
    {
        if(stage_45==0 || stage_45==1)
        {
            attitude_control=true;
            attitude_command.dyaw   = 0.;
            attitude_arena.vh       = pid_z.out(h_ideal,h_now,0);
            //cout << attitude_arena.vh << "\t" << h_now << "\t" << h_ideal <<endl;
            velocity_arena.vx       = pid_relative_x.out(0,x,1);//pid_x.out(0,x,1);
            velocity_arena.vy       = pid_relative_y.out(0,y,1);//pid_y.out(0,y,1); 
            attitude_arena.pitch =   pid_pitch.out(velocity_arena.vx,vx_r,0);          // velocity from vision
            attitude_arena.roll  =   pid_roll.out(velocity_arena.vy,vy_r,0);           //当前速度可以用1.guidance读取2.位置差分
        }
        else
        {
            attitude_control=true;
            attitude_command.dyaw   = 0.;
            attitude_arena.vh       = pid_z.out(h_ideal,h_now,0);
            //cout << attitude_arena.vh << "\t" << h_now << "\t" << h_ideal <<endl;
            velocity_arena.vx       = pid_relative_x.out(0,x,1);
            velocity_arena.vy       = pid_relative_y.out(0,y,1);
            attitude_arena.pitch =   pid_pitch.out(velocity_arena.vx,vx_r,0);          // velocity from vision
            attitude_arena.roll  =   pid_roll.out(velocity_arena.vy,vy_r,0);           //当前速度可以用1.guidance读取2.位置差分
        }
        
        //cout << vx_r << "\t" << vy_r << "\t";
        //cout << velocity_arena.vx << "\t" << velocity_arena.vy << "\t";
    }
    else if(target_number>=1)//输出速度指令
    {
        attitude_control=false;
        if(stage_45==0 || stage_45==1 || stage_45==2)
        {
            velocity_arena.dyaw     = 0.;
            velocity_arena.vh       = pid_z.out(h_ideal,h_now,0);
            velocity_arena.vx       = pid_x.out(0,x,1)+0.33*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y);
            velocity_arena.vy       = pid_y.out(0,y,1)+0.33*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y);
        }
        else
        {
            velocity_arena.dyaw     = 0.;
            velocity_arena.vh       = pid_z.out(h_ideal,h_now,0);
            velocity_arena.vx       = 0.33*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y);
            velocity_arena.vy       = 0.33*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y);

        }
    }
    else
    {
        return;
    }

    if(h_now<=1.0 && abs(x)<=0.07 && abs(y)<=0.07 && stage_45==1)// && abs(vx_r)<0.1 && abs(vy_r)<0.1)
    {
        stage_45    = 2;
        ul_ref = h_now;
        bar_ref = altitude;
    }
    else if(stage_45==2 && h_now<=0.4 && abs(x)<=0.03 && abs(y)<=0.03 )//target_number==0)
    {
        stage_45    = 3;
        tar_x_45    = tar_x;
        tar_y_45    = tar_y;
    }
    else if (stage_45==2 && world_az >1.5)
    {
        stage_45 = 4;
        control_mode=5;
    }
    // else if(h_now<=h_ideal)// || world_az >0.5)
    // {
    //     stage_45 = 0;
    //     control_mode=5; 
    // }
    return;
}

void descend(float tar_x_f,float tar_y_f,float az,float v_h)
{
    descend_counter++;
    if(descend_counter<=100 && az<=1.3)
    {
        attitude_control=false;
        velocity_arena.dyaw     = 0.;
        velocity_arena.vh       = -1*v_h;
        velocity_arena.vx       = 0.33*tar_x_f/sqrt(tar_x_f*tar_x_f+tar_y_f*tar_y_f);
        velocity_arena.vy       = 0.33*tar_y_f/sqrt(tar_x_f*tar_x_f+tar_y_f*tar_y_f);
        stage_45 = 3;
    }
    else
    {
        stage_45 = 4;
        control_mode=5;
        descend_counter=0;
    }
}

void interact_180_noobs( float x,float y,float vx_r,float vy_r, float h_now,float h_ideal,bool att_ctrl,float tar_x,float tar_y)
{
    if(att_ctrl && target_number>=1)
    {
        if(x==-20)
            return;
        // cout << - endl;
        if(h_now>=h_ideal || abs(x) >= 0.07 || abs(y)>0.07 || abs(vx_r)>0.1 || abs(vy_r)>0.1)  // Attitude control
        {
                attitude_control=true;
                stage_180               = 1;
                attitude_command.dyaw   = 0.;
                attitude_arena.vh       = pid_z.out(h_ideal,h_now,0);
                velocity_arena.vx       = pid_relative_x.out(0,x,1);//pid_x.out(0,x,1);
                velocity_arena.vy       = pid_relative_y.out(0,y,1);//pid_y.out(0,y,1); 
                attitude_arena.pitch =   pid_pitch.out(velocity_arena.vx,vx_r,0);          // velocity from vision
                attitude_arena.roll  =   pid_roll.out(velocity_arena.vy,vy_r,0); 
                // cout << "vr " << vx_r << " "<< vy_r<< endl;
        }
        else
        {
            cout << "stage 1 ends"<< endl;
            stage_180               = 2;
            timer_180_1             = tic();
            tar_x_180 = tar_x;
            tar_y_180 = tar_y;
            ul_ref = h_now;
            bar_ref = altitude;
        }
    }
    else if( !att_ctrl && target_number>=1)
    {
        if(x==-20)
            return;
        attitude_control=false;
        x -= 0.3*tar_x;
        y -= 0.3*tar_y;
        if(h_now>=h_ideal+0.05 || abs(x) >= 0.07 || abs(y)>0.07) // Attitude control
        {
            velocity_arena.dyaw     = 0.;
            velocity_arena.vh       = pid_z.out(h_ideal,h_now,0);
            velocity_arena.vx       = pid_x.out(0,x,1)+0.33*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y);
            velocity_arena.vy       = pid_y.out(0,y,1)+0.33*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y);
            cout <<"vx"<< velocity_arena.vx <<"\t"<<"vx"<< velocity_arena.vy << endl;
            stage_180               = 1;
        }
        else
        {
            cout << "stage 1 ends"<< endl;
            stage_180               = 2;
            timer_180_1             = tic();
            tar_x_180 = tar_x;
            tar_y_180 = tar_y;
            ul_ref = h_now;
            bar_ref = altitude;
        }
        
    }
    else
    {
        ;
    }
   
}


void interact_180_noobs(float h,float h_ideal,float tar_x,float tar_y,bool att_ctrl )
{
    counter_180_2++;
    if(stage_180==2 && counter_180_2<=60)
    {
        velocity_arena.dyaw     = 0.;
        velocity_arena.vh       = pid_z.out(h_ideal,h,0);
        velocity_arena.vx       = 0.5*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y);
        velocity_arena.vy       = 0.5*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y); // original velocity is 0.7m/s
        attitude_control=false;
    }
    else 
    {
        if(stage_180==3 && counter_180_2>=200)
        {
            global_status       = 4;
            control_mode        = 5;
            ascend_mode         = 3;
            stage_180           = 0;
            counter_180_2       = 0;

        }
        velocity_arena.dyaw     = 0.;
        velocity_arena.vh       = pid_z.out(h_ideal,h,0);
        velocity_arena.vx       = 0;
        velocity_arena.vy       = 0;
        stage_180               = 3; 
        attitude_control=false;

    }
}

void interact_45_noobs( float x,float y,float h,float h_ideal,float direction,float dir_x,float dir_y )
{
    attitude_control=false;
    //cout << h-h_ideal << "\t" << sqrt(x*x+y*y) << endl;
    if(h>=h_ideal || sqrt(x*x+y*y) >= 0.1) // Attitude control
    {
        velocity_arena.dyaw     = 0.;
        velocity_arena.vh       = pid_z.out(h_ideal,h,0);
        velocity_arena.vx       = pid_x.out(0,x,1);
        velocity_arena.vy       = pid_y.out(0,y,1);
        if(sqrt(x*x+y*y) >= 0.2)
        {
            velocity_arena.vy       = -1*velocity_arena.vy+0.2*dir_y;
            velocity_arena.vx       = -1*velocity_arena.vx+0.2*dir_x;
        }
        else
        {
            velocity_arena.vy       = -1*velocity_arena.vy;//+0.3*dir_y;
            velocity_arena.vx       = -1*velocity_arena.vx;//+0.3*dir_x;
        }
        
        if(h<=0.3)
        {
            stage_45               = 2;
        }
        //cout <<"vx"<< velocity_arena.vx <<"\t"<<"vx"<< velocity_arena.vy << endl;
        
        // stage_45                = 1;
    }
    else if(stage_45==1)
    {
        cout << "stage 1 ends"<< endl;

        stage_45               = 2;
        timer_45_1             = tic();
    }
    else if(stage_45==2)
    {
        velocity_arena.dyaw     = 0.;
        velocity_arena.vh       = pid_z.out(h_ideal,h,0);
        velocity_arena.vx       = -1*pid_x.out(0,x,1)+0.2*dir_x;
        velocity_arena.vy       = -1*pid_y.out(0,y,1)+0.2*dir_y;
        global_status       = 4;
        control_mode        = 5;
        stage_45           = 0;
        counter_45_2       = 0; 
        //pid_z.clean();
    }
    else
    {
        global_status       = 4;
        control_mode        = 5;
        stage_45           = 0;
        counter_45_2       = 0;
    }
}


void interact_45_noobs(float h,float h_ideal,float direction,float dir_x,float dir_y )
{
    counter_45_2++;
    if(h>=h_ideal && stage_45==2 )
    {
        velocity_arena.dyaw     = 0.;
        velocity_arena.vh       = pid_z.out(h_ideal,h,0);
        velocity_arena.vx       = 0.0+0.3*dir_x;
        velocity_arena.vy       = 0.0+0.3*dir_y;
        //velocity_arena.vx       = 0.3*cos(direction);
        //velocity_arena.vy       = 0.3*sin(direction); 
        cout<<"bbb" << velocity_arena.vh;
    }
    else 
    {
        if(stage_45==2 )
        {
            global_status       = 4;
            control_mode        = 5;
            stage_45           = 0;
            counter_45_2       = 0;

        }
        velocity_arena.dyaw     = 0.;
        velocity_arena.vh       = pid_z.out(0.2,h,0);
        velocity_arena.vx       = pid_x.out(0,0,1);
        velocity_arena.vy       = pid_y.out(0,0,1); 
        cout << "aaa";
    }
}

float wny_patrol_angle(float wny_side_corner, float wny_side_stay, float wny_side_)
{
    if(wny_side_corner==0&&wny_side_stay==0)		//在边附近，角度保持标志位为0（清零），旋转角度，将角度保持标志位置1（保持），保持住旋转后的角度
	{
		            if(wny_side_check==0)		//绿边附近
		            {
    			        if(wny_quad_theta>=0&&wny_quad_theta<3.14)		//飞行器速度方向指向场内，不旋转
		    	        {
		    	        }
			            else if(wny_quad_theta>=-1.57&&wny_quad_theta<0)	//飞行器速度方向指向场外，x正向，角度加120度
			            {
			    	        wny_theta=wny_theta+2.09;
			            }
			            else if(wny_quad_theta>=-3.14&&wny_quad_theta<-1.57)	//飞行器速度方向指向场外，x负向，角度减120度
			            {
		    		        wny_theta=wny_theta-2.09;
		    	        }
		            }
		            else if(wny_side_check==1)	//右白边附近
		            {
			            if((wny_quad_theta>=-3.14&&wny_quad_theta<-1.57)||(wny_quad_theta<3.14&&wny_quad_theta>=1.57))		//飞行器速度方向指向场内，不旋转
			            {
			            }
			            else if(wny_quad_theta>=0&&wny_quad_theta<1.57)		//飞行器速度方向指向场外，y正向，角度加120度
			            {
				            wny_theta=wny_theta+2.09;
			            }
			            else if(wny_quad_theta>=-1.57&&wny_quad_theta<0)	//飞行器速度方向指向场外，y负向，角度减120度
			            {
				            wny_theta=wny_theta-2.09;
			            }
		            }
		            else if(wny_side_check==2)	//红边附近
		            {
			            if(wny_quad_theta>=-3.14&&wny_quad_theta<0)		//飞行器速度方向指向场内，不旋转
			            {
			            }
			            else if(wny_quad_theta>=0&&wny_quad_theta<1.57)		//飞行器速度方向指向场外，x正向，角度减120度
			            {
				            wny_theta=wny_theta-2.09;
			            }
			            else if(wny_quad_theta>=1.57&&wny_quad_theta<3.14)	//飞行器速度方向指向场外，x负向，角度加120度
		    	        {
				            wny_theta=wny_theta+2.09;
			            }
		            }
		            else if(wny_side_check==3)	//左白边附近
		            {
			            if(wny_quad_theta>=-1.57&&wny_quad_theta<1.57)		//飞行器速度方向指向场内，不旋转
			            {
			            }
			            else if(wny_quad_theta>=-3.14&&wny_quad_theta<-1.57)	//飞行器速度方向指向场外，y负向，角度加120度
			            {
				            wny_theta=wny_theta+2.09;
			            }
			            else if(wny_quad_theta>=1.57&&wny_quad_theta<3.14)	//飞行器速度方向指向场外，y正向，角度减120度
			            {
				            wny_theta=wny_theta-2.09;
			            }
		            }
		            wny_side_stay=1;
	            }
	            else if(wny_side_corner==1&&wny_side_stay==0)		//在角附近，角度保持标志位为0（清零），旋转角度，将角度保持标志位置1（保持），保持住旋转后的角度
	            {
		            wny_theta=wny_theta+3.14;	//反向移动，即旋转180度
		            wny_side_stay=1;
	            }
	            if(wny_side_corner==2&&wny_side_stay==1)		//当脱离边或角附近时（附近没有边也没有角），此时将角度保持标志位从1置0，可以再次旋转角度
	            {
		            wny_side_stay=0;	
	            }
	            if(wny_theta>=3.14)			//角度限幅
	            {
		            wny_theta=wny_theta-6.28;
	            }
	            else if(wny_theta<-3.14)
	            {
		            wny_theta=wny_theta+6.28;
	            }
}

 /////////////////////////////////////Kalman///////////////////////////////////
//param:

float Xkf=0;
float X_pre=0,V_pre=0,last_Xkf=0,last_Vkf=0;
float P01=1,P02=0,P03=0,P04=0.1;
float Pre01=0,Pre02=0,Pre03=0,Pre04=0;
float Q01=0.3,Q02=0,Q03=0,Q04=0.3;
float dt=0.02;
float Kg_x=0,Kg_v=0;
float R=1.5;
float V_cal=0;
int first_time=1;
int first_time_h=1;
float vx_init=0,vy_init=0;
void Kalman_x(float y,float a)
{
    //X_pre=A*Xkf(:,k-1); %状态预测
    X_pre=last_Xkf+last_Vkf*dt-0.5*dt*dt*a;
    V_pre=last_Vkf-a*dt;
    //P_pre=A*P0*A'+Q;%协方差预测
    Pre01=P01+P03*dt;
    Pre02=P02+P04*dt;
    Pre03=P03;
    Pre04=P04;

    Pre01=Pre01+Pre02*dt;
    Pre03=Pre03+Pre04*dt;

    Pre01=Pre01+Q01;
    Pre02=Pre02+Q02;
    Pre03=Pre03+Q03;
    Pre04=Pre04+Q04;
    //Kg=P_pre*H'*inv(H*P_pre*H'+R);%计算Kalman增益
    Kg_x=Pre01/(Pre01+R);
    Kg_v=Pre03/(Pre01+R);
    //Xkf(:,k)=X_pre+Kg*(dx(k)-H*X_pre);%状态更新
    Xkf=X_pre+Kg_x*(y-X_pre);
    Vkf=V_pre+Kg_v*(y-X_pre);
    //P0=(I-Kg*H)*P_pre;%方差更新
    P01=Pre01-Pre01*Kg_x;
    P02=Pre02-Pre02*Kg_x;
    P03=Pre03-Pre01*Kg_v;
    P04=Pre04-Pre02*Kg_v;
    V_cal=(Xkf-last_Xkf)/dt;
    last_Xkf=Xkf;
    last_Vkf=Vkf;
}
/////////////////////////////////////////////////////////////////////////////////
float Xkf1=0;
float X_pre1=0,V_pre1=0,last_Xkf1=0,last_Vkf1=0;
float P11=1,P12=0,P13=0,P14=0.1;
float Pre11=0,Pre12=0,Pre13=0,Pre14=0;
float Q11=0.3,Q12=0,Q13=0,Q14=0.3;
float Kg_x1=0,Kg_v1=0;
float R1=1.5;
float V_cal1=0;
void Kalman_y(float y,float a)
{

    //X_pre=A*Xkf(:,k-1); %状态预测
    X_pre1=last_Xkf1+last_Vkf1*dt-0.5*dt*dt*a;
    V_pre1=last_Vkf1-a*dt;
    //P_pre=A*P0*A'+Q;%协方差预测
    Pre11=P11+P13*dt;
    Pre12=P12+P14*dt;
    Pre13=P13;
    Pre14=P14;

    Pre11=Pre11+Pre12*dt;
    Pre13=Pre13+Pre14*dt;

    Pre11=Pre11+Q11;
    Pre12=Pre12+Q12;
    Pre13=Pre13+Q13;
    Pre14=Pre14+Q14;
    //Kg=P_pre*H'*inv(H*P_pre*H'+R);%计算Kalman增益
    Kg_x1=Pre11/(Pre11+R1);
    Kg_v1=Pre13/(Pre11+R1);
    //Xkf(:,k)=X_pre+Kg*(dx(k)-H*X_pre);%状态更新
    Xkf1=X_pre1+Kg_x1*(y-X_pre1);
    Vkf1=V_pre1+Kg_v1*(y-X_pre1);
    //P0=(I-Kg*H)*P_pre;%方差更新
    P11=Pre11-Pre11*Kg_x1;
    P12=Pre12-Pre12*Kg_x1;
    P13=Pre13-Pre11*Kg_v1;
    P14=Pre14-Pre12*Kg_v1;
    V_cal1=(Xkf1-last_Xkf1)/dt;
    last_Xkf1=Xkf1;
    last_Vkf1=Vkf1;
}
//////////////////////////////////////////////////////////////////////////////////////////
float Height_kf=0;
float Height_pre=0,last_Height_kf=0;
float P0_Height=0.1;
float Ph_Pre=0;
float Q_Height=0.0001;
float Kg_Height=0;
float R_Height=0.02;
int Ah=1,Bh=1,Hh=1,Ih=1;
void Kalman_Height(float h_alt,float last_h_alt,float y)
{
	if(first_time_h==0)
	{
		Height_pre=Ah*last_Height_kf+Bh*(h_alt-last_h_alt);
		Ph_Pre=Ah*P0_Height*Ah+Q_Height;
		Kg_Height=Ph_Pre*Hh/(Hh*Ph_Pre*Hh+R_Height);
		Height_kf=Height_pre+Kg_Height*(y-Hh*Height_pre);
		P0_Height=(Ih-Kg_Height*Hh)*Ph_Pre;
		last_Height_kf=Height_kf; 
	}
	else
	{
		last_Height_kf=y;
		P0_Height=0.1;
		R_Height=0.02;
		Q_Height=0.0001;
		first_time_h=0;
	}
}
////////////////////////////////////////////////////////////////////////////////////////// 
float dir_x_kf=0;
float dir_x_pre=0,last_dir_x_kf=0;
float P0_dir_x=0.5;
float Pdir_x_Pre=0;
float Q_dir_x=0.0003;
float Kg_dir_x=0;
float R_dir_x=0.2;
int Adir_x=1,Hdir_x=1,Idir_x=1;

float dir_y_kf=0;
float dir_y_pre=0,last_dir_y_kf=0;
float P0_dir_y=0.5;
float Pdir_y_Pre=0;
float Q_dir_y=0.0003;
float Kg_dir_y=0;
float R_dir_y=0.2;
int Adir_y=1,Hdir_y=1,Idir_y=1;
void Kalman_Direction(float out_dir_x,float out_dir_y)
{
	dir_x_pre=Adir_x*last_dir_x_kf;
	Pdir_x_Pre=Adir_x*P0_dir_x*Adir_x+Q_dir_x;
	Kg_dir_x=Pdir_x_Pre*Hdir_x/(Hdir_x*Pdir_x_Pre*Hdir_x+R_dir_x);
	dir_x_kf=dir_x_pre+Kg_dir_x*(out_dir_x-Hdir_x*dir_x_pre);
	P0_dir_x=(Idir_x-Kg_dir_x*Hdir_x)*Pdir_x_Pre;
	last_dir_x_kf=dir_x_kf; 

	dir_y_pre=Adir_y*last_dir_y_kf;
	Pdir_y_Pre=Adir_y*P0_dir_y*Adir_y+Q_dir_y;
	Kg_dir_y=Pdir_y_Pre*Hdir_y/(Hdir_y*Pdir_y_Pre*Hdir_y+R_dir_y);
	dir_y_kf=dir_y_pre+Kg_dir_y*(out_dir_y-Hdir_y*dir_y_pre);
	P0_dir_y=(Idir_y-Kg_dir_y*Hdir_y)*Pdir_y_Pre;
	last_dir_y_kf=dir_y_kf; 
}
//////////////////////////////////////////////////////////////////////////////////////////
void control()
{
    int timeee=0;
    int landmode=0;
    first_time=1;
    first_time_h=1;
    time_t nowtime;
    nowtime = time(NULL);
    ostringstream oss;
    tm *ltm = localtime(&nowtime);
    oss << ltm->tm_mon+1 << "月"<< ltm->tm_mday << "日"<< ltm->tm_hour << "时"<<ltm->tm_min;
    string path = "/home/hitcsc/catkin_ws/log/iarc/pilot/" + oss.str();//string(ctime(&nowtime));
    string file = path + "/control.txt";
    string temp_file = path + "/height.txt";
    string v_file = path + "/velocity.txt";
    string target_file = path + "/target.txt";
    string target_full = path + "/target_full.txt";
    string houge_file = path + "/houge.txt";
    char* c = new char[path.length()+1];
    strcpy(c,path.c_str());
    if(mkdir(c,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
    {
        cout << "Mkdir success"<< endl;
    }
    else
    {
        cout << "Mkdir error"<< endl;
    }
    delete[] c;
    ofstream control_log(file);
    ofstream height_log(temp_file);
    ofstream v_log(v_file);
    ofstream target_log(target_file);
    ofstream full_log(target_full);
    ofstream hou_log(houge_file);
    ros::NodeHandle pilot;
    DJIDrone* drone = new DJIDrone(pilot);
    LEODrone* wsq_drone=new LEODrone(pilot);
    float time0 = tic();
    pid_x.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidx.txt");
    pid_y.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidy.txt");
    pid_relative_x.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pid_relative_x.txt");
    pid_relative_y.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pid_relative_y.txt");
    pid_z.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidz.txt");
    pid_pitch.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidpitch.txt");
    pid_roll.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidroll.txt");    
    pid_yaw.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidyaw.txt");
    t_p.clean();
    control_mode = -1;

    wsq_drone->obstacle_lidar.num=0;

    /*
    Five modes in total : 
    1: 45   degree interaction
    2: 180  degree interaction
    3: Tracking
    4: Patrol
    5: Ascend 

    IF PID 
    1.1:
    1.2:
    1.3:

    IF LQR
    1:    
    */
    control_mode=6;
    ros::Rate rate(50);
    while(ros::ok())
    {
        // ROS_INFO("Here comes the control part");
        
        //1.If controller is not in F mode, release contorl and exit the control program.
        
        ros::spinOnce();
        while((abs(drone->rc_channels.mode+8000.0)<1)||abs(drone->rc_channels.mode)<1)
        {
             ros::spinOnce();
             drone->release_sdk_permission_control();
             ROS_INFO("Release control.");
             control_log.close();
             height_log.close();
             v_log.close();
             target_log.close();
             full_log.close();
             hou_log.close();
             usleep(5000);

             return;
         }
        
         if(abs(drone->rc_channels.mode-8000.0)<1)
         {
		     drone->request_sdk_permission_control();
	     }
        
        /*This is the main control function*/

        //2. Decide the situation, modify the system mode. 4 modes in total.
        period++;
        if(period==100)
        {
            period=0;
        }
        
        ///// information initialization begins 
        
        float yaw_now=(-cvmGet(att_init,2,0)+cvmGet(att_right_now,2,0));
        if(yaw_now>PI)
		{
			yaw_now=yaw_now-2*PI;
		}
		else if(yaw_now<-PI)
		{
			yaw_now=yaw_now+2*PI;
		}
        // int target_number = wsq_drone->object_pos_dynamic.target_num;
        

        world_ax = wsq_drone->acceleration.ax;
        world_ay = wsq_drone->acceleration.ay;
        world_az = wsq_drone->acceleration.az;
        world_vx = -1*wsq_drone->velocity.vx;
        world_vy = -1*wsq_drone->velocity.vy;
        world_vz = -1*wsq_drone->velocity.vz;

        target_number = wsq_drone->target_dynamic.target.target_num; 
        wsq_drone->target_dynamic.target_x.resize(target_number>0?target_number:2);
        wsq_drone->target_dynamic.target_y.resize(target_number>0?target_number:2);
        wsq_drone->target_dynamic.target_dir_x.resize(target_number>0?target_number:2);
        wsq_drone->target_dynamic.target_dir_y.resize(target_number>0?target_number:2);
        wsq_drone->target_dynamic.target.target_color.resize(target_number>0?target_number:2);

        wsq_drone->ultrasonic.ranges.resize(3);
        wsq_drone->sona.ranges.resize(1);
        wsq_drone->target_dynamic.img_x.resize(2);
        wsq_drone->target_dynamic.img_y.resize(2);
        obstacle_number = wsq_drone->obstacle_dynamic.num;
        wsq_drone->obstacle_dynamic.o_x.resize(obstacle_number);
        wsq_drone->obstacle_dynamic.o_y.resize(obstacle_number);
        
        altitude = (float)(wsq_drone->global_position.altitude);
        float testttt = wsq_drone->sona.ranges[0];
        // Kalman_Height(altitude,last_altitude,testttt1);
        // float testttt = Height_kf;
        // last_altitude=altitude;
        if(target_number>=1)
        {
            dx = -wsq_drone->target_dynamic.target_x[0];
            dy = -wsq_drone->target_dynamic.target_y[0];
            dz = -wsq_drone->target_dynamic.target_z[0];
            img_x = wsq_drone->target_dynamic.img_x[0];
            img_y = wsq_drone->target_dynamic.img_y[0];
            tar_x = wsq_drone->target_dynamic.target_dir_x[0];
            tar_y = wsq_drone->target_dynamic.target_dir_y[0];

            
            
            vx = (dx - last_dx)/0.01/10;
            vy = (dy - last_dy)/0.01/10;

            if(abs(dx-last_dx)>0.15 && last_dx!=0 && last_dx!=-20) //TODO
                dx=last_dx;
            if(abs(dy-last_dy)>0.15 && last_dy!=0 && last_dy!=-20)
                dy=last_dy;
            if(first_time==0)
            {
                Kalman_x(dx,world_ax);
                Kalman_y(dy,world_ay);
                if(tar_x*last_tar_x +tar_y*last_tar_y<0) // angel between last_v and v is smaller than 45 degree
                {
                    tar_x = last_tar_x;
                    tar_y = last_tar_y;
                }
		        Kalman_Direction(tar_x,tar_y);
            }
            else//param
            {
                first_time=0;
                vx_init=0.33*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y);
                vy_init=0.33*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y);
                last_Xkf=dx;
                Xkf=dx;
                last_Vkf=0;//vx_init;
                Vkf=0;//vx_init;
                P01=1;P02=0;P03=0;P04=0.1;
                R=1.5;
                last_Xkf1=dy;
                Xkf1=dy;
                last_Vkf1=0;//vy_init;
                Vkf1=0;//vy_init;
                P11=1;P12=0;P13=0;P14=0.1;
                R1=1.5;

                last_dir_x_kf=tar_x;
                dir_x_kf=tar_x;
                P0_dir_x=0.5;
                R_dir_x=0.2;
                Q_dir_x=0.0003;
                last_dir_y_kf=tar_y;
                dir_y_kf=tar_y;
                P0_dir_y=0.5;
                R_dir_y=0.2;
                Q_dir_y=0.0003;
            }
            last_tar_x = tar_x;
            last_tar_y = tar_y;
            // tar_x=dir_x_kf;
            // tar_y=dir_y_kf;
            if(vx==0 && vy==0 && dx!=0 && dy!=0)
            {
                vx=last_vx;
                vy=last_vy;
            }
            if(abs(vx-last_vx)>=1)
            {
                vx = last_vx;
            }
            if(abs(vy-last_vy)>=1)
            {
                vy = last_vy;
            }
            last_last_dx=last_dx;
            last_last_dy=last_dy;
            last_dx = dx;
            last_dy = dy; 
            last_vx = vx;
            last_vy = vy;
        }
        else//TODO待加：丢目标后的预估
        {
            first_time=1;
            dx=0;
            dy=0;
            dz=0;
            vx=0;
            vy=0;
            last_last_dx=last_dx;
            last_last_dy=last_dy;
            last_dx = vx;
            last_dy = vy; 
        }
        if(obstacle_number>=1)
        {
            obs_dx = -wsq_drone->obstacle_dynamic.o_x[0];
            obs_dy = -wsq_drone->obstacle_dynamic.o_y[0];
        }
        
        compens_vx = (world_vx-Vkf)*0.3/0.5;
        compens_vy = (world_vy-Vkf1)*0.3/0.5;


        target_log << tic()-time0 <<  "\t" << target_number << "\t" 
        << -wsq_drone->target_dynamic.target_x[0] << "\t" << -wsq_drone->target_dynamic.target_y[0] << "\t" << wsq_drone->target_dynamic.target_dir_x[0] << "\t" << wsq_drone->target_dynamic.target_dir_y[0] << "\t" << wsq_drone->target_dynamic.target.target_color[0] << "\t"
        << -wsq_drone->target_dynamic.target_x[1] << "\t" << -wsq_drone->target_dynamic.target_y[1] << "\t" << wsq_drone->target_dynamic.target_dir_x[1] << "\t" << wsq_drone->target_dynamic.target_dir_y[1] << "\t" << wsq_drone->target_dynamic.target.target_color[1] << "\t"
        << -wsq_drone->target_dynamic.target_x[2] << "\t" << -wsq_drone->target_dynamic.target_y[2] << "\t" << wsq_drone->target_dynamic.target_dir_x[2] << "\t" << wsq_drone->target_dynamic.target_dir_y[2] << "\t" << wsq_drone->target_dynamic.target.target_color[2] << "\t"
        << -wsq_drone->target_dynamic.target_x[3] << "\t" << -wsq_drone->target_dynamic.target_y[3] << "\t" << wsq_drone->target_dynamic.target_dir_x[3] << "\t" << wsq_drone->target_dynamic.target_dir_y[3] << "\t" << wsq_drone->target_dynamic.target.target_color[3] << "\t"
        << -wsq_drone->target_dynamic.target_x[4] << "\t" << -wsq_drone->target_dynamic.target_y[4] << "\t" << wsq_drone->target_dynamic.target_dir_x[4] << "\t" << wsq_drone->target_dynamic.target_dir_y[4] << "\t" << wsq_drone->target_dynamic.target.target_color[4] << endl;
        full_log << wsq_drone->target_dynamic.target_num << "\t";
        for(int i=0;i<wsq_drone->target_dynamic.target_num;i++)
        {
            full_log<< -wsq_drone->target_dynamic.target_x[i] << "\t" << -wsq_drone->target_dynamic.target_y[i] << "\t" << wsq_drone->target_dynamic.target_dir_x[i] << "\t" << wsq_drone->target_dynamic.target_dir_y[i] << "\t" << wsq_drone->target_dynamic.target.target_color[i] << "\t";
        }
        full_log<<endl;
        v_log <<  tic()-time0  << "\t"<< target_number<< "\t" << choosed_target<<"\t" << dx << "\t" << dy << "\t" << Xkf << "\t" << Vkf << "\t" << Xkf1 << "\t" << Vkf1 << "\t" << tar_x << "\t"<< tar_y<< "\t"<< wsq_drone->opti_pos.pendulum_posx<< "\t"<< wsq_drone->opti_pos.pendulum_posy<< "\t"<< wsq_drone->opti_pos.pendulum_posz<< "\t"<<dz <<"\t" <<  world_ax << "\t" << world_ay << "\t" << world_az << "\t" << world_vx << "\t"<< world_vy<< "\t"<< world_vz << "\t"
        << compens_vx<< "\t"<< compens_vy << "\t" << dir_x_kf<< "\t"<< dir_y_kf << endl;
        height_log << tic()-time0  <<"\t" <<  world_ax << "\t" << world_ay << "\t" << world_az << "\t" << world_vx << "\t"<< world_vy<< "\t"<< world_vz << "\t" <<wsq_drone->local_position.x << "\t" <<wsq_drone->local_position.y << "\t" <<wsq_drone->local_position.z<<"\t";
        height_log << wsq_drone->global_position.altitude <<"\t"<< wsq_drone->global_position.height<< "\t"<< wsq_drone->ultrasonic.ranges[1] << "\t"<< wsq_drone->opti_pos.posx<< "\t"<< wsq_drone->opti_pos.posy<< "\t"<< wsq_drone->opti_pos.posz<< "\t" <<height_quad<<"\t"<<testttt << "\t" << velocity_arena.vx << "\t" << velocity_arena.vy << "\t" << Height_kf <<"\t"<< wsq_drone->attitude_quaternion.wx<<"\t"<< wsq_drone->attitude_quaternion.wy<<"\t"<< wsq_drone->attitude_quaternion.wz <<endl;
        /// Leo strategy recommended
     
        /*
        if(drone->rc_channels.gear==-4545.0)
        {
            target_number = 1;

        }
        else if(drone->rc_channels.gear==-10000.0)
        {
            target_number = 0;
        }*/

        //// information initialization ends 

        //3.Realization of each mode. Each mode will be divided into several occations accoding to the situation. 
        //  In each occation, the danger of obstacles should be considered.
        //  In these occations, a velocity/attitude contorl command should be generated, and the value of 'attitude_control' should be changed.



        /*main strategy(first try: line strategy)*/
        // if(target_number==1 && control_mode==4)//from patrol to 45 degree
        // {
        //      control_mode=1;
        // }
        // if(target_number==1 && wsq_drone->ultrasonic.ranges[1] >=1.25 && control_mode==5)//from patrol to 45 degree
        // {
        //      //control_mode=2;
        // }
        // if(target_number==1 && obstacle_number>=1 )
        // {
        //     if(sqrt((obs_dx-dx)*(obs_dx-dx)+(obs_dy-dy)*(obs_dy-dy))<=1.3)
        //     {
        //             if(sqrt((obs_dx-dx)*(obs_dx-dx)+(obs_dy-dy)*(obs_dy-dy))<=1.2 && control_mode==1 && fisrtshift)
        //             {
        //                 control_mode=6;
        //             }
        //             else
        //             {
        //                 control_mode=1;
        //                 fisrtshift = 0;
        //             }
        //     } 
        //     else
        //     {
        //         control_mode=1;
        //     }
        // }
        /*main strategy*/


        // cout <<testttt<<" "<<dx<<" "<<dy << " "<< tar_x <<" "<< tar_y <<" "<< dx/sqrt(dx*dx+dy*dy)*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y)+dy/sqrt(dx*dx+dy*dy)*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y)<< endl;

        if(target_number>=1 && stage_45==0 && testttt != 0)
        {
            if(dx/sqrt(dx*dx+dy*dy)*tar_x/sqrt(tar_x*tar_x+tar_y*tar_y)+dy/sqrt(dx*dx+dy*dy)*tar_y/sqrt(tar_x*tar_x+tar_y*tar_y)<.7)
            {
                control_mode= 1;
            }
        }
        if(stage_45==4)
            control_mode=5;

//******************************************houy
        if(stage_45>=1)
        {
            select_tar.x = dx;
            select_tar.y = dy;
            select_tar.dirx = tar_x;
            select_tar.diry = tar_y;
            select_tar.color = 1; // 1 is red, 2 is green
            
            Vector<_select_tar> input_track;
            _select_tar tmp_select_tar;
            for(int i=0;i<wsq_drone->target_dynamic.target_num;i++)
            {
                if(wsq_drone->target_dynamic.target.target_id[i]>0)
                {
                    tmp_select_tar.x=wsq_drone->target_dynamic.img_x[i];
                    tmp_select_tar.y=wsq_drone->target_dynamic.img_y[i];
                    tmp_select_tar.dirx=wsq_drone->target_dynamic.target_dir_x[i];
                    tmp_select_tar.diry=wsq_drone->target_dynamic.target_dir_y[i];
                    tmp_select_tar.color=wsq_drone->target_dynamic.target.target_color[i];
                    input_track.push_back(tmp_select_tar);
                }
            }
            //
            int tar_choose=hou_track(select_tar,testttt,input_track);
            if(tar_choose>=0)
            {
                dx = input_track[tar_choose].x;
                dy = input_track[tar_choose].y;
                tar_x = input_track[tar_choose].dirx;
                tar_y = input_track[tar_choose].diry;
            }
        }

//******************************************



        switch (control_mode)
        {
            case 1:     //45 degree
            {
                if(pid_flag)
                {
                    //cout << stage_45 <<"\t";
                    if( ( stage_45==0 || stage_45==1 ))
                    {
                        float height_45=testttt;
                        stage_45=1;
                        // float cmd_height = sqrt(dx*dx+dy*dy)+0.1;
                        float cmd_height =0.78;
                        inter_45(dx,dy,Vkf,Vkf1,height_45,cmd_height,0,tar_x,tar_y);
                        global_status   = 2;
                    }
                    else if(stage_45==2)
                    {
                        float height_45=ul_ref+altitude-bar_ref;
                        float cmd_height = 0.28;
                        inter_45(dx,dy,Vkf,Vkf1,height_45,cmd_height,0,tar_x,tar_y);
                    }
                    else if(stage_45==3)
                    {
                        float height_45=ul_ref+altitude-bar_ref;
                        float cmd_height = 0.15;
                        // inter_45(dx,dy,Vkf,Vkf1+0.01,height_45,cmd_height,0,tar_x_45,tar_y_45);
                        descend(tar_x_45,tar_y_45,world_az,0.28);
                    }

                    else
                    {
                        control_mode = 5;
                        stop();
                    }
                    
                }
                else
                {
                    cout << "test failed 1"<< endl;
                    stop();
                }
                break;
            }
            case 2:     // 180 degree 
            {
                if(pid_flag)
                {
                    if( stage_180==1 || stage_180 ==0)
                    {
                        float height_180=testttt;
                        interact_180_noobs(dx,dy,Vkf,Vkf1+0.01,height_180,0.85,0,tar_x, tar_y);
                        global_status   = 2;
                    }
                    else if(stage_180==2)
                    {
                        float height_180=testttt;
                        interact_180_noobs(height_180,0.06,tar_x_180,tar_y_180,0);
                    }
                    else if(stage_180 == 3)
                    {
                        float height_180=testttt;
                        interact_180_noobs(wsq_drone->sona.ranges[0],0.06,tar_x_180,tar_y_180,0);
                    }
                    else
                    {
                        control_mode = 5;
                        stop();
                    }
                    
                }
                else
                {
                    // cout << target_number << endl;
                    stop();
                }
                break;
            }
            case 3:     // track 
            {              
                float height_test=1;
                attitude_control = 0; 
               //cout << "test1"<< endl;
                if(target_number==1)// && landmode==0)
                {  
                    //cout<<tic()-time0 <<"\t"<< target_number<<"\t"<<info.x<<"\t"<<info.y<<"\t"<<info.vx<<"\t"<<info.vy<<"\t" <<endl;           
                    if(attitude_control)//(!trust_guidance)
                    {
                        cout << "attitdde"<< control_mode<< endl;
                        track(dx,dy,Vkf,Vkf1+0.01,testttt,height_test,1,tar_x,tar_y);  // attitude track
                        // track(dx,dy,world_vx,world_vy,testttt,height_test,1,0.);  // attitude track
                    }//Vkf+0.1,Vkf1+0.1
                    else
                    {
                        //cout << "velocity"<< endl;
                        track(dx,dy,0.,0.,testttt,height_test,0,tar_x,tar_y);              // Velocity track
                    }
                }
                else
                {
                    if(attitude_control)//(!trust_guidance)
                    {
                        //cout << "attitdde"<< endl;
                        track(0,0,world_vx,world_vy,testttt,1.0,1,tar_x,tar_y);  // attitude track
                    }//Vkf+0.1,Vkf1+0.1
                    else
                    {
                        //cout << "velocity"<< endl;
                        track(0,0,0.,0.,testttt,height_test,0,tar_x,tar_y);              // Velocity track
                    }/*
                    lost_target_counter++;
                    if(lost_target_counter<=10)
                    {
                        ;
                    }
                    else
                    {
                        stop();
                        lost_target_counter = 0;
                        control_mode = 5;
                        height_test=1.5;
                    }*/
                    stop();
                }
                /*
                if(abs(dx)<0.05 && abs(dy)<0.05 && timeee==0 && abs(dx)>0 && abs(dy)>0)
                {
                    timeee=1;
                    landmode=1;
                    tar_dx=dx+0.5*tar_x/(tar_x*tar_x+tar_y*tar_y);
                    tar_dy=dy+0.5*tar_y/(tar_x*tar_x+tar_y*tar_y);
                    time0000=tic()-time0;
                }
                
                if(landmode==1)
                {
                    height_test=0.2;
                    track(-tar_dx,-tar_dy,0.,0.,wsq_drone->ultrasonic.ranges[2],height_test,0,0.);
                    if(tic()-time0-time0000>2)
                        landmode=2;
                    
                }
                if(landmode==2)
                {
                    height_test=0.2;
                    track(0,0,0.,0.,wsq_drone->ultrasonic.ranges[2],height_test,0,0.);
                }
                */
                
                break;
            }

            // case 4:     // patrol (for line strategy)
            // {
            //     //2017.07.31 V2.1

            //     /*chapter1：数据读取*/
	        //     //高度测量
	        //     wny_quad_height=   ;		
        	//     //边界检测
        	//     wny_side_check=    ;		
	        //     wny_side_corner=    ;		
	        //     //前一帧速度
	        //     wny_quad_vx_last=wny_quad_vx;	
	        //     wny_quad_vy_last=wny_quad_vy;

            //     /*chapter2：速度计算*/
	        //     /*part1：巡航速度*/
	        //     wny_tar_vx=wny_tar_velocity*cos(wny_theta);
	        //     wny_tar_vy=wny_tar_velocity*sin(wny_theta);
            //     /*part2：边界约束*/
	        //     //第1层逻辑：判断位于边或角，选取反向移动或者旋转一定角度
	        //     //第2层逻辑：判断位于哪一条边，结合飞行器速度方向（wny_quad_theta）选取旋转角度的取值
	        //     //角度保持逻辑：角度保持标志位为0，并遇到边或角，进行旋转同时将标志位置1，保持旋转后角度，当不在边或角附近，且角度保持标志位为1，将角度保持标志位置0，可以在遇到边或角的时候再次进行旋转
	        //     //角度限幅逻辑：限制角度始终在[-π,π)区间内
	        //     //考虑要不要在初始的一段时间内不进行边界约束处理，防止巡航速度方向角大幅度变化
	        //     if(wny_side_corner==0&&wny_side_stay==0)		//在边附近，角度保持标志位为0（清零），旋转角度，将角度保持标志位置1（保持），保持住旋转后的角度
	        //     {
		    //         if(wny_side_check==0)		//绿边附近
		    //         {
    		// 	        if(wny_quad_theta>=0&&wny_quad_theta<3.14)		//飞行器速度方向指向场内，不旋转
		    // 	        {
		    // 	        }
			//             else if(wny_quad_theta>=-1.57&&wny_quad_theta<0)	//飞行器速度方向指向场外，x正向，角度加120度
			//             {
			//     	        wny_theta=wny_theta+2.09;
			//             }
			//             else if(wny_quad_theta>=-3.14&&wny_quad_theta<-1.57)	//飞行器速度方向指向场外，x负向，角度减120度
			//             {
		    // 		        wny_theta=wny_theta-2.09;
		    // 	        }
		    //         }
		    //         else if(wny_side_check==1)	//右白边附近
		    //         {
			//             if((wny_quad_theta>=-3.14&&wny_quad_theta<-1.57)||(wny_quad_theta<3.14&&wny_quad_theta>=1.57))		//飞行器速度方向指向场内，不旋转
			//             {
			//             }
			//             else if(wny_quad_theta>=0&&wny_quad_theta<1.57)		//飞行器速度方向指向场外，y正向，角度加120度
			//             {
			// 	            wny_theta=wny_theta+2.09;
			//             }
			//             else if(wny_quad_theta>=-1.57&&wny_quad_theta<0)	//飞行器速度方向指向场外，y负向，角度减120度
			//             {
			// 	            wny_theta=wny_theta-2.09;
			//             }
		    //         }
		    //         else if(wny_side_check==2)	//红边附近
		    //         {
			//             if(wny_quad_theta>=-3.14&&wny_quad_theta<0)		//飞行器速度方向指向场内，不旋转
			//             {
			//             }
			//             else if(wny_quad_theta>=0&&wny_quad_theta<1.57)		//飞行器速度方向指向场外，x正向，角度减120度
			//             {
			// 	            wny_theta=wny_theta-2.09;
			//             }
			//             else if(wny_quad_theta>=1.57&&wny_quad_theta<3.14)	//飞行器速度方向指向场外，x负向，角度加120度
		    // 	        {
			// 	            wny_theta=wny_theta+2.09;
			//             }
		    //         }
		    //         else if(wny_side_check==3)	//左白边附近
		    //         {
			//             if(wny_quad_theta>=-1.57&&wny_quad_theta<1.57)		//飞行器速度方向指向场内，不旋转
			//             {
			//             }
			//             else if(wny_quad_theta>=-3.14&&wny_quad_theta<-1.57)	//飞行器速度方向指向场外，y负向，角度加120度
			//             {
			// 	            wny_theta=wny_theta+2.09;
			//             }
			//             else if(wny_quad_theta>=1.57&&wny_quad_theta<3.14)	//飞行器速度方向指向场外，y正向，角度减120度
			//             {
			// 	            wny_theta=wny_theta-2.09;
			//             }
		    //         }
		    //         wny_side_stay=1;
	        //     }
	        //     else if(wny_side_corner==1&&wny_side_stay==0)		//在角附近，角度保持标志位为0（清零），旋转角度，将角度保持标志位置1（保持），保持住旋转后的角度
	        //     {
		    //         wny_theta=wny_theta+3.14;	//反向移动，即旋转180度
		    //         wny_side_stay=1;
	        //     }
	        //     if(wny_side_corner==2&&wny_side_stay==1)		//当脱离边或角附近时（附近没有边也没有角），此时将角度保持标志位从1置0，可以再次旋转角度
	        //     {
		    //         wny_side_stay=0;	
	        //     }
	        //     if(wny_theta>=3.14)			//角度限幅
	        //     {
		    //         wny_theta=wny_theta-6.28;
	        //     }
	        //     else if(wny_theta<-3.14)
	        //     {
		    //         wny_theta=wny_theta+6.28;
	        //     }

                

            //     /*part3：水平速度*/
	        //     //目前两类速度，第一类为巡航速度，第二类为避障速度
	        //     wny_quad_vx=(double)wny_tar_vx+wny_obstacle_vx;	//计算水平速度
	        //     wny_quad_vy=(double)wny_tar_vy+wny_obstacle_vy;
	        //     wny_quad_theta=atan2(wny_quad_vy,wny_quad_vx);	//计算水平速度航向,方向角区间为[-π,π)
	        //     //待解决问题：怎么同时考虑边界和障碍物约束的这一类特殊情况（应该不是简单将速度加和，考虑如何分情况讨论）
            //     /*part4：竖直速度及偏航角速度*/
	        //     if(wny_quad_posz>wny_quad_poszmax)	//具体数值可调节
	        //     {
		    //         wny_quad_vz=-0.05;
	        //     }
	        //     else if(wny_quad_posz<wny_quad_poszmin)
	        //     {
		    //         wny_quad_vz=0.05;
	        //     }
	        //     else
	        //     {
		    //         wny_quad_vz=0.0;
	        //     }
	        //     float wny_quad_vyaw=0.0;		//暂定无偏航运动
            //     /*chapter3：输出限幅*/
	        //     //先限制加速度，防止抖动；再限制速度，降低危险
	        //     /*part1：加速度限幅*/
	        //     wny_quad_ax=(wny_quad_vx-wny_quad_vx_last)/0.02;	//由速度指令计算加速度
	        //     wny_quad_ay=(wny_quad_vy-wny_quad_vy_last)/0.02;
	        //     if(wny_quad_ax>wny_quad_ax_limitation)
            // 	{
		    //         wny_quad_vx=wny_quad_vx_last+wny_quad_ax_limitation*0.02;
	        //     }
	        //     else if(wny_quad_ax<-wny_quad_ax_limitation)
            //     {
            //         wny_quad_vx=wny_quad_vx_last-wny_quad_ax_limitation*0.02;
            //     }

            //     if(wny_quad_ay>wny_quad_ay_limitation)
            //     {
            //         wny_quad_vy=wny_quad_vy_last+wny_quad_ay_limitation*0.02;
            //     }
            //     else if(wny_quad_ay<-wny_quad_ay_limitation)
            //     {
            //         wny_quad_vy=wny_quad_vy_last-wny_quad_ay_limitation*0.02;
            //     }
                



            //     /*part2：速度限幅*/
            //     if(wny_quad_vx>wny_quad_vx_limitation)
            //     {
            //         wny_quad_vx=wny_quad_vx_limitation;
            //     }
            //     else if(wny_quad_vx<-wny_quad_vx_limitation)
            //     {
            //         wny_quad_vx=-wny_quad_vx_limitation;
            //     }

            //     if(wny_quad_vy>wny_quad_vy_limitation)
            //     {
            //         wny_quad_vy=wny_quad_vy_limitation;
            //     }
            //     else if(wny_quad_vy<-wny_quad_vy_limitation)
            //     {
            //         wny_quad_vy=-wny_quad_vy_limitation;
            //     }

            //     if(wny_quad_vz>wny_quad_vz_limitation)
            //     {
            //         wny_quad_vz=wny_quad_vz_limitation;
            //     }
            //     else if(wny_quad_vz<-wny_quad_vz_limitation)
            //     {
            //         wny_quad_vz=-wny_quad_vz_limitation;
            //     }

            //     if(wny_quad_vyaw>wny_quad_vyaw_limitation)
            //     {
            //         wny_quad_vyaw=wny_quad_vyaw_limitation;
            //     }
            //     else if(wny_quad_vyaw<-wny_quad_vyaw_limitation)
            //     {
            //         wny_quad_vyaw=-wny_quad_vyaw_limitation;
            //     }



                
            //     break;
            // }

            case 5:     // ascend
            {
                ascend_counter++;
                ascend(testttt,ascend_mode,ascend_mode==3?tar_x_180:tar_x,ascend_mode==3?tar_y_180:tar_y);
                break;
            }
            case 6:
            {
                stop();
                break;
            }
            case 7:     // fly a quad for location program
            {
                attitude_control=false;
                // cout << quad_flag << endl;
                // 1,1 1,6 4,1 4,6
                if(quad_flag==0)
                {
                    if(fabs(2-wsq_drone->position_lf.x)>=0.5 || fabs(2-wsq_drone->position_lf.y)>=0.5)
                    {
                        velocity_arena.vx = pid_x.out(2,wsq_drone->position_lf.x,1);
                        velocity_arena.vy = pid_y.out(2,wsq_drone->position_lf.y,1);
                        velocity_arena.vh     = pid_z.out(1,testttt,0);
                        velocity_arena.dyaw   = 0.;
                    }
                    else
                    {
                        quad_flag=1;
                        pid_x.clean();
                        pid_y.clean();
                    }
                }
                else if(quad_flag==1)
                {
                    if(fabs(2-wsq_drone->position_lf.x)>=0.5 || fabs(5-wsq_drone->position_lf.y)>=0.5)
                    {
                        velocity_arena.vx = pid_x.out(2,wsq_drone->position_lf.x,1);
                        velocity_arena.vy = pid_y.out(5,wsq_drone->position_lf.y,1);
                        velocity_arena.vh     = pid_z.out(1,testttt,0);
                        velocity_arena.dyaw   = 0.;
                    }
                    else
                    {
                        quad_flag=2;
                        pid_x.clean();
                        pid_y.clean();
                    }
                }
                else if(quad_flag==2)
                {
                    if(fabs(3-wsq_drone->position_lf.x)>=0.5 || fabs(5-wsq_drone->position_lf.y)>=0.5)
                    {
                        velocity_arena.vx = pid_x.out(3,wsq_drone->position_lf.x,1);
                        velocity_arena.vy = pid_y.out(5,wsq_drone->position_lf.y,1);
                        velocity_arena.vh     = pid_z.out(1,testttt,0);
                        velocity_arena.dyaw   = 0.;
                    }
                    else
                    {
                        quad_flag=3;
                        pid_x.clean();
                        pid_y.clean();
                    }
                }
                else if(quad_flag==3)
                {
                    if(fabs(3-wsq_drone->position_lf.x)>=0.5 || fabs(2-wsq_drone->position_lf.y)>=0.5)
                    {
                        velocity_arena.vx = pid_x.out(3,wsq_drone->position_lf.x,1);
                        velocity_arena.vy = pid_y.out(2,wsq_drone->position_lf.y,1);
                        velocity_arena.vh     = pid_z.out(1,testttt,0);
                        velocity_arena.dyaw   = 0.;
                    }
                    else
                    {
                        quad_flag=0;
                        pid_x.clean();
                        pid_y.clean();
                    }
                }
                break;
            }
        }
            //4. Send the control command to M100.
            if(attitude_control)
            {
                
                attitude_ground.pitch  = 1*(attitude_arena.pitch*cvmGet(R_init,0,0)+attitude_arena.roll*cvmGet(R_init,0,1));
                attitude_ground.roll   = 1*(attitude_arena.pitch*cvmGet(R_init,1,0)+attitude_arena.roll*cvmGet(R_init,1,1));

                attitude_command.pitch  = 0.3+attitude_ground.pitch*cvmGet(R_now_i,0,0)+attitude_ground.roll*cvmGet(R_now_i,0,1);   // -0.7 is the pitch compensate 
                attitude_command.roll   = -0.1+attitude_ground.pitch*cvmGet(R_now_i,1,0)+attitude_ground.roll*cvmGet(R_now_i,1,1);  // -0.7 is the roll compensate 
                attitude_command.dyaw   = attitude_arena.dyaw;
                attitude_command.vh     = attitude_arena.vh;


                if(abs(attitude_command.vh)>=0.5)
                {
                    exit;
                }
                //cout << attitude_arena.pitch << "\t" << attitude_arena.roll << "\t" << attitude_ground.pitch << "\t" << attitude_ground.roll << "\t" << attitude_command.pitch << "\t" << attitude_command.roll << endl;
                //cout << attitude_command.pitch << "\t" << attitude_command.roll << endl;
                /*case 2: 去掉目标出现第一帧的相对速度读取*/

                drone->attitude_control(    Flight::HorizontalLogic::HORIZONTAL_ANGLE|
                                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                                            Flight::YawLogic::YAW_PALSTANCE |
                                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |	
                                            Flight::SmoothMode::SMOOTH_DISABLE,
                                            attitude_command.roll, -1*attitude_command.pitch, attitude_command.vh, attitude_command.dyaw);
            //cout << attitude_arena.vh << endl;
            }
            else
            {
                
                velocity_command.vx       = (velocity_arena.vx*cvmGet(R_init,0,0)+velocity_arena.vy*cvmGet(R_init,0,1)+velocity_arena.vh*cvmGet(R_init,0,2));
                velocity_command.vy       = (velocity_arena.vx*cvmGet(R_init,1,0)+velocity_arena.vy*cvmGet(R_init,1,1)+velocity_arena.vh*cvmGet(R_init,1,2));
                velocity_command.vh       = velocity_arena.vh;//velocity_arena.vx*cvmGet(R_init,2,0)+velocity_arena.vy*cvmGet(R_init,2,1)+velocity_arena.vh*cvmGet(R_init,2,2);
                velocity_command.dyaw     = velocity_arena.dyaw;
                if(fabs(velocity_command.vx)>=0.7 && fabs(velocity_command.vy)>=0.7 && fabs(velocity_command.vh)>=0.8)
                {
                    cout << "velocity command is to big"<< endl;
                    stop();
                    return;
                }
                velocity_command.vx = abs(velocity_command.vx)>0.5?sgn(velocity_command.vx)*0.5:velocity_command.vx;
                velocity_command.vy = abs(velocity_command.vy)>0.5?sgn(velocity_command.vy)*0.5:velocity_command.vy;
                drone->attitude_control(    Flight::HorizontalLogic::HORIZONTAL_VELOCITY|
                                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                                            Flight::YawLogic::YAW_PALSTANCE |
                                            Flight::HorizontalCoordinate::HORIZONTAL_GROUND |	
                                            Flight::SmoothMode::SMOOTH_DISABLE,
                                            velocity_command.vx, velocity_command.vy, velocity_command.vh, velocity_command.dyaw);
            }
        //5. 
        if(attitude_control)
        {
            control_log<<setiosflags(ios::fixed) <<control_mode<<"\t"<<stage_180<<"\t"<<stage_45<<"\t"<<ascend_mode<<"\t" << attitude_control << "\t" << attitude_command.roll<<"\t"<< attitude_command.pitch<<"\t"<<attitude_command.vh<<"\t"<<attitude_command.dyaw<<"\t"<<-cvmGet(att_init,0,0)+cvmGet(att_right_now,0,0)<<"\t"<<-cvmGet(att_init,1,0)+cvmGet(att_right_now,1,0)<<"\t"<<-cvmGet(att_init,2,0)+cvmGet(att_right_now,2,0)<<endl;
        }
        else
        {
            control_log<<setiosflags(ios::fixed) <<control_mode<<"\t"<<stage_180<<"\t"<<stage_45<<"\t"<<ascend_mode<<"\t" << attitude_control << "\t" << velocity_command.vx    <<"\t"<<velocity_command.vy <<"\t"<<velocity_command.vh <<"\t"<<velocity_command.dyaw<<"\t"<<-cvmGet(att_init,0,0)+cvmGet(att_right_now,0,0)<<"\t"<<-cvmGet(att_init,1,0)+cvmGet(att_right_now,1,0)<<"\t"<<-cvmGet(att_init,2,0)+cvmGet(att_right_now,2,0)<<endl;
        }
        // Modifing the history data
        if(control_mode!=control_mode_previous)
        {
            pid_x.clean();
            pid_y.clean();
            pid_z.clean();
            pid_yaw.clean();
            pid_pitch.clean();
            pid_roll.clean();
            pid_relative_x.clean();
            pid_relative_y.clean();
            cout<< "Clean the integral and history data of pid when the mode is changed"<<endl;
        }
        control_mode_previous = control_mode;
        // cout << "Control mode " << control_mode << " 180 mode "<< stage_180 << " 45 mode "<< stage_45 << " ascend mode "<< ascend_mode<<" tn "<< target_number<< " tt "<< tar_x_180<<" "<<tar_y_180<<endl;
        // Modification done

        rate.sleep();
    }
}