#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dynamic_reconfigure/server.h>
#include <iarc007/iarc007Config.h>
#include <sys/time.h>
#include <iarc007/jrh_pilot/control.h>
#include <iarc007/jrh_pilot/subscribe.h>
#include <iarc007/object.h>
#include <math.h>

#include <iostream>
#include <geometry_msgs/Twist.h>//position
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <opencv/highgui.h>
#include <std_msgs/Float32.h>
#include <memory.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include <opencv/cv.h>
#include <sensor_msgs/Range.h>

#include <iarc007/vehicle_pos.h>
#include <iarc007/obstacle.h>
#define circle_r 0.3
int jrh_iRobot_i;
FILE *model_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/model.txt","w");
FILE *wny_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/wny.txt","w");


//////////////////////////////////////////////////////////////////////////////////////////////比赛快速调参
float distance_ahead_45, distance_ahead_180;//机器人向前叠加距离
float height_limited;//45度/180度/跟踪模式高度限幅
float forbid_distance_land;//距离障碍物这个距离时禁止降落
float land_condition_r_45, land_condition_erryaw_45, land_condition_height_45;//45度降落条件
float land_condition_r_180, land_condition_erryaw_180, land_condition_height_180;//180度降落条件
float parallel_land_time_45_first, parallel_land_time_45_second;//45度降落两段持续时间
float parallel_land_time_180_first, parallel_land_time_180_second;//180度降落两段持续时间
float obstacle_danger_distance;//避障反向跑的距离
float velcity_land_45;//45度降落速度，维持大小
float comd_height_if_obstacle = 1.25;
float comd_follow_height = 1.5;
float velcity_zhuanquan,yaw_rate_zhuanquan;
float velcity_land_180;//180度降落速度大小
float parallel_land_velcity_180;//180度平行速度大小
//////////////////////////////////////////////////////////////////////////////////////////////

double tic(){
	struct timeval t;
	gettimeofday(&t,NULL);
	return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

struct stuff
{
        int operate_num1;
        int operate_num2;
        float time;
        float out_sita_target;

};
stuff standard2[10];
int last_model_choose;
extern float wny_px,wny_py,wny_pz;

int boundry_flag=0;

float transform_angle(int flag_select,float sita_target)
{
/*%transform_angle 根据model_select返回的flag_select值和原有的目标速度方向计算新的目标速度方向
% flag_select local_model_select返回值
% sita_target 目标速度方向，同时为返回值*/
	float pi = 3.14;
	if(flag_select==0)
	{
		sita_target=sita_target+0;
	}
	else if(flag_select==1)
	{
		sita_target=sita_target+pi/4;
	}
	else if(flag_select==2)
	{
		sita_target=sita_target+pi;
	}

	//%sita_target转换到-pi到+pi
	while(sita_target>pi)
	{
		sita_target=sita_target-2*pi;
	}
	while(sita_target<-pi)
	{
		sita_target=sita_target+2*pi;
	}
	return sita_target;
}


int local_model_select(float sita_target,int flag_location)
{
/*%已知一个目标局部位置坐标，速度方向，判断飞机应对其进行哪种操作，即不操作/转45/转180
% sita_target 目标速度方向
%flag_location 1表示飞机看到左边界（相当于global_model_select中的1区）
%              2表示飞机没有看到左/右边界（相当于global_model_select中的2区）
%              3表示飞机看到右边界（相当于global_model_select中的3区）
%返回值flag_select 0不操作 1转45 2转180

%sita_target转换到-pi到+pi*/
		float pi = 3.14;
		int flag_select;

		//%sita_target转换到-pi到+pi
		while(sita_target>pi)
		{
			sita_target=sita_target-2*pi;
		}
		while(sita_target<-pi)
		{
			sita_target=sita_target+2*pi;
		}

     if(flag_location==1)//%相当于1区
     {
         if(((sita_target>=-pi)&&(sita_target<-pi/2))||(sita_target==pi))
         {
        	 flag_select=1;
         }
         else if((sita_target>=-pi/2)&&(sita_target<0))
         {
        	 flag_select=0;
         }
         else if((sita_target>=0)&&(sita_target<=pi/4))
         {
        	 flag_select=1;
         }
         else if((sita_target>pi/4)&&(sita_target<pi))
         {
        	 flag_select=2;
         }
     }
	 /*
     else if(flag_location==2)//%相当于2区 local_strategy1
     {
         if(((sita_target>=-pi)&&(sita_target<-pi*5/8))||(sita_target==pi))
         {
        	 flag_select=1;
         }
         else if((sita_target>=-pi*5/8)&&(sita_target<0))
         {
        	 flag_select=0;
         }
         else if((sita_target>=0)&&(sita_target<=pi*7/8))
         {
        	 flag_select=2;
         }
         else if((sita_target>pi*7/8)&&(sita_target<pi))
         {
        	 flag_select=1;
         }
     }
	 */
	 /****************************************************************************/
	 
	 else if(flag_location==2)//%相当于2区 local_strategy2
	 {
	 if(((sita_target>=-pi)&&(sita_target<-pi*5/8))||(sita_target==pi))
	 {
	 flag_select=1;
	 }
	 else if((sita_target>=-pi*5/8)&&(sita_target<=-pi/4))
	 {
	 flag_select=0;
	 }
	 else if((sita_target>-pi/4)&&(sita_target<=pi*3/4))
	 {
	 flag_select=2;
	 }
	 else if((sita_target>pi*3/4)&&(sita_target<pi))
	 {
	 flag_select=1;
	 }
	 }
	 
	 /**************************************************************************************/
	 /*
	 else if(flag_location==2)//%相当于2区 local_strategy3
	 {
	 if(((sita_target>=-pi)&&(sita_target<0))||(sita_target==pi))
	 {
	 flag_select=0;
	 }
	 else if((sita_target>=0)&&(sita_target<pi))
	 {
	 flag_select=2;
	 }
	 }
	 */
     else if(flag_location==3)//%相当于3区
     {
         if((sita_target>=-pi)&&(sita_target<=-pi*3/4))
         {
        	 flag_select=1;
         }
         else if((sita_target>-pi*3/4)&&(sita_target<=-pi/2))
         {
        	 flag_select=0;
         }
         else if((sita_target>-pi/2)&&(sita_target<=pi/2))
         {
        	 flag_select=2;
         }
         else if(((sita_target>pi/2)&&(sita_target<=pi))||(sita_target==-pi))
         {
        	 flag_select=1;
         }
      }
     return flag_select;
}




struct stuff local_multitarget_select_step1(float x_target,float y_target,float sita_target,int flag_location,float t1_rotate,float t2_rotate)
{
/*%已知一个目标速度方向，计算将其驱赶出绿边过程中
%初始阶段所需操作次数、时间、操作后的速度方向
% x_target,y_target目标在局部坐标中的位置
% sita_target 目标速度方向
% flag_select 0不操作 1转45 2转180
% t1_operate t2_operate从飞机作出判断到飞机完成操作1、2需要的时间
% t1_rotate t2_rotate 目标进行动作1、2原地旋转需要的时间
% operate_num1 operate_num2 飞机进行操作1、2各自的次数*/
struct stuff standard1;//返回值
standard1.operate_num1=0;
standard1.operate_num2=0;
standard1.time=0;
standard1.out_sita_target=0;

float aaa=1;//%飞机交互时减速的加速度
float vt=0.33;//%目标速度
float t1_operate,t2_operate;//% t1_operate t2_operate从飞机作出判断到飞机完成操作1、2需要的时间,这里简化为飞机飞行至目标所在位置的时间
float pi = 3.14;
int flag_select=1;
//%sita_target转换到-pi到+pi
	while(sita_target>pi)
	{
		sita_target=sita_target-2*pi;
	}
	while(sita_target<-pi)
	{
		sita_target=sita_target+2*pi;
	}

t1_operate=sqrt(2*sqrt(x_target*x_target+y_target*y_target)/aaa);
t2_operate=sqrt(2*sqrt(x_target*x_target+y_target*y_target)/aaa);

    while(flag_select!=0)
    {
        flag_select=local_model_select(sita_target,flag_location);//%判断飞机应进行哪种操作
        if(flag_select==1)
        {
        	 standard1.time=standard1.time+t1_operate+t1_rotate;
        	 standard1.operate_num1=standard1.operate_num1+1;//%记录操作1的次数
        }
        else if(flag_select==2)
        {
            standard1.time=standard1.time+t2_operate+t2_rotate;
            standard1.operate_num2=standard1.operate_num2+1;//%记录操作2的次数
        }
        sita_target=transform_angle(flag_select,sita_target);//%操作后目标速度方向
        standard1.out_sita_target=sita_target;
    }
    return standard1;
}

int local_multitarget_select_step2(int n,float x_target[],float y_target[],float sita_target[],int flag_location,float t1_rotate,float t2_rotate)
{
/*% 已知n个目标的局部位置坐标，速度方向，判断最易驱赶的目标
% n 目标个数
% x_target,y_target目标局部位置坐标
% sita_target 目标速度方向*/
int i,j;
float pi=3.14;

for (i=0;i<n;i++)
{
	standard2[i]= local_multitarget_select_step1(x_target[i],y_target[i],sita_target[i],flag_location,t1_rotate,t2_rotate);
}

float min_time=standard2[0].time;
int min_operate_num1=standard2[0].operate_num1;
int min_operate_num2=standard2[0].operate_num2;
float min_out_sita_target=fabs(-pi/2-standard2[0].out_sita_target);

/****************************************************************************************************************/
int flag_multitarget_select=0;//多个目标中选择的目标编号

for (j=1;j<n;j++)
{
    if(fabs(-pi/2-standard2[j].out_sita_target)<min_out_sita_target)
    {
        min_out_sita_target=fabs(-pi/2-standard2[j].out_sita_target);
        flag_multitarget_select=j;
    }
    else if(fabs(-pi/2-standard2[j].out_sita_target)==min_out_sita_target)
    {
    	 if(standard2[j].time<min_time)
    	 {
    	        min_time=standard2[j].time;
    	        flag_multitarget_select=j;
    	 }
    	  else if(standard2[j].time==min_time)
    	  {
    	            if(standard2[j].operate_num2<min_operate_num2)
    	            {
    	                min_operate_num2=standard2[j].operate_num2;
    	                flag_multitarget_select=j;
    	            }
    	            else if(standard2[j].operate_num2==min_operate_num2)
    	            {
    	                if(standard2[j].operate_num1<=min_operate_num1)
    	                {
    	                    flag_multitarget_select=j;
    	                }
    	            }
    	  }
    }
}
return flag_multitarget_select;
}

int model_switch(int n,float x_target[],float y_target[],float sita_target[],int flag_location,float t1_rotate,float t2_rotate)
{
	/*已知当前视野内地面机器人数目，判断飞机赢采取何种模式
	 0跟随不操作 1转45度 2转180度 3巡航*/
	int model,flag_multitarget_select;
	float sita;
	if(n==0)
	{
		model=3;
	}
	else if(n==1)
	{
		flag_multitarget_select=0;
		jrh_iRobot_i=flag_multitarget_select;
		printf("flag_multitarget_select=%d",flag_multitarget_select);
		sita=sita_target[0];
		model=local_model_select(sita,flag_location);

	}
	else
	{
		flag_multitarget_select=local_multitarget_select_step2(n,x_target,y_target,sita_target,flag_location,t1_rotate,t2_rotate);
		jrh_iRobot_i=flag_multitarget_select;
		printf("flag_multitarget_select=%d",flag_multitarget_select);
		sita=sita_target[flag_multitarget_select];
		model=local_model_select(sita,flag_location);
	}
	/*
	if((model==1 || model==2) && (last_model_choose==3) && (wx_px>3.5 || wx_px<0.5 || wx_py>6.5 || wx_py<0.5))
	{
	model=3;
	boundry_flag=2;
	}
	*/
	//model=88;
	/******************************************************************/
	/*if(model==1)//交互去掉45度只保留180度
	{
	model=2;
	}*/
	/*if(model==2)//交互去掉180度只保留45度
	{
	model=1;
	}*/
	return model;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////
float iRobot_x=0.0f,iRobot_y=0.0f,iRobot_theta=0.0f,r_vt=0.0f,r0=0.0f,r_vt_original=0.0f;//TODO
float wx_iRobot_x[10];
float wx_iRobot_y[10];
float wx_iRobot_theta[10];
long int wx_iRobot_serial_num[10];
unsigned int wx_iRobot_num=0;
unsigned int iRobot_num=0;
float F_parallel=0,F_point=0,theta_point=0;
bool first_interaction_45=true;//ToDo %注意以后对于bool变量需要在某些地方变回来
float pos_now[3]={0,0,0};
int jishu=0,cishu=0;
float height_H=0.0f;//计算高度最高的指令
float height_L=0.0f;//计算高度最低的指令
float remember_theta;
double remember_time;
///////////////////////////////////////////PID 按照x，y，yaw,height的顺序 一定要注意以后要将积分项归零在合适的位置
float a=1.194,b=0.01,c=0.01;                   // a=1.194     0.8
float kp[4]={a,a,1.0,1.3};//kp[4]={a,a,0.4,0.8}  ×××××重要/***180度交互  kp[4]={a,a,1.0,0.8} ***/45度kp[4]={a,a,1.0,1.2}
float ki[4]={b,b,0.0,0.00};
float kd[4]={c,c,0.01,0.08};   //{c,c,0.01,0.02}
float kp_obstacle=0.5f;;//PID按照x、y顺序避障
float ki_obstacle=0.0f;
float kd_obstacle=0.0f;
float error_obstacle[8]={0.0f};
float last_error_obstacle[8]={0.0f};
float interaction_obstacle_x[8]={-80};
float interaction_obstacle_y[8]={-80};
float comd_obstacle_p[8]={0.0f};
float comd_obstacle_i[8]={0.0f};
float comd_obstacle_d[8]={0.0f};
float distance_obstacle;
float error[4]={0.0f,0.0f,0.0f,0.0f},last_error[4]={0.0f,0.0f,0.0f,0.0f},dt;
double time_now=0,last_time=0.0f,first_ensure_if_obstacle_time;
///////////////////////////////////////////用来发布指令
float comd_angle_x;
float comd_angle_y;
float comd_yaw_rate;
float comd_height;
float comd_height_v;
float comd_p[4]={0.0f,0.0f,0.0f,0.0f};//用来记录kp指令的大小x y yaw height
float comd_i[4]={0.0f,0.0f,0.0f,0.0f};
float comd_d[4]={0.0f,0.0f,0.0f,0.0f};
bool land=false;
bool take_off=false;
bool PID_control_of_xy=true;
bool first_land=true;
float take_off_height=1.3;
bool first_take_off=true;
float acc_total;
float acc_land_to_take_off=0.028;//起飞的加速度大小触发开关//TODO
double first_land_time;
//float last_model_choose;
float danger_obstacle_count=0;
bool danger_obstacle_take_off=false;
bool ob_danger_flag=false;
float danger_obstacle_take_off_height=1.8f;//1.8f;
extern float wx_px,wx_py,wx_pz;
///////////////////////////////////////////
CvMat *q_right_now=cvCreateMat(4,1,CV_32FC1);
CvMat *att_right_now=cvCreateMat(3,1,CV_32FC1);
int obstacle_bottom_num;
int total_obstacle_bottom_num=0;
#define PI 3.14159265358
float take_down_height=1.3;
bool first_zhuanquanquan=true;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////180度交互参数设计
int model_choose=2;     //【0】跟随  【1】 45度交互 【2】180度交互
bool no_land=false;
float iRobot_original_x;
float iRobot_original_y;
float comd_yaw;
bool first_follow=true;
bool first_land_180=true;

float cruise_velcity_total=0.5;
float angle_180=135;

float obstacle_theta,guidance_theta,min_roll_theta,min_roll_real_theta,roll_theta,roll_real_theta;
int min_roll_num;
float reverse_yaw_now,atan_iRobot,zhuanquan;
float parallel_land_time=1;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////巡航
int obstacle_amount_zy=0;
int obstacle_amount_yqt=0;
int obstacel_amount_total=0;
int All_obs_num=0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int flag_location;
int jrh_model;
float t1_rotate=1.0;
float t2_rotate=4.0;
//////////////////////////////////////////////////////////////////////////////////////////////////
bool circle_flag=false;
bool first_89_flag=false;
int circle_count=0;
float original_x_t,original_y_t;
float original_x_89,original_y_89;
float circle_target_distance=0.0;
float aim_position_x[6];
float aim_position_y[6];
bool refly=false;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*王宁远巡航变量定义（包含目标点，障碍物，飞行器）*/
float wny_x_t=0.0;		// target
float wny_y_t=0.0;
float wny_x_o=0.0;		// obstacle
float wny_y_o=0.0;
float wny_x_q=0.0;		// quadrotor
float wny_y_q=0.0;

float wny_v_tx=0.0;		// target_velocity 
float wny_v_ty=0.0;
float wny_v_a_x=0.0;		// avoid_velocity
float wny_v_a_y=0.0;

float wny_v_x=0.0;		// complex_velocity
float wny_v_y=0.0;
float wny_v_h=0.0;

float wny_k_target=1.0;		// proportionality of target
float wny_x_qt=0.0;
float wny_y_qt=0.0;
float wny_x_qt_limitation=0.7;
float wny_y_qt_limitation=0.7;
float wny_target_distance=0.0;

float wny_r_obstacle=2.0;	// radius of obstacle itself
float wny_R_obstacle=0.5;	// radius of obstacle area
float wny_k_o=0.0;		// proportionality of obstacle itself
float wny_L_o=0.0;		// proportionality of obstacle area
float wny_U=1.02;
float wny_T=0.70;

int wny_count=0;
int wny_num=0;
float wny_number=0.0;

float wny_v_t=0.0;
float wny_v_a=0.0;
float wny_v_at=0.0;
float wny_v_multi=0.0;

float wny_vx_limitation=0.5;
float wny_vy_limitation=0.5;

float wny_x_temp=0.0;
float wny_y_temp=0.0;
int wny_i=0;
int wny_j=0;
int wny_k=0;
int wny_n=0;
int wny_m=0;
int wny_obs_number=0;

float wny_x_qo[4];
float wny_y_qo[4];
float wny_obstacle_distance[4];
float wny_a[4];
float wny_b[4];
float wny_k_obstacle[4];
float wny_v_ax[4];
float wny_v_ay[4];

float wny_x_wait=0.0;
float wny_y_wait=0.0;
float wny_x_compare=0.0;
float wny_y_compare=0.0;

float wny_qx_ready=0.0;
float wny_qy_ready=0.0;
int wny_obs_count=0;
int wny_obs_num=0;

float wny_altitude_small=0.0;
float wny_altitude_big=0.0;
float wny_v_altitude=0.0;
int wny_altitude_flag=0;
int wny_altitude_count=0;

float wny_two_add=0.0;			//add for obstacle_number=3
float wny_three_add=0.0;

float wny_two_sum=0.0;			//sum for obstacle_number=3
float wny_three_sum=0.0;
float wny_four_sum=0.0;


float wny_yaw_now=0.0;// consider the rotation of quadrotor ( yaw is to be calculated)
float wny_one_begin=0.0;		// define four blind area (take to rad)
float wny_one_end=0.0;
float wny_two_begin=0.0;
float wny_two_end=0.0;
float wny_three_begin=0.0;
float wny_three_end=0.0;
float wny_four_begin=0.0;
float wny_four_end=0.0;
int wny_flagone=0;
int wny_flagtwo=0;
int wny_flagthree=0;
int wny_flagfour=0;
int wny_angleone=0;
int wny_angletwo=0;
int wny_anglethree=0;
int wny_anglefour=0;
float wny_angle=0.0;
float wny_vx=0.0;
float wny_vy=0.0;
int wny_judgement=0;
float wny_vyaw_extra=0.0;
float wny_v_yaw=0.0;
float original_yaw;
float plus_yaw=0.0;
struct obstacle
{
	float x[8],y[8];
	int num;
	int flag;
};
struct obstacle obstacle_yqt;
struct obstacle obstacle_zy;
struct obstacle obstacle_all;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*王宁远巡航回调函数定义（飞行器定位，障碍物定位）*/
bool first_flag = true;

void vehicle_pos_callback(const iarc007::vehicle_pos& wny)
{
	CvMat* pos_mat = cvCreateMat(3,3,CV_32FC1);
	cvmSet(pos_mat, 0, 0, wny.pos.x);
	cvmSet(pos_mat, 1, 0, wny.pos.y);
	cvmSet(pos_mat, 2, 0, wny.pos.z);
	//printf("wny.pos.x====%f\n",wny.pos.x);
	//printf("wny.pos.y====%f\n",wny.pos.y);	
	//cvGEMM(R_g_b,pos_mat,1,NULL,0,pos_mat);
	if(first_flag){
		
		first_flag = false;	
		wny_px = cvmGet(pos_mat, 0, 0);
		wny_py = cvmGet(pos_mat, 1, 0);
		wny_pz = cvmGet(pos_mat, 2, 0);
	}
	else{
		wny_px = (wny_px + cvmGet(pos_mat, 0, 0))/2.0;
		wny_py = (wny_py + cvmGet(pos_mat, 1, 0))/2.0;
		wny_pz = (wny_pz + cvmGet(pos_mat, 2, 0))/2.0;
	}
	
	cvReleaseMat(&pos_mat);
}

void yqt_obstacle_callback(const iarc007::obstacle& wny1)
{
	obstacle_yqt.num=wny1.num;
	//printf("obstacle.num====%d\n",obstacle.num);
	//wny.obstacle_x.resize(wny.num);
	//wny.obstacle_y.resize(wny.num);
	for(int wny_p=0;wny_p<wny1.num;wny_p++)
	{
		obstacle_yqt.x[wny_p]=wny1.obstacle_x[wny_p];
		obstacle_yqt.y[wny_p]=wny1.obstacle_y[wny_p];
		//printf("obstacle(%d)_x===%f\n",wny_p,obstacle.x[wny_p]);
		//printf("obstacle(%d)_y===%f\n",wny_p,obstacle.y[wny_p]);
	}
	obstacle_yqt.flag=wny1.biaozhiwei[0];
	//printf("obstacle.flag===%d\n",obstacle.flag);
}

void zy_obstacle_callback(const iarc007::obstacle& wny2)
{
		obstacle_zy.num=wny2.num;
		//printf("obstacle.num====%d\n",obstacle.num);
		//wny.obstacle_x.resize(wny.num);
		//wny.obstacle_y.resize(wny.num);
		for(int wny_q=0;wny_q<wny2.num;wny_q++)
		{
			obstacle_zy.x[wny_q]=wny2.obstacle_x[wny_q];
			obstacle_zy.y[wny_q]=wny2.obstacle_y[wny_q];
			//printf("obstacle(%d)_x===%f\n",wny_p,obstacle.x[wny_p]);
			//printf("obstacle(%d)_y===%f\n",wny_p,obstacle.y[wny_p]);
		}
		obstacle_zy.flag=wny2.biaozhiwei[0];
		//printf("obstacle.flag===%d\n",obstacle.flag);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FILE *comd_setpoint=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/comd_setpoint.txt","w");
FILE *comd_setpoint_limited=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/comd_setpoint_limited.txt","w");
FILE *comd_setpoint_block=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/comd_setpoint_block.txt","w");
FILE *error_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/error.txt","w");
FILE *error_block_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/error_block.txt","w");
FILE *parent_list_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/parent_list.txt","w");
FILE *obstacle_pos_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/obstacle.txt","w");
FILE *obstacle_comd_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/obstacle_comd.txt","w");
FILE *global_race_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/global_race.txt","w");
FILE *follow_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/follow.txt","w");
FILE *follow_txt_block=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/follow_block.txt","w");
FILE *original_signal_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/original_signal.txt","w");
FILE *original_signal_txt_block=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/original_signal_block.txt","w");

void dynamic_callback(iarc007::iarc007Config &config, uint32_t level) {
	kp[0]=config.double_kp_1;kp[1]=config.double_kp_2;kp[2]=config.double_kp_3;kp[3]=config.double_kp_4;
	ki[0]=config.double_ki_1;ki[1]=config.double_ki_2;ki[2]=config.double_ki_3;ki[3]=config.double_ki_4;
	kd[0]=config.double_kd_1;kd[1]=config.double_kd_2;kd[2]=config.double_kd_3;kd[3]=config.double_kd_4;

	ROS_INFO("Reconfigure Request: kp_1=%f ki_1=%f kd_1=%f\n",
		kp[0],ki[0],kd[0]);
	ROS_INFO("Reconfigure Request: kp_2=%f ki_2=%f kd_2=%f\n",
		kp[1],ki[1],kd[1]);
	ROS_INFO("Reconfigure Request: kp_3=%f ki_3=%f kd_3=%f\n",
		kp[2],ki[2],kd[2]);
	ROS_INFO("Reconfigure Request: kp_4=%f ki_4=%f kd_4=%f\n",
		kp[3],ki[3],kd[3]);
}







void control()
{
    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);
    JRHDrone* jrh_drone=new JRHDrone(nh);

    jrh_drone->ultrasonic.ranges.resize(5);
    ros::Rate rate(50);
    jrh_drone->object_pos_dynamic.target_x.resize(20);
    jrh_drone->object_pos_dynamic.target_y.resize(20);
    jrh_drone->object_pos_dynamic.target_dir.resize(20);

	//dynamic_reconfigure::Server<iarc007::iarc007Config> server;
	//dynamic_reconfigure::Server<iarc007::iarc007Config>::CallbackType f;
	//f = boost::bind(&dynamic_callback, _1, _2);
	//server.setCallback(f);

    ros::Subscriber obstacle_subscriber;
    ros::Subscriber obstacle_yqt_subscriber;

    obstacle_subscriber=nh.subscribe("/zy_obstacle", 1, zy_obstacle_callback);
    obstacle_yqt_subscriber=nh.subscribe("/yqt_obstacle", 1, yqt_obstacle_callback);

while(ros::ok())
{
	ros::spinOnce();
	while((abs(drone->rc_channels.mode+8000.0)<1)||abs(drone->rc_channels.mode)<1)
	{
		ros::spinOnce();
		drone-> release_sdk_permission_control();
		ROS_INFO("obtain  error !!! %f",drone->rc_channels.mode);
		usleep(5000);
		first_interaction_45=true;
		first_take_off=true;
		land=false;
		first_zhuanquanquan=true;
		first_follow=true;
		take_off=false;
	}




//TODO
//////////////////////////////////////////////////////////////////////////////////////////////比赛快速调参
 distance_ahead_45=0.0;distance_ahead_180=0.1;//机器人向前叠加距离
 height_limited=1.5;//45度/180度/跟踪模式高度限幅
 forbid_distance_land=1.8;//距离障碍物这个距离时禁止降落
 land_condition_r_45=0.08;land_condition_erryaw_45=15; land_condition_height_45=1;//45度降落条件
 land_condition_r_180=0.15; land_condition_erryaw_180=10;land_condition_height_180=1.6;//180度降落条件
 parallel_land_time_45_first=1;parallel_land_time_45_second=3;//45度降落两段持续时间
 parallel_land_time_180_first=1;parallel_land_time_180_second=5;//180度降落两段持续时间
 obstacle_danger_distance=1.65;//避障反向跑的距离
 velcity_land_45=0.2;//45度降落速度，维持大小
 comd_height_if_obstacle = 1.25;
 comd_follow_height = 1.5;
 velcity_zhuanquan=0.3;
 yaw_rate_zhuanquan=18;
 velcity_land_180=0.5;//180度降落速度大小
 parallel_land_velcity_180 = 0.2;//180度平行速度大小
//////////////////////////////////////////////////////////////////////////////////////////////





if(abs(drone->rc_channels.mode-8000.0)<1)
{
	if(drone->rc_channels.gear==(-4545))
	{
		usleep(100);	
		drone->request_sdk_permission_control();
	}
	//////////////////////////////////////////////////////////////////////////

	flag_location=2;
	wx_iRobot_num=jrh_drone->object_pos_dynamic.target_num;

	//计算得到当前的姿态角att_right_now
   cvmSet(q_right_now, 0, 0, jrh_drone->attitude_quaternion.q0);
   cvmSet(q_right_now, 1, 0, jrh_drone->attitude_quaternion.q1);
   cvmSet(q_right_now, 2, 0, jrh_drone->attitude_quaternion.q2);
   cvmSet(q_right_now, 3, 0, jrh_drone->attitude_quaternion.q3);
   Quaternion_To_Euler(q_right_now,att_right_now);

   printf("%f  %f  %f  %f\n",jrh_drone->attitude_quaternion.q0,jrh_drone->attitude_quaternion.q1,jrh_drone->attitude_quaternion.q2,jrh_drone->attitude_quaternion.q3);
	///////////////////////////////////////////
   float yaw_now=(-cvmGet(att_init,2,0)+cvmGet(att_right_now,2,0));

		if(yaw_now>PI)
		{
			yaw_now=yaw_now-2*PI;
		}
		else if(yaw_now<-PI)
		{
			yaw_now=yaw_now+2*PI;
		}
   ////////////////////////////////////////
  printf("yaw_now %f\n",yaw_now*180/PI);
  fprintf(model_txt,"yaw_now %f \n",yaw_now*180/PI);


/////////////////////////////////////////////////////边界约束////////////////////////////////////////////////////
//	if(wx_px>18 || wx_px<2 || wx_py>18|| wx_py<2)//出界
//	{
//		boundry_flag=1;
//		model_choose=3;
//	}
//	else//未出界
//	{
//		boundry_flag=0;
//	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
for(int i=0;i<wx_iRobot_num;i++)
{
	wx_iRobot_x[i]=(jrh_drone->object_pos_dynamic.target_x[i])*0.01;
	wx_iRobot_y[i]=(jrh_drone->object_pos_dynamic.target_y[i])*0.01;
	wx_iRobot_theta[i]=jrh_drone->object_pos_dynamic.target_dir[i];
	wx_iRobot_serial_num[i]=jrh_drone->object_pos_dynamic.serial_flag[i];
}

if(land==false && take_off==false && boundry_flag==0)////////////////////////////////策略判断
{
	jrh_model=model_switch(wx_iRobot_num,wx_iRobot_x,wx_iRobot_y,wx_iRobot_theta,flag_location, t1_rotate, t2_rotate);
	model_choose=jrh_model;
}

if(take_off==true  && boundry_flag==0)///////////////////////////////////////////////
{
	jrh_model=model_switch(wx_iRobot_num,wx_iRobot_x,wx_iRobot_y,wx_iRobot_theta,flag_location, t1_rotate, t2_rotate);
	model_choose=jrh_model;
	if(danger_obstacle_take_off==1)//由于障碍物使model_choose=88
	{
		if(jrh_drone->ultrasonic.ranges[0]<danger_obstacle_take_off_height)
		{
			model_choose=88;
		}
		else
		{
			danger_obstacle_take_off=false;
			take_off=false;
			first_interaction_45 = true;
			first_take_off = true;
			first_follow=true;
			first_zhuanquanquan=true;
			obstacle_amount_zy=jrh_drone->obstacle_pos_dynamic.num;//从张雨订阅一共看到几个机器人
			obstacle_amount_yqt=jrh_drone->obstacle_pos_dynamic_yqt.num;//从于庆涛订阅一共看到几个机器人
			obstacle_bottom_num=jrh_drone->obstacle_bottom_num.num;
			if(obstacle_amount_zy+obstacle_amount_yqt+obstacle_bottom_num==0)
			{
				model_choose=89;//下降到制定高度
				//original_yaw=yaw_now*180/PI;
				first_89_flag=0;
			}
			else
			{
				model_choose=90;//悬停于高空  重新进入90模式
				circle_flag=0;
				total_obstacle_bottom_num=0;
			}
		}
	}
	else//由于交互过程丢目标使model_choose=88
	{
		if((jrh_drone->ultrasonic.ranges[0]<take_off_height))
		{
			if(model_choose==3 || model_choose==1 || model_choose == 2)
			{
				model_choose=88;
			}
		}
		else
		{
			take_off=false;
			first_interaction_45 = true;
			first_zhuanquanquan=true;
			first_take_off = true;
			first_follow=true;
		}

		if(model_choose==1||model_choose==2 || model_choose==0)
		{
			take_off=false;
			first_zhuanquanquan=true;
			first_interaction_45 = true;
			first_take_off = true;
			first_follow=true;
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////// 丢失目标起飞
if(PID_control_of_xy==1 && model_choose==3 && (last_model_choose==1 || last_model_choose==2))
{
	model_choose=88;
	take_off=true;
}
/////////////////////////////////////////////////////////////////////////////////////// 0 1 2 3相互切换更新变量
if((model_choose==1||model_choose==2 || model_choose==3)&&last_model_choose==0)
{
	take_off=false;
	first_interaction_45 = true;
	first_take_off = true;
	first_follow=true;
	first_zhuanquanquan=true;
}
if((model_choose==0||model_choose==2 || model_choose==3)&&last_model_choose==1)
{
	take_off=false;
	first_interaction_45 = true;
	first_take_off = true;
	first_follow=true;
	first_zhuanquanquan=true;
}
if((model_choose==1||model_choose==0 || model_choose==3)&&last_model_choose==2)
{
	take_off=false;
	first_interaction_45 = true;
	first_take_off = true;
	first_follow=true;
	first_zhuanquanquan=true;
}
if((model_choose==0||model_choose==1 || model_choose==2)&&last_model_choose==3)
{
	take_off=false;
	first_interaction_45 = true;
	first_take_off = true;
	first_follow=true;
	first_zhuanquanquan=true;
}
///////////////////////////////////////////////////////////////////////////////////////
if((model_choose==0 ||  model_choose==1 || model_choose==2) && jrh_drone->ultrasonic.ranges[0]>1 && iRobot_num>0 && land==false)
{
	if(fabs(iRobot_theta*180/PI-572.957795)<0.2)
	{
		model_choose=3;
		fprintf(model_txt,"进没进来\n");
	}
}

if(last_model_choose==89)
{
	refly=0;
	for(int i=0;i<obstacel_amount_total;i++)
	{
		distance_obstacle=sqrt(interaction_obstacle_x[i]*interaction_obstacle_x[i]+interaction_obstacle_y[i]*interaction_obstacle_y[i])-obstacle_danger_distance;
		if(distance_obstacle<0)
		{
			refly=1;
			printf("refly_dis %f",distance_obstacle);
			fprintf(model_txt,"refly_dis %f",distance_obstacle);
			break;
		}
	}

	if(obstacle_bottom_num>0 || refly==1)//if(obstacle_amount_zy+obstacle_amount_yqt+obstacle_bottom_num>0)//////此处根据情况是否加入障碍物距离限制条件
	{
		if(refly==0)
		{
			printf("refly_obstacle_bottom_num %d",obstacle_bottom_num);
			fprintf(model_txt,"refly_obstacle_bottom_num %d",obstacle_bottom_num);
		}
		else
		{
			printf("refly_dis %f",distance_obstacle);
			fprintf(model_txt,"refly_dis %f",distance_obstacle);
		}
		danger_obstacle_take_off=true;
		model_choose=88;
		take_off=true;
	}
	else if(jrh_drone->ultrasonic.ranges[0]>take_down_height)
	{
		model_choose=89;
	}
}

if(last_model_choose==90)
{
	if(circle_flag==0)//if(obstacle_amount_zy+obstacle_amount_yqt+obstacle_bottom_num>0)
	{
		model_choose=90;//单纯model_choose=90 继续进入90模式
	}
	else if(circle_flag==1)
	{
		if(total_obstacle_bottom_num>0)
		{
			model_choose=90;//model_choose=90  +  circle_flag=0 重新进入90模式
			circle_flag=0;
			total_obstacle_bottom_num=0;
		}
		else
		{
			model_choose=89;
			first_89_flag=0;
		}
	}
}


	jrh_drone->jrh_model_choose.header.stamp=ros::Time::now();
	jrh_drone->jrh_model_choose.model_choose=model_choose;
	jrh_drone->jrh_model_choose.target_num_choose=wx_iRobot_serial_num[jrh_iRobot_i];
	jrh_drone->jrh_model_choose_publisher.publish(jrh_drone->jrh_model_choose);


	last_model_choose=model_choose;
	fprintf(model_txt,"**************************************************  time  %f  ****************************\n",tic());
	printf("*******************model_choose     %d   %d***********\n",model_choose,land);
	fprintf(model_txt,"model_choose     %d   %d\n",model_choose,land);
	printf("wx_px  %f  wx_py  %f\n",wx_px,wx_py);
	fprintf(model_txt,"wx_px  %f  wx_py  %f  boundry_flag %d\n",wx_px,wx_py,boundry_flag);

	iRobot_x=(jrh_drone->object_pos_dynamic.target_x[jrh_iRobot_i])*0.01;
	iRobot_y=(jrh_drone->object_pos_dynamic.target_y[jrh_iRobot_i])*0.01;
	iRobot_theta=jrh_drone->object_pos_dynamic.target_dir[jrh_iRobot_i];
	iRobot_num=jrh_drone->object_pos_dynamic.target_num;
	iRobot_original_x=iRobot_x;
	iRobot_original_y=iRobot_y;

	printf("iRobot_num  %d--- iRobot_theta %f\n",iRobot_num,iRobot_theta*180/PI);
	fprintf(model_txt,"iRobot_num  %d--- iRobot_theta %f\n",iRobot_num,iRobot_theta*180/PI);
	//TODO


	//////////////////////////////////////////////////////////////////////////
	obstacle_amount_zy=jrh_drone->obstacle_pos_dynamic.num;//从张雨订阅一共看到几个机器人
	obstacle_amount_yqt=jrh_drone->obstacle_pos_dynamic_yqt.num;//从于庆涛订阅一共看到几个机器人
	obstacel_amount_total=obstacle_amount_zy+obstacle_amount_yqt;
	if((obstacel_amount_total)>0)
	{
		for(int i=0;i<obstacle_amount_zy;i++)//从张雨那里拿到障碍物位置信息
		{
			interaction_obstacle_x[i]=jrh_drone->obstacle_pos_dynamic.obstacle_x[i];
			interaction_obstacle_y[i]=jrh_drone->obstacle_pos_dynamic.obstacle_y[i];
			fprintf(obstacle_pos_txt,"time  %f  total_num %d num %d obstacle_x %f  obstacle_y %f\n",tic(),obstacel_amount_total,i+1,interaction_obstacle_x[i],interaction_obstacle_y[i]);
			printf("zy_ob_total%d--num %d--ob_r_vt %f -- ob_x %f--ob_y %f\n",obstacel_amount_total,i+1,sqrt(interaction_obstacle_x[i]*interaction_obstacle_x[i]+interaction_obstacle_y[i]*interaction_obstacle_y[i]),interaction_obstacle_x[i],interaction_obstacle_y[i]);
			fprintf(model_txt,"zy_obsatcle_total%d num %d --obsatcle_r_vt %f -- obstacle_x %f -- obstacle_y %f\n",obstacel_amount_total,i+1,sqrt(interaction_obstacle_x[i]*interaction_obstacle_x[i]+interaction_obstacle_y[i]*interaction_obstacle_y[i]),interaction_obstacle_x[i],interaction_obstacle_y[i]);
			//printf(" obstacle_x %f  obstacle_y %f \n",interaction_obstacle_x[i],interaction_obstacle_y[i]);
		}
		for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)////从于庆涛那里拿到障碍物位置信息
		{
			interaction_obstacle_x[i]=jrh_drone->obstacle_pos_dynamic_yqt.obstacle_x[i-obstacle_amount_zy];
			interaction_obstacle_y[i]=jrh_drone->obstacle_pos_dynamic_yqt.obstacle_y[i-obstacle_amount_zy];
			fprintf(obstacle_pos_txt,"time  %f  total_num %d num %d obstacle_x %f  obstacle_y %f\n",tic(),obstacel_amount_total,i+1,interaction_obstacle_x[i],interaction_obstacle_y[i]);
			printf("yqt_ob_total%d--num %d--ob_r_vt %f -- ob_x %f--ob_y %f\n",obstacel_amount_total,i+1,sqrt(interaction_obstacle_x[i]*interaction_obstacle_x[i]+interaction_obstacle_y[i]*interaction_obstacle_y[i]),interaction_obstacle_x[i],interaction_obstacle_y[i]);
			fprintf(model_txt,"yqt_obsatcle_total%d num %d --obsatcle_r_vt %f -- obstacle_x %f -- obstacle_y %f\n",obstacel_amount_total,i+1,sqrt(interaction_obstacle_x[i]*interaction_obstacle_x[i]+interaction_obstacle_y[i]*interaction_obstacle_y[i]),interaction_obstacle_x[i],interaction_obstacle_y[i]);
			//printf(" obstacle_x %f  obstacle_y %f \n",interaction_obstacle_x[i],interaction_obstacle_y[i]);
		}
		for(int i=obstacel_amount_total;i<8;i++)
		{
			interaction_obstacle_x[i]=-80;
			interaction_obstacle_y[i]=-80;
		}
	}
	else
	{
		printf("obstacle   total_num 0  *****************no obstacle**************\n");
		fprintf(model_txt,"obstacle   total_num 0  *****************no obstacle**************\n");
		for(int i=0;i<8;i++)
		{
			interaction_obstacle_x[i]=-80;
			interaction_obstacle_y[i]=-80;
		}
	}
	fprintf(original_signal_txt," [time]  %f  [i_num]  %d  [i_x]  %f  [i_y]  %f  [i_theta]  %f  [obs_num]  %d  [obs_x0]  %f  [obs_y0]  %f  [obs_x1]  %f  [obs_y1]  %f  \n",tic(),iRobot_num,iRobot_x,iRobot_y,iRobot_theta,obstacel_amount_total,interaction_obstacle_x[0],interaction_obstacle_y[0],interaction_obstacle_x[1],interaction_obstacle_y[1]);
	fprintf(original_signal_txt_block," %f  %d  %f  %f  %f  %d  %f  %f  %f  %f  \n",tic(),iRobot_num,iRobot_x,iRobot_y,iRobot_theta,obstacel_amount_total,interaction_obstacle_x[0],interaction_obstacle_y[0],interaction_obstacle_x[1],interaction_obstacle_y[1]);
	printf(" height_now %f\n",jrh_drone->ultrasonic.ranges[0]);
	fprintf(model_txt," height_now %f\n",jrh_drone->ultrasonic.ranges[0]);
//////////////////////////////////////////////////////////////////////////a=1.194,b=0.01,c=0.01;
	  if(model_choose==0)//跟随模式PID参数
	  {
		  kp[0]=1.194; kp[1]=1.194; kp[2]=0.4; kp[3]=0.4;
		  ki[0]=0.01; ki[1]=0.01; ki[2]=0.00; ki[3]=0.00;
		  kd[0]=0.01; kd[1]=0.01; kd[2]=0.01; kd[3]=0.02;
	  }
	  if(model_choose==1)//45度PID参数
	  {
		  if(jrh_drone->ultrasonic.ranges[0]>1.5)
		  {
			  kp[0]=1.5; kp[1]=1.5; kp[2]=0.8; kp[3]=0.8;//1.194 1.194 1.0 1.3
			  ki[0]=0.01; ki[1]=0.01; ki[2]=0.00; ki[3]=0.00;
			  kd[0]=0.01; kd[1]=0.01; kd[2]=0.01; kd[3]=0.02;
		  }
		  else if(jrh_drone->ultrasonic.ranges[0]>0.7)
		  {
			  kp[0]=1.194; kp[1]=1.194; kp[2]=0.8; kp[3]=0.8;//1.194 1.194 1.0 1.3
			  ki[0]=0.01; ki[1]=0.01; ki[2]=0.00; ki[3]=0.00;
			  kd[0]=0.01; kd[1]=0.01; kd[2]=0.01; kd[3]=0.02;
		  }
		  else
		  {
			  kp[0]=1.194; kp[1]=1.194; kp[2]=0.8; kp[3]=1.5;
			  ki[0]=0.01; ki[1]=0.01; ki[2]=0.00; ki[3]=0.00;
			  kd[0]=0.01; kd[1]=0.01; kd[2]=0.01; kd[3]=0.05;
		  }
	  }

	  if(model_choose==2)//180度PID参数
	  {
		  if(jrh_drone->ultrasonic.ranges[0]>0.7)
		  {
			  kp[0]=0.8; kp[1]=0.8; kp[2]=1.0; kp[3]=1.0;
			  kd[0]=0.02; kd[1]=0.02; kd[2]=0.01; kd[3]=0.0;
		  }
		  else if(jrh_drone->ultrasonic.ranges[0]>0.5)
		  {
			  kp[0]=0.8; kp[1]=0.8; kp[2]=1.0; kp[3]=0.8;
			  kd[0]=0.02; kd[1]=0.02; kd[2]=0.01; kd[3]=0.04;
		  }
		  else
		  {
			  kp[0]=0.5; kp[1]=0.5; kp[2]=1.0; kp[3]=0.5;
			  kd[0]=0.02; kd[1]=0.02; kd[2]=0.01; kd[3]=0.03;
		  }
		  if(fabs(yaw_now*180/PI-iRobot_theta*180/PI)<angle_180)
		  {
			  kp[0]=0.4; kp[1]=0.4;  kp[2]=0.5; kp[3]=0.8;
			  kd[0]=0.02; kd[1]=0.02; kd[2]=0.01; kd[3]=0.0;
		  }

		  ki[0]=0.01; ki[1]=0.01; ki[2]=0.00; ki[3]=0.02;
	  }

	  if(model_choose==88 || model_choose==89 || model_choose==90)
	  {
		  kp[0]=1.2; kp[1]=1.2; kp[2]=1.0; kp[3]=0.7;
		  ki[0]=0.01; ki[1]=0.01; ki[2]=0.00; ki[3]=0.00;
		  kd[0]=0.01; kd[1]=0.01; kd[2]=0.01; kd[3]=0.02;
	  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//45\180度交互程序开始
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if((model_choose==1||model_choose==2 )&&(iRobot_num>0))
{
////////////////////////////////////////////////////////////////////////////////////////////////刚进入交互模式时采取原地转圈
	/*
	if(first_zhuanquanquan==true)
	{
		first_ensure_if_obstacle_time=tic();
		time_now=first_ensure_if_obstacle_time;
		first_zhuanquanquan=false;
	}
while((time_now-first_ensure_if_obstacle_time)<1)
{
	ros::spinOnce();
	time_now=tic();
	reverse_yaw_now=yaw_now-2*PI;

	if(reverse_yaw_now<-PI)
	{
		reverse_yaw_now=reverse_yaw_now+2*PI;
	}
	atan_iRobot=atan2(iRobot_original_y,iRobot_original_x);
	zhuanquan=atan_iRobot-yaw_now;
	if(zhuanquan<-PI)
	{
		zhuanquan=zhuanquan+28*PI;
	}
	else if(zhuanquan>PI)
	{
		zhuanquan=zhuanquan-2*PI;
	}
	if(zhuanquan<0 )
	{
		comd_yaw_rate=-yaw_rate_zhuanquan;
	}
	else
	{
		comd_yaw_rate=yaw_rate_zhuanquan;
	}


	if(comd_yaw_rate<0)
	{
		comd_angle_x=velcity_zhuanquan*cos(iRobot_theta);
		comd_angle_y=velcity_zhuanquan*sin(iRobot_theta);//-0.417*PI
	}
	else
	{
		comd_angle_x=velcity_zhuanquan*cos(iRobot_theta);
		comd_angle_y=velcity_zhuanquan*sin(iRobot_theta);
	}
	comd_height_v=0;
	fprintf(model_txt,"first_ensure_if_obstacle_time  %f  time_now  %f\n",first_ensure_if_obstacle_time,time_now);
	fprintf(model_txt,"转圈圈 comd_yaw_rate %f  atan_iRobot  %f\n",comd_yaw_rate,atan_iRobot);
fprintf(model_txt,"comd_angle_x %f  comd_angle_y %f\n",comd_angle_x,comd_angle_y);
	////////////////////////////////////////////将指令从赛场动坐标系到地理坐标系
	float comd_vx=comd_angle_x*cvmGet(R_init,0,0)+comd_angle_y*cvmGet(R_init,0,1)+comd_height_v*cvmGet(R_init,0,2);
	float comd_vy=comd_angle_x*cvmGet(R_init,1,0)+comd_angle_y*cvmGet(R_init,1,1)+comd_height_v*cvmGet(R_init,1,2);
	float comd_zv=comd_angle_x*cvmGet(R_init,2,0)+comd_angle_y*cvmGet(R_init,2,1)+comd_height_v*cvmGet(R_init,2,2);
////////////////////////////////////////////发送指令地理坐标系中
	drone->velocity_control(0x49,comd_vx,comd_vy,comd_zv,comd_yaw_rate);
	rate.sleep();
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		if(first_interaction_45)
		{
			last_time=0;
			time_now=tic();
			dt=0.0f;
			comd_i[0]=0;
			comd_i[1]=0;
			comd_i[2]=0;
			comd_i[3]=0;
		    comd_obstacle_i[8]={0.0f};
			first_interaction_45=false;
			land=false;
			first_land=true;
			first_land_180=true;
		}
		else
		{
			time_now=tic();
			dt=time_now-last_time;
		}

		iRobot_x=(jrh_drone->object_pos_dynamic.target_x[jrh_iRobot_i])*0.01;
		iRobot_y=(jrh_drone->object_pos_dynamic.target_y[jrh_iRobot_i])*0.01;
		iRobot_theta=jrh_drone->object_pos_dynamic.target_dir[jrh_iRobot_i];
		iRobot_num=jrh_drone->object_pos_dynamic.target_num;
		iRobot_original_x=iRobot_x;
		iRobot_original_y=iRobot_y;

		//TODO


//////////////////////////////////////////////////////////////////////////
//TODO
obstacle_amount_zy=jrh_drone->obstacle_pos_dynamic.num;//从张雨订阅一共看到几个机器人
obstacle_amount_yqt=jrh_drone->obstacle_pos_dynamic_yqt.num;//从于庆涛订阅一共看到几个机器人
obstacel_amount_total=obstacle_amount_zy+obstacle_amount_yqt;
//得到所有障碍物信息，如果没有障碍物信息则返回-80信息

if((obstacel_amount_total)>0)
{
	for(int i=0;i<obstacle_amount_zy;i++)//从张雨那里拿到障碍物位置信息
	{
		interaction_obstacle_x[i]=jrh_drone->obstacle_pos_dynamic.obstacle_x[i];
		interaction_obstacle_y[i]=jrh_drone->obstacle_pos_dynamic.obstacle_y[i];
	}
	for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)////从于庆涛那里拿到障碍物位置信息
	{
		interaction_obstacle_x[i]=jrh_drone->obstacle_pos_dynamic_yqt.obstacle_x[i-obstacle_amount_zy];
		interaction_obstacle_y[i]=jrh_drone->obstacle_pos_dynamic_yqt.obstacle_y[i-obstacle_amount_zy];
		//printf(" obstacle_x %f  obstacle_y %f \n",interaction_obstacle_x[i],interaction_obstacle_y[i]);
	}
	for(int i=obstacel_amount_total;i<8;i++)
	{
		interaction_obstacle_x[i]=-80;
		interaction_obstacle_y[i]=-80;
	}
}
else
{
	for(int i=0;i<8;i++)
	{
		interaction_obstacle_x[i]=-80;
		interaction_obstacle_y[i]=-80;
	}
}

		///////////////////////////////////////////
		//计算得到当前的姿态角att_right_now
	   cvmSet(q_right_now, 0, 0, jrh_drone->attitude_quaternion.q0);
	   cvmSet(q_right_now, 1, 0, jrh_drone->attitude_quaternion.q1);
	   cvmSet(q_right_now, 2, 0, jrh_drone->attitude_quaternion.q2);
	   cvmSet(q_right_now, 3, 0, jrh_drone->attitude_quaternion.q3);
	   Quaternion_To_Euler(q_right_now,att_right_now);
		///////////////////////////////////////////

	   yaw_now=(-cvmGet(att_init,2,0)+cvmGet(att_right_now,2,0));
	   ////////////////////////////////////////

	   		if(yaw_now>PI)
	   		{
	   			yaw_now=yaw_now-2*PI;
	   		}
	   		else if(yaw_now<-PI)
	   		{
	   			yaw_now=yaw_now+2*PI;
	   		}
	 	   ////////////////////////////////////////
 	    printf("yaw_now %f\n",yaw_now*180/PI);
 	    fprintf(model_txt,"yaw_now %f \n",yaw_now*180/PI);
	    r_vt_original=sqrt(iRobot_original_x*iRobot_original_x+iRobot_original_y*iRobot_original_y);
		printf("yaw_now %f  r_vt_original %f\n",yaw_now*180/PI,r_vt_original);
	    fprintf(model_txt,"yaw_now %f  r_vt_original %f\n",yaw_now*180/PI,r_vt_original);

	    if(model_choose==1)
	    {
			iRobot_x=iRobot_x+ distance_ahead_45*cos(iRobot_theta)-cos(yaw_now)*fabs(-0.132*sin(-cvmGet(att_right_now,1,0))+0.12*cos(-cvmGet(att_right_now,1,0)));
			iRobot_y=iRobot_y+ distance_ahead_45*sin(iRobot_theta)-sin(yaw_now)*fabs(-0.132*sin(-cvmGet(att_right_now,1,0))+0.12*cos(-cvmGet(att_right_now,1,0)));
			r_vt=sqrt(iRobot_x*iRobot_x+iRobot_y*iRobot_y);
			if(jrh_drone->ultrasonic.ranges[0]>1.5)
			{
				iRobot_x=iRobot_x+0.0*cos(iRobot_theta)-cos(yaw_now)*fabs(-0.132*sin(-cvmGet(att_right_now,1,0))+0.12*cos(-cvmGet(att_right_now,1,0)));
				iRobot_y=iRobot_y+0.0*sin(iRobot_theta)-sin(yaw_now)*fabs(-0.132*sin(-cvmGet(att_right_now,1,0))+0.12*cos(-cvmGet(att_right_now,1,0)));
				r_vt=sqrt(iRobot_x*iRobot_x+iRobot_y*iRobot_y);
			}
	    	if(fabs(iRobot_theta*180/PI-572.957795)<0.2)//||fabs(yaw_now*180/PI-iRobot_theta*180/PI)<angle_180)
	    	{
	    		iRobot_x=iRobot_x;
	    		iRobot_y=iRobot_y;
				r_vt = sqrt(iRobot_x*iRobot_x + iRobot_y*iRobot_y);
	    	}
	    }
	    else if(model_choose==2)
	    {
	    	if(fabs(iRobot_theta*180/PI-572.957795)<0.2)//||fabs(yaw_now*180/PI-iRobot_theta*180/PI)<angle_180)
	    	{
	    		iRobot_x=iRobot_x;
	    		iRobot_y=iRobot_y;
	    	}
	    	else
	    	{
				iRobot_x=iRobot_x+ distance_ahead_180*cos(iRobot_theta)-cos(yaw_now)*fabs(-0.132*sin(-cvmGet(att_right_now,1,0))+0.12*cos(-cvmGet(att_right_now,1,0)));
				iRobot_y=iRobot_y+ distance_ahead_180*sin(iRobot_theta)-sin(yaw_now)*fabs(-0.132*sin(-cvmGet(att_right_now,1,0))+0.12*cos(-cvmGet(att_right_now,1,0)));
	    	}
			r_vt=sqrt(iRobot_x*iRobot_x+iRobot_y*iRobot_y);
	    }

		pos_now[2]=jrh_drone->ultrasonic.ranges[0];

///////////////////////////////////////////相机高度，规划高度		
		if(model_choose==1)
		{
			if(fabs(r_vt_original-0.1)>0.4)
			{
				height_H=r_vt_original*1.5+0.42;
				height_L=r_vt_original*1.5+0.195;
			}
			else if(fabs(r_vt_original-0.1)>0.1 && fabs(r_vt_original-0.1)<0.4)
			{
				height_H=fabs(r_vt_original-0.1)*2.05+0.345;
				height_L=r_vt_original*1.5+0.195;
			}
			else if(fabs(r_vt_original-0.1)<0.1)
			{
				height_H=0.55;
				height_L=0.495;
			}

			if(pos_now[2]>height_H+0.031)
			{
				comd_height=height_H+0.031;
			}
			else if(pos_now[2]>height_L+0.031)
			{
				comd_height=pos_now[2];

			}
			else
			{
				comd_height=height_L+0.031;
			}
			if(ob_danger_flag==true)
			{
				height_H=comd_height_if_obstacle;
				height_L=comd_height_if_obstacle-0.1;
			}
		}
		else if(model_choose==2)
		{
			if(fabs(r_vt_original- distance_ahead_180)>0.74- distance_ahead_180)
			{
				height_H = fabs(r_vt_original- distance_ahead_180)*1.5 + distance_ahead_180*1.5 + 0.42;
				height_L =fabs( r_vt_original- distance_ahead_180)*1.5 + distance_ahead_180*1.5 + 0.195;
			}
			else if(fabs(r_vt_original- distance_ahead_180)<0.74 - distance_ahead_180)
			{
				height_H=1.53;
				height_L=1.3;
			}
		    if(fabs(yaw_now*180/PI-iRobot_theta*180/PI)<angle_180)
		    {
			    height_H=1.5;
			    height_L=1.5;
		     }
			if(pos_now[2]>height_H+0.031)
			{
					comd_height=height_H+0.031;
			}
			else if(pos_now[2]>height_L+0.031)
			{
					comd_height=pos_now[2];
			}
			else
			{
					comd_height=height_L+0.031;
			}
			if(ob_danger_flag==true)
			{
				height_H=comd_height_if_obstacle;
				height_L=comd_height_if_obstacle-0.1;
			}
		}
///////////////////////////////////////////

///////////////////////////////////////////
		//高度自保措施 以后注意修改！ToDo
		if(comd_height>height_limited)
		{
			comd_height= height_limited;
		}
//////////////////////////////////////////////////////////////////////////////////////////////判断障碍物是否进入危险区域，决定是否可以land
		if(obstacel_amount_total>0)
		{
			for(int i=0;i<obstacel_amount_total;i++)
			{
				distance_obstacle=sqrt(interaction_obstacle_x[i]*interaction_obstacle_x[i]+interaction_obstacle_y[i]*interaction_obstacle_y[i])-obstacle_danger_distance;
				if(distance_obstacle<forbid_distance_land-obstacle_danger_distance)
				{
					no_land=true;
					break;
				}
				if(i==obstacel_amount_total-1)
				{
					no_land=false;
				}
			}
		}

		else
		{
			no_land=false;
		}
		///////////////////////////////////////////选择控制模式：1改进平行接近法 2PID控制 3垂直降落///////////////////////进入land
		PID_control_of_xy=true;
		if(model_choose==1)
		{
			if(r_vt<land_condition_r_45 && fabs(error[2])<land_condition_erryaw_45 && (jrh_drone->ultrasonic.ranges[0])<land_condition_height_45 && no_land==false && (jrh_drone->ultrasonic.ranges[0])>0 )
			{//静态测试0.03好使
				land=true;
				PID_control_of_xy=false;
				if(first_land)
				{
					remember_theta=iRobot_theta;
					remember_time=time_now;
					first_land=false;
				}
			}
		}
		else if(model_choose==2)
		{
			if(r_vt<land_condition_r_180 && (jrh_drone->ultrasonic.ranges[0])<land_condition_height_180 && fabs(error[2])<land_condition_erryaw_180 && no_land==false && (jrh_drone->ultrasonic.ranges[0])>0 )
			{
				land=true;
				PID_control_of_xy=false;
				if (first_land_180)
				{
					remember_theta = iRobot_theta;
					remember_time = time_now;
					first_land_180 = false;
				}
			}

		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		if(PID_control_of_xy)
		{
			if(model_choose==1)
			{
				error[0]=iRobot_x;
				error[1]=iRobot_y;
				error[2]=(iRobot_theta*180)/PI-yaw_now*180/PI;
				if(fabs(iRobot_theta*180/PI-572.957795)<0.2)//给出方向错误，一直为572.957795    一直跟着走
				{
					error[2] = (atan2(iRobot_original_y, iRobot_original_x) * 180) / PI - yaw_now * 180 / PI;
				}
////////////////////////////////////////////////////////////////////////////////////
				if(obstacel_amount_total==1)
				{
					distance_obstacle=sqrt(interaction_obstacle_x[0]*interaction_obstacle_x[0]+interaction_obstacle_y[0]*interaction_obstacle_y[0])-obstacle_danger_distance;
					if(distance_obstacle>0)
					{
					obstacle_theta=(atan2(interaction_obstacle_y[0],interaction_obstacle_x[0])*180)/PI;
printf("obstacle_theta  %f\n",obstacle_theta);
fprintf(model_txt,"obstacle_theta  %f\n",obstacle_theta);
					guidance_theta=yaw_now*180/PI+30;
					if(guidance_theta>180)
					{
						guidance_theta=guidance_theta-360;
					}
printf("guidance  1   theta  %f  \n",guidance_theta);
fprintf(model_txt,"guidance  1   theta  %f  \n",guidance_theta);
					min_roll_theta=fabs(guidance_theta-obstacle_theta);
					min_roll_real_theta=-guidance_theta+obstacle_theta;
					if(min_roll_theta>180)
					{
						min_roll_theta=fabs(min_roll_theta-360);
						min_roll_real_theta=min_roll_theta;
					}
					else if(min_roll_theta<-180)
					{
						min_roll_theta=fabs(min_roll_theta+360);
						min_roll_real_theta=-min_roll_theta;
					}
					min_roll_num=1;
printf("min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
fprintf(model_txt,"min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
////////////////////////////////////////////////////////////////////////////////////
					for(int i=2;i<6;i++)
					{
						guidance_theta=yaw_now*180/PI-30-60*(i-1);
						if(guidance_theta<-180)
						{
							guidance_theta=guidance_theta+360;
						}

						roll_theta=fabs(guidance_theta-obstacle_theta);
						roll_real_theta=-guidance_theta+obstacle_theta;
						if(roll_theta>180)
						{
							roll_theta=fabs(roll_theta-360);
							roll_real_theta=roll_theta;
						}
						else if(roll_theta<-180)
						{
							roll_theta=fabs(roll_theta+360);
							roll_real_theta=-roll_theta;
						}
						if(roll_theta<min_roll_theta)
						{
							min_roll_theta=roll_theta;
							min_roll_real_theta=roll_real_theta;
							min_roll_num=i;
						}
printf("guidance  %d   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",i,guidance_theta,roll_theta,roll_real_theta);
printf("min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
fprintf(model_txt,"guidance  %d   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",i,guidance_theta,roll_theta,roll_real_theta);
fprintf(model_txt,"min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
					}
////////////////////////////////////////////////////////////////////////////////////
					guidance_theta=yaw_now*180/PI-30;
					if(guidance_theta>180)
					{
						guidance_theta=guidance_theta-360;
					}

					roll_theta=fabs(guidance_theta-obstacle_theta);
					roll_real_theta=-guidance_theta+obstacle_theta;
					if(roll_theta>180)
					{
						roll_theta=fabs(roll_theta-360);
						roll_real_theta=roll_theta;
					}
					else if(roll_theta<-180)
					{
						roll_theta=fabs(roll_theta+360);
						roll_real_theta=-roll_theta;
					}

					if(roll_theta<min_roll_theta)
					{
						min_roll_theta=roll_theta;
						min_roll_real_theta=roll_real_theta;
						min_roll_num=6;
					}
printf("guidance  6   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",guidance_theta,roll_theta,roll_real_theta);
printf("min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
fprintf(model_txt,"guidance  6   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",guidance_theta,roll_theta,roll_real_theta);
fprintf(model_txt,"min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);

					error[2]=min_roll_real_theta;
////////////////////////////////////////////////////////////////////////////////////
				}
				}
////////////////////////////////////////////////////////////////////////////////////

				comd_yaw=error[2]+ yaw_now * 180 / PI;
				error[3]=comd_height-jrh_drone->ultrasonic.ranges[0];
			}
			else if(model_choose==2)
			{
				error[0]=iRobot_x;
				error[1]=iRobot_y;
/////////////////////////////////////////////////////////////////////////////////////
				if(fabs(iRobot_theta*180/PI-572.957795)<0.2)//给出方向错误，一直为572.957795    一直跟着走
				{
					error[2] = (atan2(iRobot_original_y, iRobot_original_x) * 180) / PI - yaw_now * 180 / PI;
				}
				else//给出方向正确，使用iRobot_theta
				{
					error[2]=(iRobot_theta*180)/PI-yaw_now*180/PI;

					if(iRobot_theta>0)//保证所有角度为顺时针指向目标
					{
						error[2]=(iRobot_theta*180)/PI-180-yaw_now*180/PI;
					}
					else
					{
						error[2]=(iRobot_theta*180)/PI+180-yaw_now*180/PI;
					}
				}



////////////////////////////////////////////////////////////////////////////////////
				if(obstacel_amount_total==1)
				{
					distance_obstacle=sqrt(interaction_obstacle_x[0]*interaction_obstacle_x[0]+interaction_obstacle_y[0]*interaction_obstacle_y[0])-obstacle_danger_distance-0.15;
					if(distance_obstacle>0)
					{
					obstacle_theta=(atan2(interaction_obstacle_y[0],interaction_obstacle_x[0])*180)/PI;
printf("obstacle_theta  %f\n",obstacle_theta);
fprintf(model_txt,"obstacle_theta  %f\n",obstacle_theta);
					guidance_theta=yaw_now*180/PI+30;
					if(guidance_theta>180)
					{
						guidance_theta=guidance_theta-360;
					}
printf("guidance  1   theta  %f  \n",guidance_theta);
fprintf(model_txt,"guidance  1   theta  %f  \n",guidance_theta);
					min_roll_theta=fabs(guidance_theta-obstacle_theta);
					min_roll_real_theta=-guidance_theta+obstacle_theta;
					if(min_roll_theta>180)
					{
						min_roll_theta=fabs(min_roll_theta-360);
						min_roll_real_theta=min_roll_theta;
					}
					else if(min_roll_theta<-180)
					{
						min_roll_theta=fabs(min_roll_theta+360);
						min_roll_real_theta=-min_roll_theta;
					}
					min_roll_num=1;
printf("min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
fprintf(model_txt,"min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
////////////////////////////////////////////////////////////////////////////////////
					for(int i=2;i<6;i++)
					{
						guidance_theta=yaw_now*180/PI-30-60*(i-1);
						if(guidance_theta<-180)
						{
							guidance_theta=guidance_theta+360;
						}

						roll_theta=fabs(guidance_theta-obstacle_theta);
						roll_real_theta=-guidance_theta+obstacle_theta;
						if(roll_theta>180)
						{
							roll_theta=fabs(roll_theta-360);
							roll_real_theta=roll_theta;
						}
						else if(roll_theta<-180)
						{
							roll_theta=fabs(roll_theta+360);
							roll_real_theta=-roll_theta;
						}
						if(roll_theta<min_roll_theta)
						{
							min_roll_theta=roll_theta;
							min_roll_real_theta=roll_real_theta;
							min_roll_num=i;
						}
printf("guidance  %d   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",i,guidance_theta,roll_theta,roll_real_theta);
printf("min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
fprintf(model_txt,"guidance  %d   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",i,guidance_theta,roll_theta,roll_real_theta);
fprintf(model_txt,"min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
					}
////////////////////////////////////////////////////////////////////////////////////
					guidance_theta=yaw_now*180/PI-30;
					if(guidance_theta>180)
					{
						guidance_theta=guidance_theta-360;
					}

					roll_theta=fabs(guidance_theta-obstacle_theta);
					roll_real_theta=-guidance_theta+obstacle_theta;
					if(roll_theta>180)
					{
						roll_theta=fabs(roll_theta-360);
						roll_real_theta=roll_theta;
					}
					else if(roll_theta<-180)
					{
						roll_theta=fabs(roll_theta+360);
						roll_real_theta=-roll_theta;
					}

					if(roll_theta<min_roll_theta)
					{
						min_roll_theta=roll_theta;
						min_roll_real_theta=roll_real_theta;
						min_roll_num=6;
					}
printf("guidance  6   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",guidance_theta,roll_theta,roll_real_theta);
printf("min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
fprintf(model_txt,"guidance  6   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",guidance_theta,roll_theta,roll_real_theta);
fprintf(model_txt,"min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);

					error[2]=min_roll_real_theta;
////////////////////////////////////////////////////////////////////////////////////
				}
				}
////////////////////////////////////////////////////////////////////////////////////
								error[3]=comd_height-jrh_drone->ultrasonic.ranges[0];
			}

			if(obstacel_amount_total>0)
			{
				for(int i=0;i<obstacle_amount_zy;i++)
				{
					distance_obstacle=sqrt(interaction_obstacle_x[i]*interaction_obstacle_x[i]+interaction_obstacle_y[i]*interaction_obstacle_y[i])-obstacle_danger_distance;
					if(distance_obstacle>3-obstacle_danger_distance)
					{
						distance_obstacle=3-obstacle_danger_distance;
					}
					else if(distance_obstacle<0)
					{
						distance_obstacle=0;
					}
					error_obstacle[i]=(0.33/(obstacle_danger_distance-3))*fabs(distance_obstacle)+0.33;
				}
				for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)
				{
					distance_obstacle=sqrt(interaction_obstacle_x[i]*interaction_obstacle_x[i]+interaction_obstacle_y[i]*interaction_obstacle_y[i])-obstacle_danger_distance;
					if(distance_obstacle>3-obstacle_danger_distance)
					{
						distance_obstacle=3-obstacle_danger_distance;
					}
					else if(distance_obstacle<0)
					{
						distance_obstacle=0;
					}
					error_obstacle[i]=(0.33/(obstacle_danger_distance-3))*fabs(distance_obstacle)+0.33;
				}
			}
		}
		if(land)
		{
			if(model_choose==1)
			{
				if(time_now-remember_time<parallel_land_time_45_first)
				{
					comd_height=0.24;
				}
				else
				{
					comd_height=0.18;
				}
				error[0]=0;
				error[1]=0;
				error[2]=0;
				error[3]=comd_height-jrh_drone->ultrasonic.ranges[0];

				if (tic() - remember_time>parallel_land_time_45_second)
				{
					land = false;
					take_off = true;
					model_choose = 88;
					printf("############################################################\n");
					printf("remember_time %f\n", remember_time);
					fprintf(model_txt, "############################################################\n");
					fprintf(model_txt, "remember_time %f\n", remember_time);
				}

			}
			else if(model_choose==2)
			{
				if (tic() - remember_time < parallel_land_time_180_first)
				{
					comd_height = 1.0;
				}
				else
				{
					comd_height = 0.2;
				}
				error[0] = 0;
				error[1] = 0;
				error[2]=0;
				error[3]=comd_height-jrh_drone->ultrasonic.ranges[0];
				printf("############################################################\n");
				printf("remember_time %f\n", remember_time);
				fprintf(model_txt, "############################################################\n");
				fprintf(model_txt, "remember_time %f\n", remember_time);
				fprintf(model_txt,"[180] comd_height %f \n",comd_height);
				//此处添加如果高度到达制定高度、加速度和较小，开始检测加速度是否有突变，如果有则land=false take_off=true model_choose==88
				//另外到达一定时间后take_off
				if (fabs(comd_height - 0.2)<0.01)
				{
					if (fabs(error[3])<0.05)
					{
						if (first_land == true)
						{
							first_land_time = tic();
							first_land = false;
						}
						if(tic()-first_land_time>0.5)
						{
							acc_total = jrh_drone->acceleration.ax*jrh_drone->acceleration.ax + jrh_drone->acceleration.ay*jrh_drone->acceleration.ay;
							if (acc_total>acc_land_to_take_off)
							{
								land = false;
								take_off = true;
								model_choose = 88;
								fprintf(model_txt, "碰撞起飞 \n");
							}
						}
						if (tic() - first_land_time>parallel_land_time_180_second)
						{
							land = false;
							take_off = true;
							model_choose = 88;
							fprintf(model_txt, "到点啦起飞 \n");
						}
						fprintf(model_txt, "first_land_time %f\n", first_land_time);
					}
				}

			}
		}	
	
///////////////////////////////////////////用来调偏航角的
		if(error[2]>180)
		{
			error[2]=(error[2]-2*180);
		}
		else if(error[2]<-180)
		{
			error[2]=(error[2]+2*180);
		}
		//printf("yaw_now %f\n",yaw_now*180/PI);
		printf("yaw_err  %f\n",error[2]);
		fprintf(model_txt,"yaw_err  %f\n",error[2]);
///////////////////////////////////////////	
	////////////////////////////////////////////计算kp项指令大小
		comd_p[0]=error[0]*kp[0];
		comd_p[1]=error[1]*kp[1];
		comd_p[2]=error[2]*kp[2];
		comd_p[3]=error[3]*kp[3];
		if(obstacel_amount_total>0)
		{
			for(int i=0;i<obstacle_amount_zy;i++)
			{
					comd_obstacle_p[i]=error_obstacle[i]*kp_obstacle;
			}
			for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)
			{
					comd_obstacle_p[i]=error_obstacle[i]*kp_obstacle;
			}
		}
	////////////////////////////////////////////计算ki项指令大小
		if(dt==0.0f)
		{
			comd_i[0]=0.0f;
			comd_i[1]=0.0f;
			comd_i[2]=0.0f;
			comd_i[3]=0.0f;
			if(obstacel_amount_total>0)
			{
				for(int i=0;i<obstacle_amount_zy;i++)
				{
						comd_obstacle_i[i]=0;
				}
				for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)
				{
						comd_obstacle_i[i]=0;
				}
			}

		}
		else
		{
			comd_i[0]=(comd_i[0]+error[0]*dt)*ki[0];
			comd_i[1]=(comd_i[1]+error[1]*dt)*ki[1];
			comd_i[2]=(comd_i[2]+error[2]*dt)*ki[2];
			comd_i[3]=(comd_i[3]+error[3]*dt)*ki[3];
			if(obstacel_amount_total>0)
			{
				for(int i=0;i<obstacle_amount_zy;i++)
				{
						comd_obstacle_i[i]=(comd_obstacle_i[i]+error_obstacle[i]*dt)*ki_obstacle;
				}
				for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)
				{
					comd_obstacle_i[i]=(comd_obstacle_i[i]+error_obstacle[i]*dt)*ki_obstacle;
				}
			}
		}
	////////////////////////////////////////////计算kd项指令大小
		if(dt==0.0f)
		{
			comd_d[0]=0.0f;
			comd_d[1]=0.0f;
			comd_d[2]=0.0f;
			comd_d[3]=0.0f;
			if(obstacel_amount_total>0)
			{
				for(int i=0;i<obstacle_amount_zy;i++)
				{
						comd_obstacle_d[i]=0;
				}
				for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)
				{
						comd_obstacle_d[i]=0;
				}
			}
		}
		else
		{
			comd_d[0]=kd[0]*(error[0]-last_error[0])/dt;
			comd_d[1]=kd[1]*(error[1]-last_error[1])/dt;
			comd_d[2]=kd[2]*(error[2]-last_error[2])/dt;
			comd_d[3]=kd[3]*(error[3]-last_error[3])/dt;
			if(obstacel_amount_total>0)
			{
				for(int i=0;i<obstacle_amount_zy;i++)
				{
						comd_obstacle_d[i]=kd_obstacle*(error_obstacle[i]-last_error_obstacle[i])/dt;
				}
				for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)
				{
						comd_obstacle_d[i]=kd_obstacle*(error_obstacle[i]-last_error_obstacle[i])/dt;
				}
			}
		}
	////////////////////////////////////////////整合指令的大小

		if(PID_control_of_xy==1&&land==0)
		{
			if(model_choose==1)
			{
				printf("[45]PID Start!!\n");
				fprintf(model_txt,"[45]PID Start!!\n");
				if(jrh_drone->ultrasonic.ranges[0]<1.5)
				{
					if(iRobot_y>-0.05)//防止冲过头
					{
						comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0]+0.24*cos(iRobot_theta);
						comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1]+0.24*sin(iRobot_theta);
					}
					else
					{
						comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0]+0.12*cos(iRobot_theta);
						comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1]+0.12*sin(iRobot_theta);
				     }
				}
				else
				{
					comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0];//+0.24*cos(iRobot_theta);
					comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1];//+0.24*sin(iRobot_theta);
				}
				if(fabs(iRobot_theta*180/PI-572.957795)<0.2)
				{
					comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0];///////////////////////////////
					comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1];///////////////////////////////
				}
				comd_yaw_rate=comd_p[2]+comd_i[2]+comd_d[2];
				comd_height_v=comd_p[3]+comd_i[3]+comd_d[3];
				printf("goal_comd x %f y%f\n",comd_angle_x,comd_angle_y);
				fprintf(model_txt,"goal_comd x %f y%f\n",comd_angle_x,comd_angle_y);
//printf("obstacle num %d\n",obstacel_amount_total);
				if(obstacel_amount_total>0)
				{
					danger_obstacle_count=0;
					ob_danger_flag=false;
					if(obstacel_amount_total==1)
					{
						distance_obstacle=sqrt(interaction_obstacle_x[0]*interaction_obstacle_x[0]+interaction_obstacle_y[0]*interaction_obstacle_y[0])-obstacle_danger_distance;
						if(distance_obstacle>0)
						{
						comd_angle_x=(comd_angle_x+((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						comd_angle_y=(comd_angle_y+((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						printf("########safe!#######obstacle _comd  x %f y %f\n",((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])),((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						fprintf(model_txt,"########safe!################obstacle _comd  x %f y %f\n",((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])),((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						ob_danger_flag=true;
						}
						else
						{
							comd_angle_x=(0.66*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
							comd_angle_y=(0.66*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
							ob_danger_flag=true;
							printf("##########Danger!!!!!!!!!!#######obstacle_comd x %f y%f\n",comd_angle_x,comd_angle_y);
							fprintf(model_txt,"##########Only One Danger! Take_off!!!!!!!!!##############\n");
						}
					}
					else
					{
						/*
						danger_obstacle_take_off=true;
						model_choose=88;
						take_off=true;
						fprintf(model_txt,"##########Above Two Danger! Take_off!!!!!!!!!##############\n");
					*/
						for(int i=0;i<obstacel_amount_total;i++)
						{
							distance_obstacle=sqrt(interaction_obstacle_x[i]*interaction_obstacle_x[i]+interaction_obstacle_y[i]*interaction_obstacle_y[i])-obstacle_danger_distance;
							if(distance_obstacle>0)
							{
							comd_angle_x=(comd_angle_x+((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							comd_angle_y=(comd_angle_y+((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							printf("########safe!#######obstacle _comd  x %f y %f\n",((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])),((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							fprintf(model_txt,"########safe!################obstacle _comd  x %f y %f\n",((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])),((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							ob_danger_flag=true;
							}
							else
							{
								/*
								comd_angle_x=(0.66*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
								comd_angle_y=(0.66*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
								ob_danger_flag=true;
								printf("##########Danger!!!!!!!!!!#######obstacle_comd x %f y%f\n",comd_angle_x,comd_angle_y);
								fprintf(model_txt,"##########Above Two Danger! Take_off!!!!!!!!!##############\n");
								//printf("-----------------------------------Danger!!!!!!!!!!!----------------------------------------\n");
								danger_obstacle_count=danger_obstacle_count+1;
								if(danger_obstacle_count>1)*/
								{
									danger_obstacle_take_off=true;
									model_choose=88;
									take_off=true;
									break;
								}
							}
						}
					}
				}
				else
				{
					ob_danger_flag=false;
				}

			fprintf(error_txt,"[45-PID] [time]  %f  [Tx]  %f  [Ty]  %f   [T_yaw] %f   [Tz]  %f   \n",time_now,error[0],error[1],error[2],error[3]);
			fprintf(error_block_txt,"%f %f %f %f %f   \n",time_now,error[0],error[1],error[2],error[3]);
			fprintf(comd_setpoint,"[45-PID] [time]  %f  [vx]  %f  [ x_t]  %f   [vy] %f   [y_t]  %f  [r_vt] %f   [i_theta] %f [c_h] %f  [vz] %f [yaw_rate] %f [height_now] %f  [yaw_now] %f [comd_yaw] %f [vx_now] %f [vy_now] %f [vz_now] %f  [r_vt_original] %f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,jrh_drone->ultrasonic.ranges[0],yaw_now*180/PI,comd_yaw,jrh_drone->velocity.vx,jrh_drone->velocity.vy,jrh_drone->velocity.vz,r_vt_original);
			fprintf(comd_setpoint_block,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f%f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,jrh_drone->ultrasonic.ranges[0],yaw_now*180/PI,comd_yaw,jrh_drone->velocity.vx,jrh_drone->velocity.vy,jrh_drone->velocity.vz,r_vt_original);
			}
			if(model_choose==2)
			{
				printf("[180]PID Start!!\n");
				fprintf(model_txt,"[180]PID Start!!\n");
				if(fabs(iRobot_theta*180/PI-572.957795)<0.2)
				{
					comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0];///////////////////////////////
					comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1];///////////////////////////////
				}
				else
				{
					comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0]+parallel_land_velcity_180*cos(iRobot_theta);///////////////////////////////
					comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1]+parallel_land_velcity_180*sin(iRobot_theta);///////////////////////////////
				}
				comd_yaw_rate=comd_p[2]+comd_i[2]+comd_d[2];
				comd_height_v=comd_p[3]+comd_i[3]+comd_d[3];
				printf("goal_comd x %f y%f\n",comd_angle_x,comd_angle_y);
				fprintf(model_txt,"goal_comd x %f y%f\n",comd_angle_x,comd_angle_y);
				//printf("obstacle num %d\n",obstacel_amount_total);

				if(obstacel_amount_total>0)
				{
					danger_obstacle_count=0;
					ob_danger_flag=false;
					if(obstacel_amount_total==1)
					{
						distance_obstacle=sqrt(interaction_obstacle_x[0]*interaction_obstacle_x[0]+interaction_obstacle_y[0]*interaction_obstacle_y[0])-obstacle_danger_distance;
						if(distance_obstacle>0)
						{
						comd_angle_x=(comd_angle_x+((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						comd_angle_y=(comd_angle_y+((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						printf("########safe!#######obstacle _comd  x %f y %f\n",((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])),((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						fprintf(model_txt,"########safe!################obstacle _comd  x %f y %f\n",((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])),((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						ob_danger_flag=true;
						}
						else
						{
							comd_angle_x=(0.66*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
							comd_angle_y=(0.66*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
							ob_danger_flag=true;
							printf("##########Danger!!!!!!!!!!#######obstacle_comd x %f y%f\n",comd_angle_x,comd_angle_y);
							fprintf(model_txt,"##########Only One Danger! Take_off!!!!!!!!!##############\n");
						}
					}
					else
					{
						/*
						danger_obstacle_take_off=true;
						model_choose=88;
						take_off=true;
						fprintf(model_txt,"##########Above Two Danger! Take_off!!!!!!!!!##############\n");
					*/
						for(int i=0;i<obstacel_amount_total;i++)
						{
							distance_obstacle=sqrt(interaction_obstacle_x[i]*interaction_obstacle_x[i]+interaction_obstacle_y[i]*interaction_obstacle_y[i])-obstacle_danger_distance;
							if(distance_obstacle>0)
							{
							comd_angle_x=(comd_angle_x+((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							comd_angle_y=(comd_angle_y+((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							printf("########safe!#######obstacle _comd  x %f y %f\n",((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])),((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							fprintf(model_txt,"########safe!################obstacle _comd  x %f y %f\n",((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])),((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							ob_danger_flag=true;
							}
							else
							{
								/*
								comd_angle_x=(0.66*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
								comd_angle_y=(0.66*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
								ob_danger_flag=true;
								printf("##########Danger!!!!!!!!!!#######obstacle_comd x %f y%f\n",comd_angle_x,comd_angle_y);
								fprintf(model_txt,"##########Danger!!!!!!!!!!##############obstacle_comd x %f y%f\n",comd_angle_x,comd_angle_y);
								//printf("-----------------------------------Danger!!!!!!!!!!!----------------------------------------\n");
								danger_obstacle_count=danger_obstacle_count+1;
								if(danger_obstacle_count>1)*/
								{
									danger_obstacle_take_off=true;
									model_choose=88;
									take_off=true;
									break;
								}
							}
						}
					}

				}
				else
				{
					ob_danger_flag=false;
				}

			fprintf(error_txt,"[180-PID] [time]  %f  [Tx]  %f  [Ty]  %f   [T_yaw] %f   Tz]  %f   \n",time_now,error[0],error[1],error[2],error[3]);
			fprintf(error_block_txt,"%f  %f  %f  %f  %f  \n",time_now,error[0],error[1],error[2],error[3]);
			fprintf(comd_setpoint,"[180-PID] [time]  %f  [vx]  %f  [ x_t]  %f   [vy] %f   [y_t]  %f  [r_vt] %f   [i_theta] %f [c_h] %f  [vz] %f [yaw_rate] %f  [r_vt_original] %f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,r_vt_original);
			fprintf(comd_setpoint_block,"  %f    %f    %f   %f   %f     %f   %f    %f   %f  %f  %f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,r_vt_original);
			}
			if(model_choose==3)/////////////////////////////////////////////////////////////
			{
				printf("######no goal !!\n");
				fprintf(model_txt,"######no goal !!\n");
			}
		}
		if(land)
		{
			if(model_choose==1)
			{
				printf("[45]Land Start!!\n");
				fprintf(model_txt,"[45]Land Start!!\n");
			comd_angle_x=0+velcity_land_45*cos(remember_theta);
			comd_angle_y=0+velcity_land_45*sin(remember_theta);
			comd_yaw_rate=0;
			comd_height_v=comd_p[3]+comd_i[3]+comd_d[3];
			fprintf(error_txt,"[45-land] [time]  %f  [Tx]  %f  [Ty]  %f   [T_yaw] %f   [Tz]  %f  \n",time_now,error[0],error[1],error[2],error[3]);
			fprintf(error_block_txt,"%f  %f  %f  %f  %f  \n",time_now,error[0],error[1],error[2],error[3]);
			fprintf(comd_setpoint,"[45-PID] [time]  %f  [vx]  %f  [ x_t]  %f   [vy] %f   [y_t]  %f  [r_vt] %f   [i_theta] %f [c_h] %f  [vz] %f [yaw_rate] %f [height_now] %f  [yaw_now] %f [comd_yaw] %f [vx_now] %f [vy_now] %f [vz_now] %f [r_vt_original] %f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,jrh_drone->ultrasonic.ranges[0],yaw_now*180/PI,comd_yaw,jrh_drone->velocity.vx,jrh_drone->velocity.vy,jrh_drone->velocity.vz,r_vt_original);
			fprintf(comd_setpoint_block,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,jrh_drone->ultrasonic.ranges[0],yaw_now*180/PI,comd_yaw,jrh_drone->velocity.vx,jrh_drone->velocity.vy,jrh_drone->velocity.vz,r_vt_original);
			}
			if(model_choose==2)
			{
				printf("[180]Land Start!!\n");
				fprintf(model_txt,"[180]Land Start!!\n");
				if(tic()-remember_time<parallel_land_time_180_first)////////////////////////////向后延迟降落时间
				{
					comd_angle_x=velcity_land_180*cos(remember_theta);
					comd_angle_y=velcity_land_180*sin(remember_theta);
				}
				else
				{
					comd_angle_x=0;
					comd_angle_y=0;
				}
				comd_yaw_rate=0;
				comd_height_v=comd_p[3]+comd_i[3]+comd_d[3];

				fprintf(error_txt,"[180-land] [time]  %f  [Tx]  %f  [Ty]  %f   [T_yaw] %f   [Tz]  %f  \n",time_now,error[0],error[1],error[2],error[3]);
				fprintf(error_block_txt,"%f  %f  %f   %f   %f  \n",time_now,error[0],error[1],error[2],error[3]);
				fprintf(comd_setpoint,"[180-PID] [time]  %f  [vx]  %f  [ x_t]  %f   [vy] %f   [y_t]  %f  [r_vt] %f   [i_theta] %f [c_h] %f  [vz] %f [yaw_rate] %f [height_now] %f  [yaw_now] %f [comd_yaw] %f [r_vt_original] %f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,jrh_drone->ultrasonic.ranges[0],yaw_now*180/PI,comd_yaw,r_vt_original);
				fprintf(comd_setpoint_block,"%f    %f    %f   %f   %f     %f   %f    %f   %f  %f %f %f %f %f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,jrh_drone->ultrasonic.ranges[0],yaw_now*180/PI,comd_yaw,r_vt_original);
			}
		}

		if(take_off==false)
		{
	////////////////////////////////////////////限幅
		if(model_choose==1 && ob_danger_flag==false)
		{
			if(jrh_drone->ultrasonic.ranges[0]<0.8)
			{
				if(r_vt>1.0)
				{
						if(comd_angle_x>0.4f)//静态限幅0.4 好使  动态时0.4也好使，略显慢
						{
							comd_angle_x=0.4f;
						}
						else if(comd_angle_x<-0.4f)
						{
							comd_angle_x=-0.4f;
						}
						if(comd_angle_y>0.4f)
						{
							comd_angle_y=0.4f;
						}
						else if(comd_angle_y<-0.4f)
						{
							comd_angle_y=-0.4f;
						}
				}
				else if(r_vt>0.1 && r_vt<1.0)//静态限幅0.2 好使  动态时0.2也好使，略显慢//1.0
				{
						if(comd_angle_x>0.28f)//0.4
						{
							comd_angle_x=0.28f;
						}
						else if(comd_angle_x<-0.28f)
						{
							comd_angle_x=-0.28f;
						}
						if(comd_angle_y>0.28f)
						{
							comd_angle_y=0.28f;
						}
						else if(comd_angle_y<-0.28f)
						{
							comd_angle_y=-0.28f;
						}
				}
				else//静态限幅0.4 好使  动态时0.4也好使，略显慢
				{
							if(comd_angle_x>0.6f)
							{
								comd_angle_x=0.6f;
							}
							else if(comd_angle_x<-0.6f)
							{
								comd_angle_x=-0.6f;
							}
							if(comd_angle_y>0.6f)
							{
								comd_angle_y=0.6f;
							}
							else if(comd_angle_y<-0.6f)
							{
								comd_angle_y=-0.6f;
							}
				}
			}
			else
			{
							if(comd_angle_x>0.6f)
							{
								comd_angle_x=0.6f;
							}
							else if(comd_angle_x<-0.6f)
							{
								comd_angle_x=-0.6f;
							}
							if(comd_angle_y>0.6f)
							{
								comd_angle_y=0.6f;
							}
							else if(comd_angle_y<-0.6f)
							{
								comd_angle_y=-0.6f;
							}
			}
		}

		if(model_choose==2 && ob_danger_flag==false)
		{
			if(jrh_drone->ultrasonic.ranges[0]>1.8)//1.8
			{
				if(comd_angle_x>0.3f)//0.3
				{
					comd_angle_x=0.3f;
				}
				else if(comd_angle_x<-0.3f)
				{
					comd_angle_x=-0.3f;
				}
				if(comd_angle_y>0.3f)
				{
					comd_angle_y=0.3f;
				}
				else if(comd_angle_y<-0.3f)
				{
					comd_angle_y=-0.3f;
				}
			}
			else
			{
				if(r_vt<0.5)
				{
					if(comd_angle_x>0.5f)//1.0
					{
						comd_angle_x=0.5;
					}
					else if(comd_angle_x<-0.5)
					{
						comd_angle_x=-0.5;
					}
					if(comd_angle_y>0.5)
					{
						comd_angle_y=0.5;
					}
					else if(comd_angle_y<-0.5)
					{
						comd_angle_y=-0.5;
					}
				}
				else
				{
					if(comd_angle_x>0.5f)//1.0
					{
						comd_angle_x=0.5f;
					}
					else if(comd_angle_x<-0.5f)
					{
						comd_angle_x=-0.5f;
					}
					if(comd_angle_y>0.5f)
					{
						comd_angle_y=0.5f;
					}
					else if(comd_angle_y<-0.5f)
					{
						comd_angle_y=-0.5f;
					}
				}
			}
		}
//////////////////////////////////////////////////////////////////////////////////////////
		if(model_choose==1)
		{
			if(r_vt>0.5 && r_vt<1.0)
			{
				if(comd_height_v>0.8f)
				{
					comd_height_v=0.8f;
				}
				else if(comd_height_v<-0.8f)
				{
					comd_height_v=-0.8f;
				}
			}
			else if(r_vt>0.1 && r_vt<0.5)
			{
				if(comd_height_v>0.8f)
				{
					comd_height_v=0.8f;
				}
				else if(comd_height_v<-0.8f)
				{
					comd_height_v=-0.8f;
				}
			}
			else
			{
				if(comd_height_v>1.0f)
				{
					comd_height_v=1.0f;
				}
				else if(comd_height_v<-1.0f)
				{
					comd_height_v=-1.0f;
				}
			}

			if(jrh_drone->ultrasonic.ranges[0]>1.4)
			{
				if(comd_height_v>0.5f)
				{
					comd_height_v=0.5f;
				}
				else if(comd_height_v<-0.5f)
				{
					comd_height_v=-0.5f;
				}
			}

		}

		if(model_choose==2)
		{
			if(comd_height_v>0.8f)
			{
				comd_height_v=0.8f;
			}
			else if(comd_height_v<-0.8f)
			{
				comd_height_v=-0.8f;
			}
		}
//////////////////////////////////////////////////////////////////////////////////////////
		if(jrh_drone->ultrasonic.ranges[0]>1.8 && jrh_drone->ultrasonic.ranges[0]<2.0)//1.8
		{
			if(comd_yaw_rate>20.0f)
			{
				comd_yaw_rate=20.0f;
			}
			else if(comd_yaw_rate<-20.0f)
			{
				comd_yaw_rate=-20.0f;
			}
		}
		else
		{
			if(comd_yaw_rate>50.0f)
			{
				comd_yaw_rate=50.0f;
			}
			else if(comd_yaw_rate<-50.0f)
			{
				comd_yaw_rate=-50.0f;
			}
		}
		if(model_choose==2)
		{
			if(comd_yaw_rate>18.0f)
			{
				comd_yaw_rate=18.0f;
			}
			else if(comd_yaw_rate<-18.0f)
			{
				comd_yaw_rate=-18.0f;
			}
		}


	////////////////////////////////////////////
		fprintf(comd_setpoint_limited," %f    %f   %f   %f  \n",time_now,comd_angle_x,comd_angle_y,comd_height_v);

		last_error[0]=error[0];last_error[1]=error[1];last_error[2]=error[2];last_error[3]=error[3];
		if(obstacel_amount_total>0)
		{
			for(int i=0;i<obstacel_amount_total;i++)
			{
				last_error_obstacle[i]=error_obstacle[i];
			}
		}
		last_time=time_now;
		if(model_choose==1)
		{
			printf("final_goal_comd x %f y%f\n",comd_angle_x,comd_angle_y);
			fprintf(model_txt,"final_goal_comd x %f y%f\n",comd_angle_x,comd_angle_y);

			printf("rembember_theta%f\n",remember_theta);
			printf("[45]T %f  yawv   %f   vz %f  c_ height %f   \n",time_now,comd_yaw_rate,comd_height_v,comd_height);
			printf("[45]vx %f   x_t %f  vy %f  y_t %f r_vt %f   theta %f \n",comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI);///////////////////////////////////////////////////////////////
			printf("[45]height_now  %f  theta_tg %f  \n",pos_now[2],iRobot_theta);

			fprintf(model_txt,"rembember_theta%f\n",remember_theta);
			fprintf(model_txt,"[45]T %f  yawv   %f   vz %f  c_ height %f   \n",time_now,comd_yaw_rate,comd_height_v,comd_height);
			fprintf(model_txt,"[45]vx %f   x_t %f  vy %f  y_t %f r_vt %f   theta %f\n",comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI);
			fprintf(model_txt,"[45]height_now  %f  theta_tg %f  \n",pos_now[2],iRobot_theta);

		}
		else if(model_choose==2)
		{
			printf("final_goal_comd x %f y%f\n",comd_angle_x,comd_angle_y);
			fprintf(model_txt,"final_goal_comd x %f y%f\n",comd_angle_x,comd_angle_y);

			//printf("[180]T %f  yawv   %f   vz %f  c_ height %f   \n",time_now,comd_yaw_rate,comd_height_v,comd_height);
			printf("[180]vx %f--x_t %f  vy %f  y_t %f r_vt %f  i_theta %f\n",comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI);
			printf("[180]height_now  %f  r_vt_original  %f  v_z %f \n",pos_now[2],r_vt_original,comd_height_v);
			printf("[180]comd_height  %f   \n",comd_height);

			fprintf(model_txt,"[180]vx %f--x_t %f  vy %f  y_t %f r_vt %f  i_theta %f\n",comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI);
			fprintf(model_txt,"[180]height_now  %f  r_vt_original  %f  v_z %f \n",pos_now[2],r_vt_original,comd_height_v);
			fprintf(model_txt,"[180]comd_height  %f   \n",comd_height);
		}

		////////////////////////////////////////////将指令从赛场动坐标系到地理坐标系
		float comd_vx=comd_angle_x*cvmGet(R_init,0,0)+comd_angle_y*cvmGet(R_init,0,1)+comd_height_v*cvmGet(R_init,0,2);
		float comd_vy=comd_angle_x*cvmGet(R_init,1,0)+comd_angle_y*cvmGet(R_init,1,1)+comd_height_v*cvmGet(R_init,1,2);
		float comd_zv=comd_angle_x*cvmGet(R_init,2,0)+comd_angle_y*cvmGet(R_init,2,1)+comd_height_v*cvmGet(R_init,2,2);
	////////////////////////////////////////////发送指令地理坐标系中
		drone->velocity_control(0x49,comd_vx,comd_vy,comd_zv,comd_yaw_rate);
		}
	}
else if(model_choose==0)//跟随模式哦
{
	if(first_zhuanquanquan==true)
	{
		first_ensure_if_obstacle_time=tic();
		first_zhuanquanquan=false;
	}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////刚进入交互模式时采取原地转圈

while((time_now-first_ensure_if_obstacle_time)<1)
{
	ros::spinOnce();
	time_now=tic();
	reverse_yaw_now=yaw_now-2*PI;

	if(reverse_yaw_now<-PI)
	{
		reverse_yaw_now=reverse_yaw_now+2*PI;
	}
	atan_iRobot=atan2(iRobot_original_y,iRobot_original_x);
	zhuanquan=atan_iRobot-yaw_now;
	if(zhuanquan<-PI)
	{
		zhuanquan=zhuanquan+28*PI;
	}
	else if(zhuanquan>PI)
	{
		zhuanquan=zhuanquan-2*PI;
	}
	if(zhuanquan<0 )
	{
		comd_yaw_rate=-yaw_rate_zhuanquan;
	}
	else
	{
		comd_yaw_rate=yaw_rate_zhuanquan;
	}
	if(comd_yaw_rate<0)
	{
		comd_angle_x=velcity_zhuanquan*cos(iRobot_theta);
		comd_angle_y=velcity_zhuanquan*sin(iRobot_theta);
	}
	else
	{
		comd_angle_x=velcity_zhuanquan*cos(iRobot_theta);
		comd_angle_y=velcity_zhuanquan*sin(iRobot_theta);
	}
	comd_height_v=0;
	fprintf(model_txt,"first_ensure_if_obstacle_time  %f  time_now  %f\n",first_ensure_if_obstacle_time,time_now);
	fprintf(model_txt,"转圈圈 comd_yaw_rate %f  atan_iRobot  %f\n",comd_yaw_rate,atan_iRobot);

	////////////////////////////////////////////将指令从赛场动坐标系到地理坐标系
	float comd_vx=comd_angle_x*cvmGet(R_init,0,0)+comd_angle_y*cvmGet(R_init,0,1)+comd_height_v*cvmGet(R_init,0,2);
	float comd_vy=comd_angle_x*cvmGet(R_init,1,0)+comd_angle_y*cvmGet(R_init,1,1)+comd_height_v*cvmGet(R_init,1,2);
	float comd_zv=comd_angle_x*cvmGet(R_init,2,0)+comd_angle_y*cvmGet(R_init,2,1)+comd_height_v*cvmGet(R_init,2,2);
////////////////////////////////////////////发送指令地理坐标系中
	drone->velocity_control(0x49,comd_vx,comd_vy,comd_zv,comd_yaw_rate);
	rate.sleep();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if(first_follow==true)
		{
			last_time=0;
			time_now=tic();
			dt=0.0f;
			comd_i[0]=0;
			comd_i[1]=0;
			comd_i[2]=0;
			comd_i[3]=0;
		    comd_obstacle_i[8]={0.0f};
		    first_follow=false;
		}
		else
		{
			time_now=tic();
			dt=time_now-last_time;
		}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		iRobot_x=(jrh_drone->object_pos_dynamic.target_x[jrh_iRobot_i])*0.01;
		iRobot_y=(jrh_drone->object_pos_dynamic.target_y[jrh_iRobot_i])*0.01;
		iRobot_theta=jrh_drone->object_pos_dynamic.target_dir[jrh_iRobot_i];
		iRobot_num=jrh_drone->object_pos_dynamic.target_num;
		iRobot_original_x=iRobot_x;
		iRobot_original_y=iRobot_y;
//////////////////////////////////////////////////////////////////////////
	//TODO
		obstacle_amount_zy=jrh_drone->obstacle_pos_dynamic.num;//从张雨订阅一共看到几个机器人
		obstacle_amount_yqt=jrh_drone->obstacle_pos_dynamic_yqt.num;//从于庆涛订阅一共看到几个机器人
		obstacel_amount_total=obstacle_amount_zy+obstacle_amount_yqt;
		//得到所有障碍物信息，如果没有障碍物信息则返回-80信息
		if((obstacel_amount_total)>0)
		{
			for(int i=0;i<obstacle_amount_zy;i++)//从张雨那里拿到障碍物位置信息
			{
				interaction_obstacle_x[i]=jrh_drone->obstacle_pos_dynamic.obstacle_x[i];
				interaction_obstacle_y[i]=jrh_drone->obstacle_pos_dynamic.obstacle_y[i];
			}
			for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)////从于庆涛那里拿到障碍物位置信息
			{
				interaction_obstacle_x[i]=jrh_drone->obstacle_pos_dynamic_yqt.obstacle_x[i-obstacle_amount_zy];
				interaction_obstacle_y[i]=jrh_drone->obstacle_pos_dynamic_yqt.obstacle_y[i-obstacle_amount_zy];
				//printf(" obstacle_x %f  obstacle_y %f \n",interaction_obstacle_x[i],interaction_obstacle_y[i]);
			}
			for(int i=obstacel_amount_total;i<8;i++)
			{
				interaction_obstacle_x[i]=-80;
				interaction_obstacle_y[i]=-80;
			}
		}
		else
		{

			for(int i=0;i<8;i++)
			{
				interaction_obstacle_x[i]=-80;
				interaction_obstacle_y[i]=-80;
			}
		}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		iRobot_x=iRobot_x+0.0*cos(iRobot_theta)-cos(yaw_now)*fabs(-0.132*sin(-cvmGet(att_right_now,1,0))+0.10*cos(-cvmGet(att_right_now,1,0)));
		iRobot_y=iRobot_y+0.0*sin(iRobot_theta)-sin(yaw_now)*fabs(-0.132*sin(-cvmGet(att_right_now,1,0))+0.10*cos(-cvmGet(att_right_now,1,0)));

		if(fabs(iRobot_theta*180/PI-572.957795)<0.2)
		{
			iRobot_x=iRobot_x;
			iRobot_y=iRobot_y;
		}
		error[0]=iRobot_x;
		error[1]=iRobot_y;
		if(fabs(iRobot_theta*180/PI-572.957795)<0.2)//给出方向错误，一直为572.957795    一直跟着走
		{
			error[2] = (atan2(iRobot_original_y, iRobot_original_x) * 180) / PI - yaw_now * 180 / PI;
		}
		else
		{
			error[2]=(iRobot_theta*180)/PI-yaw_now*180/PI;
		}

////////////////////////////////////////////////////////////////////////////////////
				if(obstacel_amount_total==1)
				{
					distance_obstacle=sqrt(interaction_obstacle_x[0]*interaction_obstacle_x[0]+interaction_obstacle_y[0]*interaction_obstacle_y[0])-obstacle_danger_distance-0.15;
					if(distance_obstacle>0)
					{
					obstacle_theta=(atan2(interaction_obstacle_y[0],interaction_obstacle_x[0])*180)/PI;
printf("obstacle_theta  %f\n",obstacle_theta);
fprintf(model_txt,"obstacle_theta  %f\n",obstacle_theta);
					guidance_theta=yaw_now*180/PI+30;
					if(guidance_theta>180)
					{
						guidance_theta=guidance_theta-360;
					}
printf("guidance  1   theta  %f  \n",guidance_theta);
fprintf(model_txt,"guidance  1   theta  %f  \n",guidance_theta);
					min_roll_theta=fabs(guidance_theta-obstacle_theta);
					min_roll_real_theta=-guidance_theta+obstacle_theta;
					if(min_roll_theta>180)
					{
						min_roll_theta=fabs(min_roll_theta-360);
						min_roll_real_theta=min_roll_theta;
					}
					else if(min_roll_theta<-180)
					{
						min_roll_theta=fabs(min_roll_theta+360);
						min_roll_real_theta=-min_roll_theta;
					}
					min_roll_num=1;
printf("min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
fprintf(model_txt,"min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
////////////////////////////////////////////////////////////////////////////////////
					for(int i=2;i<6;i++)
					{
						guidance_theta=yaw_now*180/PI-30-60*(i-1);
						if(guidance_theta<-180)
						{
							guidance_theta=guidance_theta+360;
						}

						roll_theta=fabs(guidance_theta-obstacle_theta);
						roll_real_theta=-guidance_theta+obstacle_theta;
						if(roll_theta>180)
						{
							roll_theta=fabs(roll_theta-360);
							roll_real_theta=roll_theta;
						}
						else if(roll_theta<-180)
						{
							roll_theta=fabs(roll_theta+360);
							roll_real_theta=-roll_theta;
						}
						if(roll_theta<min_roll_theta)
						{
							min_roll_theta=roll_theta;
							min_roll_real_theta=roll_real_theta;
							min_roll_num=i;
						}
printf("guidance  %d   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",i,guidance_theta,roll_theta,roll_real_theta);
printf("min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
fprintf(model_txt,"guidance  %d   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",i,guidance_theta,roll_theta,roll_real_theta);
fprintf(model_txt,"min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
					}
////////////////////////////////////////////////////////////////////////////////////
					guidance_theta=yaw_now*180/PI-30;
					if(guidance_theta>180)
					{
						guidance_theta=guidance_theta-360;
					}

					roll_theta=fabs(guidance_theta-obstacle_theta);
					roll_real_theta=-guidance_theta+obstacle_theta;
					if(roll_theta>180)
					{
						roll_theta=fabs(roll_theta-360);
						roll_real_theta=roll_theta;
					}
					else if(roll_theta<-180)
					{
						roll_theta=fabs(roll_theta+360);
						roll_real_theta=-roll_theta;
					}

					if(roll_theta<min_roll_theta)
					{
						min_roll_theta=roll_theta;
						min_roll_real_theta=roll_real_theta;
						min_roll_num=6;
					}
printf("guidance  6   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",guidance_theta,roll_theta,roll_real_theta);
printf("min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);
fprintf(model_txt,"guidance  6   G_theta  %f  roll_theta  %f   roll_real_theta  %f\n",guidance_theta,roll_theta,roll_real_theta);
fprintf(model_txt,"min_roll_theta %f  min_roll_real  %f  \n",min_roll_theta,min_roll_real_theta);

					error[2]=min_roll_real_theta;
////////////////////////////////////////////////////////////////////////////////////
					}
				}
////////////////////////////////////////////////////////////////////////////////////

		if(obstacel_amount_total>0)
		{
			error[3]=comd_height_if_obstacle-jrh_drone->ultrasonic.ranges[0];
		}
		else
		{
			error[3]=comd_follow_height-jrh_drone->ultrasonic.ranges[0];
		}


				comd_p[0]=error[0]*kp[0];
				comd_p[1]=error[1]*kp[1];
				comd_p[2]=error[2]*kp[2];
				comd_p[3]=error[3]*kp[3];

				if(obstacel_amount_total>0)
				{
					for(int i=0;i<obstacle_amount_zy;i++)
					{
							comd_obstacle_p[i]=error_obstacle[i]*kp_obstacle;
					}
					for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)
					{
							comd_obstacle_p[i]=error_obstacle[i]*kp_obstacle;
					}
				}

			////////////////////////////////////////////计算ki项指令大小
				if(dt==0.0f)
				{
					comd_i[0]=0.0f;
					comd_i[1]=0.0f;
					comd_i[2]=0.0f;
					comd_i[3]=0.0f;
					if(obstacel_amount_total>0)
					{
						for(int i=0;i<obstacle_amount_zy;i++)
						{
								comd_obstacle_i[i]=0;
						}
						for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)
						{
								comd_obstacle_i[i]=0;
						}
					}

				}
				else
				{
					comd_i[0]=(comd_i[0]+error[0]*dt)*ki[0];
					comd_i[1]=(comd_i[1]+error[1]*dt)*ki[1];
					comd_i[2]=(comd_i[2]+error[2]*dt)*ki[2];
					comd_i[3]=(comd_i[3]+error[3]*dt)*ki[3];
					if(obstacel_amount_total>0)
					{
						for(int i=0;i<obstacle_amount_zy;i++)
						{
								comd_obstacle_i[i]=(comd_obstacle_i[i]+error_obstacle[i]*dt)*ki_obstacle;
						}
						for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)
						{
							comd_obstacle_i[i]=(comd_obstacle_i[i]+error_obstacle[i]*dt)*ki_obstacle;
						}
					}
				}
			////////////////////////////////////////////计算kd项指令大小
				if(dt==0.0f)
				{
					comd_d[0]=0.0f;
					comd_d[1]=0.0f;
					comd_d[2]=0.0f;
					comd_d[3]=0.0f;
					if(obstacel_amount_total>0)
					{
						for(int i=0;i<obstacle_amount_zy;i++)
						{
								comd_obstacle_d[i]=0;
						}
						for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)
						{
								comd_obstacle_d[i]=0;
						}
					}
				}
				else
				{
					comd_d[0]=kd[0]*(error[0]-last_error[0])/dt;
					comd_d[1]=kd[1]*(error[1]-last_error[1])/dt;
					comd_d[2]=kd[2]*(error[2]-last_error[2])/dt;
					comd_d[3]=kd[3]*(error[3]-last_error[3])/dt;
					if(obstacel_amount_total>0)
					{
						for(int i=0;i<obstacle_amount_zy;i++)
						{
								comd_obstacle_d[i]=kd_obstacle*(error_obstacle[i]-last_error_obstacle[i])/dt;
						}
						for(int i=obstacle_amount_zy;i<obstacel_amount_total;i++)
						{
								comd_obstacle_d[i]=kd_obstacle*(error_obstacle[i]-last_error_obstacle[i])/dt;
						}
					}
				}

				comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0];
				comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1];
				comd_yaw_rate=comd_p[2]+comd_i[2]+comd_d[2];
				comd_height_v=comd_p[3]+comd_i[3]+comd_d[3];
				if(obstacel_amount_total>0)
				{
					danger_obstacle_count=0;
					ob_danger_flag=false;
					if(obstacel_amount_total==1)
					{
						distance_obstacle=sqrt(interaction_obstacle_x[0]*interaction_obstacle_x[0]+interaction_obstacle_y[0]*interaction_obstacle_y[0])-obstacle_danger_distance;
						if(distance_obstacle>0)
						{
						comd_angle_x=(comd_angle_x+((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						comd_angle_y=(comd_angle_y+((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						printf("########safe!#######obstacle _comd  x %f y %f\n",((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])),((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						fprintf(model_txt,"########safe!################obstacle _comd  x %f y %f\n",((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])),((comd_obstacle_p[0]+comd_obstacle_i[0]+comd_obstacle_d[0]))*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
						ob_danger_flag=true;
						}
						else
						{
							comd_angle_x=(0.66*cos(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
							comd_angle_y=(0.66*sin(atan2(-interaction_obstacle_y[0],-interaction_obstacle_x[0])));
							ob_danger_flag=true;
							printf("##########Danger!!!!!!!!!!#######obstacle_comd x %f y%f\n",comd_angle_x,comd_angle_y);
							fprintf(model_txt,"##########Only One Danger! Take_off!!!!!!!!!##############\n");
						}
					}
					else
					{
						/*
						danger_obstacle_take_off=true;
						model_choose=88;
						take_off=true;
						fprintf(model_txt,"##########Above Two Danger! Take_off!!!!!!!!!##############\n");
					*/
					   for(int i=0;i<obstacel_amount_total;i++)
					   {
							distance_obstacle=sqrt(interaction_obstacle_x[i]*interaction_obstacle_x[i]+interaction_obstacle_y[i]*interaction_obstacle_y[i])-obstacle_danger_distance;
							if(distance_obstacle>0)
							{
							comd_angle_x=(comd_angle_x+((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							comd_angle_y=(comd_angle_y+((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							printf("########safe!#######obstacle _comd  x %f y %f\n",((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])),((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							fprintf(model_txt,"########safe!################obstacle _comd  x %f y %f\n",((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])),((comd_obstacle_p[i]+comd_obstacle_i[i]+comd_obstacle_d[i]))*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							ob_danger_flag=true;
							}
							else
							{
							/*
						}
							comd_angle_x=(0.66*cos(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							comd_angle_y=(0.66*sin(atan2(-interaction_obstacle_y[i],-interaction_obstacle_x[i])));
							ob_danger_flag=true;
							printf("##########Danger!!!!!!!!!!#######obstacle_comd x %f y%f\n",comd_angle_x,comd_angle_y);
							fprintf(model_txt,"##########Danger!!!!!!!!!!##############obstacle_comd x %f y%f\n",comd_angle_x,comd_angle_y);
							//printf("-----------------------------------Danger!!!!!!!!!!!----------------------------------------\n");
							danger_obstacle_count=danger_obstacle_count+1;
							if(danger_obstacle_count>1)*/
							{
								danger_obstacle_take_off=true;
								model_choose=88;
								take_off=true;
								break;
							}
							}
					}


					}
				}
				//////////////////////////////////////////////////////////////////////////////////////////跟随模式限幅
			if(comd_angle_x>0.5f)
			{
				comd_angle_x=0.5f;
			}
			else if(comd_angle_x<-0.5f)
			{
				comd_angle_x=-0.5f;
			}
			if(comd_angle_y>0.5f)
			{
				comd_angle_y=0.5f;
			}
			else if(comd_angle_y<-0.5f)
			{
				comd_angle_y=-0.5f;
			}
			if(comd_height_v>0.8f)
			{
				comd_height_v=0.8f;
			}
			else if(comd_height_v<-0.8f)
			{
				comd_height_v=-0.8f;
			}
			if(comd_yaw_rate>50.0f)
			{
				comd_yaw_rate=50.0f;
			}
			else if(comd_yaw_rate<-50.0f)
			{
				comd_yaw_rate=-50.0f;
			}
			//////////////////////////////////////////////////////////////////////////////////////////

			fprintf(model_txt,"/////////////////////////////////////////////////follow_model///////////////////////////////////////////////////////////////////////////////////\n");
			fprintf(model_txt,"model_choose  0 \n");
			fprintf(model_txt,"iRobot_x %f iRobot_y %f iRobot_theta %f \n",iRobot_x,iRobot_y,iRobot_theta*180/PI);
			fprintf(model_txt,"height_now  %f  \n",jrh_drone->ultrasonic.ranges[0]);
			fprintf(model_txt,"comd_xv  %f   comd_yv  %f   comd_zv  %f  comd_yaw_v  %f\n",comd_angle_x,comd_angle_y,comd_height_v,comd_yaw_rate);
			fprintf(model_txt,"//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////\n");//follow_txt

			fprintf(follow_txt,"time_now %f iRobot_x  %f  iRobot_y  %f  iRobot_theta  %f  height_now  %f  comd_xv  %f   comd_yv  %f   comd_zv  %f  comd_yaw_v  %f\n ",time_now,iRobot_x,iRobot_y,iRobot_theta*180/PI,jrh_drone->ultrasonic.ranges[0],comd_angle_x,comd_angle_y,comd_height_v,comd_yaw_rate);
			fprintf(follow_txt_block,"%f  %f  %f  %f  %f  %f   %f  %f  %f\n ",time_now,iRobot_x,iRobot_y,iRobot_theta*180/PI,jrh_drone->ultrasonic.ranges[0],comd_angle_x,comd_angle_y,comd_height_v,comd_yaw_rate);
////////////////////////////////////////////将指令从赛场动坐标系到地理坐标系
			float comd_vx=comd_angle_x*cvmGet(R_init,0,0)+comd_angle_y*cvmGet(R_init,0,1)+comd_height_v*cvmGet(R_init,0,2);
			float comd_vy=comd_angle_x*cvmGet(R_init,1,0)+comd_angle_y*cvmGet(R_init,1,1)+comd_height_v*cvmGet(R_init,1,2);
			float comd_zv=comd_angle_x*cvmGet(R_init,2,0)+comd_angle_y*cvmGet(R_init,2,1)+comd_height_v*cvmGet(R_init,2,2);
		////////////////////////////////////////////发送指令地理坐标系中
			drone->velocity_control(0x49,comd_vx,comd_vy,comd_zv,comd_yaw_rate);

			last_time=time_now;
			last_error[0]=error[0];
			last_error[1]=error[1];
			last_error[2]=error[2];
			last_error[3]=error[3];
		}
	else if(model_choose==3)/////////////////////////////////////////////////////////////300
	{
		//TODO
		//王宁远巡航
		/*set a circle trajectory of target*/

		if(wny_target_distance<1.2)
		{
				wny_count++;
		}
		else
		{
		}
		wny_num=wny_count%3000;
		wny_number=(float)wny_num;
		wny_x_t=7.0*sin(wny_number*3.14/1500.0)+10.0;		/*T=14.4s，r=7m，O=（0,0）*/
		wny_y_t=3.0;																				/*x_limitation and y_limitation will adjust T*/

		/*wny_x_t=1.0*cos(wny_number*3.14/1000.0)+2.0;
		wny_y_t=1.0*sin(wny_number*3.14/1000.0)+3.0;	*/


		/*read parameters*/
	
		wny_x_o=0.0;  //obstacle
		wny_y_o=0.0;
		wny_x_q=wny_px;
		wny_y_q=wny_py;
		printf("wny_x_q=%f  wny_y_q=%f\n",wny_x_q,wny_y_q);


		/*calculate target_velocity*/
	
		wny_x_qt=wny_x_t-wny_x_q;					
		wny_y_qt=wny_y_t-wny_y_q;

		if(wny_x_qt>wny_x_qt_limitation)
		{
				wny_x_qt=wny_x_qt_limitation;
		}
		else if(wny_x_qt<-wny_x_qt_limitation)
		{
				wny_x_qt=-wny_x_qt_limitation;
		}
		else
		{
		}

		if(wny_y_qt>wny_y_qt_limitation)
		{
				wny_y_qt=wny_y_qt_limitation;
		}
		else if(wny_y_qt<-wny_y_qt_limitation)
		{
				wny_y_qt=-wny_y_qt_limitation;
		}
		else
		{
		}

		wny_v_tx=wny_k_target*wny_x_qt;
		wny_v_ty=wny_k_target*wny_y_qt;
		wny_target_distance=sqrt(wny_x_qt*wny_x_qt+wny_y_qt*wny_y_qt);


		/*obstacle setting*/

		for(wny_m=0;wny_m<4;wny_m++)
		{
			wny_x_qo[wny_m]=0.0;
			wny_y_qo[wny_m]=0.0;
			wny_obstacle_distance[wny_m]=0.0;
			wny_a[wny_m]=0.0;
			wny_b[wny_m]=0.0;
			wny_k_obstacle[wny_m]=0.0;
			wny_v_ax[wny_m]=0.0;
			wny_v_ay[wny_m]=0.0;
		}

		/*read obstacle from yqt and zy*/

		for(int wny_all=0;wny_all<(obstacle_yqt.num+obstacle_zy.num);wny_all++)
		{
				if(wny_all<obstacle_yqt.num)
				{
					obstacle_all.x[wny_all]=obstacle_yqt.x[wny_all];
					obstacle_all.y[wny_all]=obstacle_yqt.y[wny_all];
				}
				else
				{
					obstacle_all.x[wny_all]=obstacle_zy.x[wny_all-obstacle_yqt.num];
					obstacle_all.y[wny_all]=obstacle_zy.y[wny_all-obstacle_yqt.num];
				}
		}
		obstacle_all.num=obstacle_yqt.num+obstacle_zy.num;


		/*calculate obstacle_velocity*/

		if(obstacle_all.num>4)			//set the number of obstacles
		{
				wny_obs_number=4;
		}
		else
		{
				wny_obs_number=obstacle_all.num;
		}


		if(obstacle_all.num>1)
		{
			for(wny_i=1;wny_i<obstacle_all.num;wny_i++)		//selection sort (obstacle>=2)
			{
				wny_x_temp=obstacle_all.x[wny_i];
				wny_y_temp=obstacle_all.y[wny_i];
				for(wny_j=(wny_i-1);wny_j>-1;wny_j--)
				{
						if((obstacle_all.x[wny_j]*obstacle_all.x[wny_j]+obstacle_all.y[wny_j]*obstacle_all.y[wny_j])>(wny_x_temp*wny_x_temp+wny_y_temp*wny_y_temp))
						{
								obstacle_all.x[wny_i]=obstacle_all.x[wny_j];
								obstacle_all.y[wny_i]=obstacle_all.y[wny_j];
								obstacle_all.x[wny_j]=wny_x_temp;
								obstacle_all.y[wny_j]=wny_y_temp;
						}
						else
						{
						}
				}


			}
		}
		else
		{
		}

		/*pick the valid numbers*/
		/*if(obstacle_all.num>0) //obstacle detected
		{
				wny_x_qo=wny_qx_ready;
				wny_y_qo=wny_qy_ready;
				wny_obs_count=0;
		}
		else
		{

				if(wny_obs_count==18) 	//delay
				{
						wny_x_qo=-10.0;
						wny_y_qo=-10.0;
				}
				else
				{
						wny_obs_count++;
				}
		}*/


		wny_k_o=wny_U*wny_k_target;
		wny_L_o=wny_R_obstacle/sqrt(wny_U/wny_T-1.0);

		if(wny_obs_number>0)
		{
			for(wny_k=0;wny_k<wny_obs_number;wny_k++)
			{
				wny_x_qo[wny_k]=-obstacle_all.x[wny_k];
				wny_y_qo[wny_k]=-obstacle_all.y[wny_k];
				wny_obstacle_distance[wny_k]=sqrt(wny_x_qo[wny_k]*wny_x_qo[wny_k]+wny_y_qo[wny_k]*wny_y_qo[wny_k]);

				printf("wny_x_qo[%d]=====%f",wny_k,wny_x_qo[wny_k]);
				printf("wny_y_qo[%d]=====%f",wny_k,wny_y_qo[wny_k]);
fprintf(wny_txt,"[time  %f ][K  %d]  wny_x_qo=====%f    wny_y_qo=====%f\n",tic(),wny_k,wny_x_qo[wny_k],wny_y_qo[wny_k]);

				if(wny_obstacle_distance[wny_k]>(wny_r_obstacle+wny_R_obstacle))
				{
						wny_k_obstacle[wny_k]=0.0;
				}
				else if(wny_obstacle_distance[wny_k]<(wny_r_obstacle+wny_R_obstacle)&&wny_obstacle_distance[wny_k]>wny_r_obstacle)
				{
						wny_a[wny_k]=(wny_obstacle_distance[wny_k]-wny_r_obstacle)/wny_L_o;
						wny_k_obstacle[wny_k]=wny_k_o/(1.0+wny_a[wny_k]*wny_a[wny_k]);
				}
				else if(wny_obstacle_distance[wny_k]<wny_r_obstacle)
				{
						wny_b[wny_k]=wny_obstacle_distance[wny_k]/wny_r_obstacle;
						wny_k_obstacle[wny_k]=wny_k_o/(wny_b[wny_k]*wny_b[wny_k]);
				}
				wny_v_ax[wny_k]=wny_k_obstacle[wny_k]*wny_x_qo[wny_k];
				wny_v_ay[wny_k]=wny_k_obstacle[wny_k]*wny_y_qo[wny_k];
			}

			/*calculate complex_velocity*/

			wny_v_x=wny_v_tx;
			wny_v_y=wny_v_ty;
			for(wny_k=0;wny_k<wny_obs_number;wny_k++)
			{
				wny_v_x=wny_v_x+wny_v_ax[wny_k];
				wny_v_y=wny_v_y+wny_v_ay[wny_k];
			}

			/*case 1：enter a local minimum*/

			if((fabs(wny_v_x)<0.05&&fabs(wny_v_y)<0.05)&&wny_target_distance>0.2)
			{
				// raise the avoid_velocity, ensure safety and escape form local minimum

				wny_v_x=wny_v_tx;
				wny_v_y=wny_v_ty;
				for(wny_k=0;wny_k<wny_obs_number;wny_k++)
				{
						wny_v_x=wny_v_x+1.1*wny_v_ax[wny_k];
						wny_v_y=wny_v_y+1.1*wny_v_ay[wny_k];
				}
			}
			else
			{
			}

			/*case 2：different management of obstacles by number*/

			//sum all of obstacles first, then calculate target and obstacle_total separately

			wny_v_a_x=0.0;
			wny_v_a_y=0.0;

			for(wny_k=0;wny_k<wny_obs_number;wny_k++)
			{
				wny_v_a_x=wny_v_a_x+wny_v_ax[wny_k];
				wny_v_a_y=wny_v_a_y+wny_v_ay[wny_k];
			}

			if(wny_obs_number==1)								//section 1: two obstacles opposite
			{
					wny_v_t=sqrt(wny_v_tx*wny_v_tx+wny_v_ty*wny_v_ty);
					wny_v_a=sqrt(wny_v_a_x*wny_v_a_x+wny_v_a_y*wny_v_a_y);
					wny_v_at=wny_v_a*wny_v_t;
					wny_v_multi=wny_v_tx*wny_v_a_x+wny_v_ty*wny_v_a_y;

					if(wny_v_multi<(-0.8*wny_v_at))		//cos(angle)<-0.8
					{
							//avoid through right
							wny_v_x=wny_v_tx;
							wny_v_y=wny_v_ty;
							for(wny_k=0;wny_k<wny_obs_number;wny_k++)
							{
									wny_v_x=wny_v_x+wny_v_ax[wny_k]-wny_v_ay[wny_k];
									wny_v_y=wny_v_y+wny_v_ay[wny_k]+wny_v_ax[wny_k];
							}
					}
					else
					{
					}
					wny_altitude_flag=0;
			}

			else if(wny_obs_number==2)					//section 2: two obstacles opposite
			{
					if(fabs(wny_v_a_x)<0.4&&fabs(wny_v_a_y)<0.4)
					{
							wny_v_x=wny_v_tx-wny_v_ay[0];
							wny_v_y=wny_v_tx+wny_v_ax[0];
							wny_altitude_flag=0;
					}
					else
					{
							wny_v_t=sqrt(wny_v_tx*wny_v_tx+wny_v_ty*wny_v_ty);
							wny_v_a=sqrt(wny_v_a_x*wny_v_a_x+wny_v_a_y*wny_v_a_y);
							wny_v_at=wny_v_a*wny_v_t;
							wny_v_multi=wny_v_tx*wny_v_a_x+wny_v_ty*wny_v_a_y;

							if(wny_v_multi<(-0.8*wny_v_at))		//cos(angle)<-0.8
							{
									//avoid through right
									wny_v_x=wny_v_tx;
									wny_v_y=wny_v_ty;
									for(wny_k=0;wny_k<wny_obs_number;wny_k++)
									{
											wny_v_x=wny_v_x+wny_v_ax[wny_k]-wny_v_ay[wny_k];
											wny_v_y=wny_v_y+wny_v_ay[wny_k]+wny_v_ax[wny_k];
									}
							}
							else
							{
							}
							wny_altitude_flag=0;
					}
			}
			else if(wny_obs_number==3)					//section 3: three obstacles triangle
			{
					wny_two_add=fabs(wny_v_ax[1]+wny_v_ax[2])+fabs(wny_v_ay[1]+wny_v_ay[2]);
					wny_three_add=fabs(wny_v_ax[0]+wny_v_ax[1]+wny_v_ax[2])+fabs(wny_v_ay[0]+wny_v_ay[1]+wny_v_ay[2]);
					if(wny_two_add>wny_three_add)
					{
							model_choose=88;
							danger_obstacle_take_off=true;
							take_off=true;
							//wny_altitude_flag=1;					//to switch the jrh model_choose=88
					}
					else
					{
							wny_v_t=sqrt(wny_v_tx*wny_v_tx+wny_v_ty*wny_v_ty);
							wny_v_a=sqrt(wny_v_a_x*wny_v_a_x+wny_v_a_y*wny_v_a_y);
							wny_v_at=wny_v_a*wny_v_t;
							wny_v_multi=wny_v_tx*wny_v_a_x+wny_v_ty*wny_v_a_y;

							if(wny_v_multi<(-0.8*wny_v_at))		//cos(angle)<-0.8
							{
									//avoid through right
									wny_v_x=wny_v_tx;
									wny_v_y=wny_v_ty;
									for(wny_k=0;wny_k<wny_obs_number;wny_k++)
									{
											wny_v_x=wny_v_x+wny_v_ax[wny_k]-wny_v_ay[wny_k];
											wny_v_y=wny_v_y+wny_v_ay[wny_k]+wny_v_ax[wny_k];
									}
							}
							else
							{
							}
							wny_altitude_flag=0;
					}

			}
			else if(wny_obs_number==4)					//section 3: three obstacles triangle
			{
					wny_two_sum=fabs(wny_v_ax[2]+wny_v_ax[3])+fabs(wny_v_ay[2]+wny_v_ay[3]);
					wny_three_sum=fabs(wny_v_ax[1]+wny_v_ax[2]+wny_v_ax[3])+fabs(wny_v_ay[1]+wny_v_ay[2]+wny_v_ay[3]);
					wny_four_sum=fabs(wny_v_ax[0]+wny_v_ax[1]+wny_v_ax[2]+wny_v_ax[3])+fabs(wny_v_ay[0]+wny_v_ay[1]+wny_v_ay[2]+wny_v_ay[3]);
					if(wny_two_sum>wny_three_sum||wny_three_sum>wny_four_sum)
					{
							model_choose=88;
							danger_obstacle_take_off=true;
							take_off=true;
							//wny_altitude_flag=1;					//to switch the jrh model_choose=88
					}
					else
					{
							wny_v_t=sqrt(wny_v_tx*wny_v_tx+wny_v_ty*wny_v_ty);
							wny_v_a=sqrt(wny_v_a_x*wny_v_a_x+wny_v_a_y*wny_v_a_y);
							wny_v_at=wny_v_a*wny_v_t;
							wny_v_multi=wny_v_tx*wny_v_a_x+wny_v_ty*wny_v_a_y;

							if(wny_v_multi<(-0.8*wny_v_at))		//cos(angle)<-0.8
							{
									//avoid through right
									wny_v_x=wny_v_tx;
									wny_v_y=wny_v_ty;
									for(wny_k=0;wny_k<wny_obs_number;wny_k++)
									{
											wny_v_x=wny_v_x+wny_v_ax[wny_k]-wny_v_ay[wny_k];
											wny_v_y=wny_v_y+wny_v_ay[wny_k]+wny_v_ax[wny_k];
									}
							}
							else
							{
							}
							wny_altitude_flag=0;
					}

			}
		}
		else if(wny_obs_number==0)
		{
			wny_v_x=wny_v_tx;
			wny_v_y=wny_v_ty;
			wny_altitude_flag=0;
		}
		printf("obstacle_number===========%d\n",wny_obs_number);

		/*case 3：total velocity towards blind area*/
		wny_yaw_now=yaw_now;

		wny_one_begin=wny_yaw_now+0.785;			//one, two, and three is from 0 to π
		wny_one_end=wny_yaw_now+1.309;
		wny_two_begin=wny_yaw_now+2.880;
		wny_two_end=wny_yaw_now+3.142;
		wny_three_begin=wny_yaw_now-3.142;
		wny_three_end=wny_yaw_now-2.880;
		wny_four_begin=wny_yaw_now-1.309;
		wny_four_end=wny_yaw_now-0.785;

		if(wny_one_begin>3.14)				//area_one is between -π to π
		{
				wny_one_begin=wny_one_begin-6.28;
		}
		else
		{
		}
		if(wny_one_end>3.14)
		{
				wny_one_end=wny_one_end-6.28;
		}
		else
		{
		}

		if(wny_two_begin>3.14)				//area_two is between -π to π
		{
				wny_two_begin=wny_two_begin-6.28;
		}
		else
		{
		}
		if(wny_two_end>3.14)
		{
				wny_two_end=wny_two_end-6.28;
		}
		else
		{
		}

		if(wny_three_begin<-3.14)			//area_three is between -π to π
		{
				wny_three_begin=wny_three_begin+6.28;
		}
		else
		{
		}
		if(wny_three_end<-3.14)
		{
				wny_three_end=wny_three_end+6.28;
		}
		else
		{
		}

		if(wny_four_begin<-3.14)			//area_four is between -π to π
		{
				wny_four_begin=wny_four_begin+6.28;
		}
		else
		{
		}
		if(wny_four_end<-3.14)
		{
				wny_four_end=wny_four_end+6.28;
		}
		else
		{
		}

		wny_vx=(double)wny_v_x;			//calculate the velocity angle in field coordinate
		wny_vy=(double)wny_v_y;
		wny_angle=(float)atan2(wny_vy,wny_vx);

		if((wny_one_end-wny_one_begin)<0)
		{
				wny_flagone=1;
		}
		else
		{
				wny_flagone=0;
		}

		if((wny_two_end-wny_two_begin)<0)
		{
				wny_flagtwo=1;
		}
		else
		{
				wny_flagtwo=0;
		}

		if((wny_three_end-wny_three_begin)<0)
		{
				wny_flagthree=1;
		}
		else
		{
				wny_flagthree=0;
		}

		if((wny_four_end-wny_four_begin)<0)
		{
				wny_flagfour=1;
		}
		else
		{
				wny_flagfour=0;
		}

		if((wny_angle>wny_one_begin&&wny_angle<3.14)||(wny_angle<wny_one_end&&wny_angle>-3.14))	//velocity towards to blind area or not
		{
				wny_angleone=1;
		}
		else if(wny_angle>wny_one_begin&&wny_angle<wny_one_end)
		{
				wny_angleone=0;
		}

		if((wny_angle>wny_two_begin&&wny_angle<3.14)||(wny_angle<wny_two_end&&wny_angle>-3.14))	//velocity towards to blind area or not
		{
				wny_angletwo=1;
		}
		else if(wny_angle>wny_two_begin&&wny_angle<wny_two_end)
		{
				wny_angletwo=0;
		}

		if((wny_angle>wny_three_begin&&wny_angle<3.14)||(wny_angle<wny_three_end&&wny_angle>-3.14))	//velocity towards to blind area or not
		{
				wny_anglethree=1;
		}
		else if(wny_angle>wny_three_begin&&wny_angle<wny_three_end)
		{
				wny_anglethree=0;
		}

		if((wny_angle>wny_four_begin&&wny_angle<3.14)||(wny_angle<wny_four_end&&wny_angle>-3.14))	//velocity towards to blind area or not
		{
				wny_anglefour=1;
		}
		else if(wny_angle>wny_four_begin&&wny_angle<wny_four_end)
		{
				wny_anglefour=0;
		}


		if((wny_flagone==1)&&(wny_angleone==1))
		{
				wny_judgement=1;
		}
		else if((wny_flagone==0)&&(wny_angleone==0))
		{
				wny_judgement=1;
		}
		else if((wny_flagtwo==1)&&(wny_angletwo==1))
		{
				wny_judgement=1;
		}
		else if((wny_flagtwo==0)&&(wny_angletwo==0))
		{
				wny_judgement=1;
		}
		else if((wny_flagthree==1)&&(wny_anglethree==1))
		{
				wny_judgement=1;
		}
		else if((wny_flagthree==0)&&(wny_anglethree==0))
		{
				wny_judgement=1;
		}
		else if((wny_flagfour==1)&&(wny_anglefour==1))
		{
				wny_judgement=1;
		}
		else if((wny_flagfour==0)&&(wny_anglefour==0))
		{
				wny_judgement=1;
		}
		else
		{
				wny_judgement=0;
		}

		if(wny_judgement==1)
		{
				wny_vyaw_extra=26.0;
		}
		else if(wny_judgement==0)
		{
				wny_vyaw_extra=0.0;
		}
		wny_v_yaw=15.0+wny_vyaw_extra;



		/*mode 1 and mode 2 switch for surround*/

		/*if(wny_altitude_flag==0)
		{
				if(wny_altitude_count<201&&wny_altitude_count>0)	//raise and hold on 1.8m for total 4s
				{
					wny_altitude_small=1.5;
					wny_altitude_big=1.7;
					wny_v_altitude=0.4;
					wny_v_x=0.0;
					wny_v_y=0.0;
					wny_altitude_count++;
				}
				else
				{
					wny_altitude_small=0.9;
					wny_altitude_big=1.1;
					wny_v_altitude=0.1;
					wny_altitude_count=0;
				}
		}
		else if(wny_altitude_flag==1)
		{
				wny_altitude_small=1.5;
				wny_altitude_big=1.7;
				wny_v_altitude=0.4;
				wny_v_x=0.0;
				wny_v_y=0.0;
				wny_altitude_count++;
		}*/

		if(wny_altitude_flag==0||wny_altitude_flag==1)
		{
				wny_altitude_small=1.35;
				wny_altitude_big=1.45;
				wny_v_altitude=0.1;
		}

		/*limitation of velocity signal*/
	
		if(wny_v_x>wny_vx_limitation)
		{
			wny_v_x=wny_vx_limitation;
		}
		else if(wny_v_x<-wny_vx_limitation)
		{
			wny_v_x=-wny_vx_limitation;
		}
		else
		{
		}

		if(wny_v_y>wny_vy_limitation)
		{
			wny_v_y=wny_vy_limitation;
		}
		else if(wny_v_y<-wny_vy_limitation)
		{
			wny_v_y=-wny_vy_limitation;
		}
		else
		{
		}


		/*keep M100 on a certain altitude*/
		wny_pz=jrh_drone->ultrasonic.ranges[0];
		if(wny_pz<wny_altitude_small)
		{
		         wny_v_h=wny_v_altitude;
		}
		else if(wny_pz>wny_altitude_big)
		{
			     wny_v_h=-wny_v_altitude;
		}
		else
		{
			     wny_v_h=0.0;
		}


		/*transmit velocity signal to M100*/
		float comd_vx=wny_v_x*cvmGet(R_init,0,0)+wny_v_y*cvmGet(R_init,0,1)+wny_v_h*cvmGet(R_init,0,2);
		float comd_vy=wny_v_x*cvmGet(R_init,1,0)+wny_v_y*cvmGet(R_init,1,1)+wny_v_h*cvmGet(R_init,1,2);
		float comd_zv=wny_v_x*cvmGet(R_init,2,0)+wny_v_y*cvmGet(R_init,2,1)+wny_v_h*cvmGet(R_init,2,2);
	////////////////////////////////////////////发送指令地理坐标系中
		drone->velocity_control(0x49,comd_vx,comd_vy,comd_zv,0.0);
			
		/*drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_PALSTANCE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |			//BODY or GROUND
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            wny_v_x,wny_v_y,0.0,0.0);*/
		
	}
	else if(model_choose==88 || model_choose==89 || model_choose==90)//如果180度感应成功，则起飞
	{
		if(first_take_off)
		{
			last_time=0;
			time_now=tic();
			dt=0.0f;
			comd_i[0]=0;
			comd_i[1]=0;
			comd_i[2]=0;
			comd_i[3]=0;
			first_take_off=false;
		}
		else
		{
			time_now=tic();
			dt=time_now-last_time;
		}
		/*
		////////////////////////////////////////////////计算偏航角///////////////////////////////////////////////////
		//计算得到当前的姿态角att_right_now
	   cvmSet(q_right_now, 0, 0, jrh_drone->attitude_quaternion.q0);
	   cvmSet(q_right_now, 1, 0, jrh_drone->attitude_quaternion.q1);
	   cvmSet(q_right_now, 2, 0, jrh_drone->attitude_quaternion.q2);
	   cvmSet(q_right_now, 3, 0, jrh_drone->attitude_quaternion.q3);
	   Quaternion_To_Euler(q_right_now,att_right_now);
		///////////////////////////////////////////

	    float yaw_now=(-cvmGet(att_init,2,0)+cvmGet(att_right_now,2,0));
	   ////////////////////////////////////////

	   		if(yaw_now>PI)
	   		{
	   			yaw_now=yaw_now-2*PI;
	   		}
	   		else if(yaw_now<-PI)
	   		{
	   			yaw_now=yaw_now+2*PI;
	   		}
	 	   ////////////////////////////////////////
 	  // printf("pitch %f roll %f yaw %f \n",180*cvmGet(att_right_now,1,0)/PI,180*cvmGet(att_right_now,0,0)/PI,180*cvmGet(att_right_now,2,0)/PI);
 	    printf("yaw_now %f\n",yaw_now*180/PI);
 	    fprintf(model_txt,"yaw_now %f \n",yaw_now*180/PI);
		*/
		if(model_choose==88)//model_choose==88的处理：包括紧急起飞处理、飞行中丢目标处理
		{
			if(danger_obstacle_take_off)
			{
				error[3]=danger_obstacle_take_off_height-jrh_drone->ultrasonic.ranges[0];
				printf("danger_com_height%f\n",danger_obstacle_take_off_height);
				fprintf(model_txt,"danger_com_height%f",danger_obstacle_take_off_height);
			}
			else
			{
			  error[3]=take_off_height-jrh_drone->ultrasonic.ranges[0];
				printf("com_height%f\n",take_off_height);
				fprintf(model_txt,"com_height%f",take_off_height);
			}
		}
		else if(model_choose==89)
		{
			if(first_89_flag==0)
			{
				original_x_89=wx_px;
				original_y_89=wx_py;
				first_89_flag=1;
			}
			first_89_flag=1;
			error[0]=original_x_89-wx_px;
			error[1]=original_y_89-wx_py;
			error[3]=take_down_height-jrh_drone->ultrasonic.ranges[0];
			printf("take_down_height%f original_x_89 %f original_y_89 %f\n",take_down_height,original_x_89,original_y_89);
			fprintf(model_txt,"take_down_height%f",take_down_height);
		}
		else if(model_choose==90)//你好
		{
			if(circle_count==0 && circle_flag==0)
			{
				original_x_t=wx_px;
				original_y_t=wx_py;
				aim_position_x[0]=original_x_t+circle_r;aim_position_y[0]=original_y_t+circle_r;
				aim_position_x[1]=original_x_t-circle_r;aim_position_y[1]=original_y_t+circle_r;
				aim_position_x[2]=original_x_t-circle_r;aim_position_y[2]=original_y_t-circle_r;
				aim_position_x[3]=original_x_t+circle_r;aim_position_y[3]=original_y_t-circle_r;
				aim_position_x[4]=original_x_t+0.0;aim_position_y[4]=original_y_t+0.0;
				circle_count=1;
			}
			if(circle_count<6 && circle_flag==0 && circle_count>=1)
			{
				total_obstacle_bottom_num+=obstacle_bottom_num;
				error[0]=aim_position_x[circle_count-1]-wx_px;
				error[1]=aim_position_y[circle_count-1]-wx_py;
				circle_target_distance=sqrt(error[0]*error[0]+error[1]*error[1]);
				if(circle_target_distance<0.2)
				{
					circle_count++;
				}
			}
			if(circle_count>=6)
			{
				circle_count=0;
				circle_flag=1;
			}

			error[3]=danger_obstacle_take_off_height-jrh_drone->ultrasonic.ranges[0];
			printf("hover_height %f circle_target_distance %f\n",danger_obstacle_take_off_height,circle_target_distance);
			fprintf(model_txt,"hover_height %f circle_target_distance %f",danger_obstacle_take_off_height,circle_target_distance);

			printf("original_x_t  %f  original_y_t  %f  circle_count %d  circle_flag %d  total_obstacle_bottom_num  %d\n",original_x_t,original_y_t,circle_count,circle_flag,total_obstacle_bottom_num);//你好
			fprintf(model_txt,"original_x_t  %f  original_y_t  %f  circle_count %d  circle_flag %d  total_obstacle_bottom_num  %d\n",original_x_t,original_y_t,circle_count,circle_flag,total_obstacle_bottom_num);

		}
////////////////////////////////////////////计算kp项指令大小
	      comd_p[0]=error[0]*kp[0];
	      comd_p[1]=error[1]*kp[1];
	      comd_p[3]=error[3]*kp[3];
////////////////////////////////////////////计算ki项指令大小
	      if(dt==0.0f)
	     {
	    	  comd_i[3]=0.0f;
	     }
	     else
	     {
	    	 comd_i[3]=(comd_i[3]+error[3]*dt)*ki[3];
	     }
////////////////////////////////////////////计算kd项指令大小
	     if(dt==0.0f)
	     {
	    	 comd_d[3]=0.0f;
	     }
	     else
	     {
	     	 comd_d[3]=kd[3]*(error[3]-last_error[3])/dt;
	     }
	     comd_angle_x=comd_p[0];
	     comd_angle_y=comd_p[1];

		 if(model_choose==89)//下降时使飞行器保持偏航旋转//if(fabs(yaw_now*180/PI-original_yaw)>90)//测试时如果不转，可能是yaw_now一直给original_yaw赋值的原因，正常飞行时应该不会
		{
			comd_yaw_rate=15;
		}
		else
		{
			comd_yaw_rate=0;
		}
	     comd_height_v=comd_p[3]+comd_i[3]+comd_d[3];

	//////////////////////////////////限幅///////////////////////////////////////
		 if(comd_angle_x>0.3f)
		comd_angle_x=0.3f;
		 if(comd_angle_x<-0.3f)
		comd_angle_x=-0.3f;
		 if(comd_angle_y>0.3f)
		comd_angle_y=0.3f;
		 if(comd_angle_y<-0.3f)
		comd_angle_y=-0.3f;

			printf("comd_angle_x %f  comd_angle_y %f  comd_yaw_rate  %f\n",comd_angle_x,comd_angle_y,comd_yaw_rate);
			fprintf(model_txt,"comd_angle_x %f  comd_angle_y %f  comd_yaw_rate  %f\n",comd_angle_x,comd_angle_y,comd_yaw_rate);

	 	float comd_vx=comd_angle_x*cvmGet(R_init,0,0)+comd_angle_y*cvmGet(R_init,0,1)+comd_height_v*cvmGet(R_init,0,2);
	 	float comd_vy=comd_angle_x*cvmGet(R_init,1,0)+comd_angle_y*cvmGet(R_init,1,1)+comd_height_v*cvmGet(R_init,1,2);
	 	float comd_zv=comd_angle_x*cvmGet(R_init,2,0)+comd_angle_y*cvmGet(R_init,2,1)+comd_height_v*cvmGet(R_init,2,2);
	 	drone->velocity_control(0x49,comd_vx,comd_vy,comd_zv,comd_yaw_rate);
	 	last_time=time_now;
	 	last_error[0]=error[0];
	 	last_error[1]=error[1];
	 	last_error[3]=error[3];
	}

	////////////////////////跳出程序也要降落45度或者180度降落

	if(land && (jrh_drone->object_pos_dynamic.target_num==0) )
	{
			  time_now=tic();
			  dt=time_now-last_time;
			  if(model_choose==1)
			  {
				  if(time_now-remember_time<parallel_land_time_45_first)
				  {
					  comd_height=0.24;
				  }
				  else
				  {
					  comd_height=0.18;
				  }
			  }
			  else if(model_choose==2)
			  {
				  if (tic() - remember_time < parallel_land_time_180_first)
				  {
					  comd_height = 1;
				  }
				  else
				  {
					  comd_height = 0.2;
				  }
			  }
			  error[0] = 0;
			  error[1] = 0;
			  error[3]=comd_height-jrh_drone->ultrasonic.ranges[0];
		  ////////////////////////////////////////////计算kp项指令大小
		  	  comd_p[3]=error[3]*kp[3];
		  	////////////////////////////////////////////计算ki项指令大小
		  		if(dt==0.0f)
		  		{
		  			comd_i[3]=0.0f;
		  		}
		  		else
		  		{
		  			comd_i[3]=(comd_i[3]+error[3]*dt)*ki[3];
		  		}
		  	////////////////////////////////////////////计算kd项指令大小
		  		if(dt==0.0f)
		  		{
		  			comd_d[3]=0.0f;
		  		}
		  		else
		  		{
		  			comd_d[3]=kd[3]*(error[3]-last_error[3])/dt;
		  		}
				if(model_choose==1)
				{
						printf("[45]Land-Lost  Start!!\n");
						fprintf(model_txt,"[45]Land-Lost  Start!!\n");
						comd_angle_x=0+0.2*cos(remember_theta);//原来0.26，静态好使
						comd_angle_y=0+0.2*sin(remember_theta);
						comd_yaw_rate=0;
						comd_height_v=comd_p[3]+comd_i[3]+comd_d[3];
						fprintf(error_txt,"[45-land-lost] [time]  %f  [error_x]  %f  [error_y]  %f   [error_yaw] %f   [error_z]  %f   \n",time_now,error[0],error[1],error[2],error[3]);
						fprintf(error_block_txt,"%f  %f  %f   %f   %f   \n",time_now,error[0],error[1],error[2],error[3]);
						fprintf(comd_setpoint,"[45-land-lost] [time]  %f  [vx]  %f  [ x_t]  %f   [vy] %f   [y_t]  %f  [r_vt] %f   [i_theta] %f [c_h] %f  [vz] %f [yaw_rate] %f [height_now] %f  [yaw_now] %f [comd_yaw] %f [vx_now] %f [vy_now] %f [vz_now] %f [r_vt_original]%f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,jrh_drone->ultrasonic.ranges[0],yaw_now*180/PI,comd_yaw,jrh_drone->velocity.vx,jrh_drone->velocity.vy,jrh_drone->velocity.vz,r_vt_original);
						fprintf(comd_setpoint_block,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,jrh_drone->ultrasonic.ranges[0],yaw_now*180/PI,comd_yaw,jrh_drone->velocity.vx,jrh_drone->velocity.vy,jrh_drone->velocity.vz,r_vt_original);
				}
				else if(model_choose==2)
				{
						printf("[180]Land-Lost  Start!!\n");
						fprintf(model_txt,"[180]Land-Lost  Start!!\n");
						  if (tic() - remember_time <parallel_land_time_180_first)////////////////////////////180向后延迟降落时间
						  {
								comd_angle_x=velcity_land_180*cos(remember_theta);
								comd_angle_y=velcity_land_180*sin(remember_theta);
						  }
						  else
						  {
							  comd_angle_x=0;
							  comd_angle_y=0;
						  }

						comd_yaw_rate=0;
						comd_height_v=comd_p[3]+comd_i[3]+comd_d[3];
						printf("[180] v_z %f \n",comd_height_v);
						printf("############################################################\n");
						printf("remember_time %f\n", remember_time);
						fprintf(model_txt, "############################################################\n");
						fprintf(model_txt, "remember_time %f\n", remember_time);
						fprintf(model_txt,"[180] v_z %f  comd_height %f \n",comd_height_v,comd_height);
						fprintf(error_txt,"[180-land-lost] [time]  %f  [error_x]  %f  [error_y]  %f   [error_yaw] %f   [error_z]  %f   \n",time_now,error[0],error[1],error[2],error[3]);
						fprintf(error_block_txt,"%f  %f  %f  %f   %f   \n",time_now,error[0],error[1],error[2],error[3]);
						fprintf(comd_setpoint,"[180-land-lost] [time]  %f  [comd_vx]  %f  [ x_t]  %f   [comd_vy] %f   [y_t]  %f  [r_vt] %f   [theta] %f [c_height] %f  [vz] %f [yaw_rate] %f  [r_vt_original] %f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,r_vt_original);
						fprintf(comd_setpoint_block,"  %f    %f    %f   %f   %f     %f   %f    %f   %f  %f %f\n",time_now,comd_angle_x,iRobot_x,comd_angle_y,iRobot_y,r_vt,iRobot_theta*180/PI,comd_height,comd_height_v,comd_yaw_rate,r_vt_original);
				}
				float comd_vx=comd_angle_x*cvmGet(R_init,0,0)+comd_angle_y*cvmGet(R_init,0,1)+comd_height_v*cvmGet(R_init,0,2);
				float comd_vy=comd_angle_x*cvmGet(R_init,1,0)+comd_angle_y*cvmGet(R_init,1,1)+comd_height_v*cvmGet(R_init,1,2);
				float comd_zv=comd_angle_x*cvmGet(R_init,2,0)+comd_angle_y*cvmGet(R_init,2,1)+comd_height_v*cvmGet(R_init,2,2);
				drone->velocity_control(0x49,comd_vx,comd_vy,comd_zv,comd_yaw_rate);
				last_time=time_now;
				last_error[3]=error[3];
				if(model_choose==1)
				{
					if (tic() - remember_time>parallel_land_time_45_second)
					{
						land = false;
						take_off = true;
						model_choose = 88;
						printf("############################################################\n");
						printf("remember_time %f\n", remember_time);
						fprintf(model_txt, "############################################################\n");
						fprintf(model_txt, "remember_time %f\n", remember_time);
					}
				}


				if(model_choose==2)
				{
					if (fabs(comd_height - 0.2)<0.01)
					{
						if (fabs(error[3])<0.05)
						{
							if (first_land == true)
							{
								first_land_time = tic();
								first_land = false;
							}
							if(tic()-first_land_time>0.5)
							{
								acc_total = jrh_drone->acceleration.ax*jrh_drone->acceleration.ax + jrh_drone->acceleration.ay*jrh_drone->acceleration.ay;
								if (acc_total>acc_land_to_take_off)
								{
									land = false;
									take_off = true;
									model_choose = 88;
									fprintf(model_txt, "碰撞起飞 \n");
								}
							}

							if (tic() - first_land_time>parallel_land_time_180_second)
							{
								land = false;
								take_off = true;
								model_choose = 88;
								fprintf(model_txt, "到点起飞啦 \n");
							}
							fprintf(model_txt, "first_land_time %f\n", first_land_time);
						}
					}
				}
	}
}
	   rate.sleep();
}
}
