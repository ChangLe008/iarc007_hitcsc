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
#define circle_r 0.4//避障90模式边长的一半


FILE *model_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/model.txt","w");
FILE *input_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/input.txt","w");
FILE *output_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/output.txt","w");
double tic()
{
	struct timeval t;

	gettimeofday(&t,NULL);
	return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}
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
int height_flag;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
int last_model_choose;
int jrh_iRobot_i;
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
float landheight_1;
float landheight_2;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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


    ros::Subscriber obstacle_subscriber;
    ros::Subscriber obstacle_yqt_subscriber;


while(ros::ok())
{
	ROS_INFO("===========");
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

	iRobot_x=(jrh_drone->object_pos_dynamic.target_x[jrh_iRobot_i])*0.01;
	iRobot_y=(jrh_drone->object_pos_dynamic.target_y[jrh_iRobot_i])*0.01;
	iRobot_theta=jrh_drone->object_pos_dynamic.target_dir[jrh_iRobot_i];
	iRobot_num=jrh_drone->object_pos_dynamic.target_num;
	iRobot_original_x=iRobot_x;
	iRobot_original_y=iRobot_y;

	printf("iRobot_x %f iRobot_y %f iRobot_theta %f iRobot_num %d height %f\n ",iRobot_x,iRobot_y,iRobot_theta,iRobot_num,jrh_drone->ultrasonic.ranges[0]);
	fprintf(model_txt,"iRobot_x %f iRobot_y %f iRobot_theta %f iRobot_num %d\n ",iRobot_x,iRobot_y,iRobot_theta,iRobot_num);


	if(iRobot_num>0)
		jrh_model=1;
	else if(iRobot_num==0)
		jrh_model=3;
	//TO/DO
	//////////////////////////////////////////////////////////////////////////////////////////////比赛快速调参
 distance_ahead_45=0.0;distance_ahead_180=0.1;//机器人向前叠加距离
 height_limited=1.5;//45度/180度/跟踪模式高度限幅
 forbid_distance_land=1.8;//距离障碍物这个距离时禁止降落
 land_condition_r_45=0.08;land_condition_erryaw_45=15; land_condition_height_45=1;//45度降落条件
 land_condition_r_180=0.15; land_condition_erryaw_180=10;land_condition_height_180=1.6;//180度降落条件
 parallel_land_time_45_first=1;parallel_land_time_45_second=3;//45度降落两段持续时间
 parallel_land_time_180_first=2;parallel_land_time_180_second=5;//180度降落两段持续时间
 obstacle_danger_distance=1.65;//避障反向跑的距离
 velcity_land_45=0;//0.2;//45度降落速度，维持大小
 comd_height_if_obstacle = 1.25;
 comd_follow_height = 1.5;
 velcity_zhuanquan=0.3;
 yaw_rate_zhuanquan=18;
 velcity_land_180=0.5;//180度降落速度大小
 parallel_land_velcity_180 =0;// 0.2;//180度平行速度大小
 landheight_1=0.24;        landheight_2=0.18;    //45度交互land中两次给定高度值
	//////////////////////////////////////////////////////////////////////////////////////////////


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

if(abs(drone->rc_channels.mode-8000.0)<1)
{
	if(drone->rc_channels.gear==(-4545))
	{
		usleep(100);	
		drone->request_sdk_permission_control();
	}
	////////////////////////////////////////////////////////////////
	//计算得到当前的姿态角att_right_now
   cvmSet(q_right_now, 0, 0, jrh_drone->attitude_quaternion.q0);
   cvmSet(q_right_now, 1, 0, jrh_drone->attitude_quaternion.q1);
   cvmSet(q_right_now, 2, 0, jrh_drone->attitude_quaternion.q2);
   cvmSet(q_right_now, 3, 0, jrh_drone->attitude_quaternion.q3);
   Quaternion_To_Euler(q_right_now,att_right_now);

   printf("%f  %f  %f  %f\n",jrh_drone->attitude_quaternion.q0,jrh_drone->attitude_quaternion.q1,jrh_drone->attitude_quaternion.q2,jrh_drone->attitude_quaternion.q3);
	///////////////////////////////////////////

if(land==false && take_off==false )////////////////////////////////策略判断
{
	model_choose=jrh_model;
}

if(take_off==true)///////////////////////////////////////////////
{
	model_choose=jrh_model;
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
printf("model_choose: %d\n",model_choose);
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


	jrh_drone->jrh_model_choose.header.stamp=ros::Time::now();
	jrh_drone->jrh_model_choose.model_choose=model_choose;
	jrh_drone->jrh_model_choose.target_num_choose=wx_iRobot_serial_num[jrh_iRobot_i];
	jrh_drone->jrh_model_choose_publisher.publish(jrh_drone->jrh_model_choose);


	last_model_choose=model_choose;
	fprintf(model_txt,"**************************************************  time  %f  ****************************\n",tic());
	printf("*******************model_choose     %d   %d***********\n",model_choose,land);




	//TODO


	//////////////////////////////////////////////////////////////////////////
	obstacle_amount_zy=jrh_drone->obstacle_pos_dynamic.num;//从张雨订阅一共看到几个机器人
	obstacle_amount_yqt=jrh_drone->obstacle_pos_dynamic_yqt.num;//从于庆涛订阅一共看到几个机器人
	obstacel_amount_total=obstacle_amount_zy+obstacle_amount_yqt;
	obstacel_amount_total=0;
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
		}
		for(int i=obstacel_amount_total;i<8;i++)
		{
			interaction_obstacle_x[i]=-80;
			interaction_obstacle_y[i]=-80;
		}
	}
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
			  kp[0]=1.0; kp[1]=1.0; kp[2]=0.8; kp[3]=0.8;//1.5
			  ki[0]=0.01; ki[1]=0.01; ki[2]=0.00; ki[3]=0.00;
			  kd[0]=0.01; kd[1]=0.01; kd[2]=0.01; kd[3]=0.02;
		  }
		  else if(jrh_drone->ultrasonic.ranges[0]>0.7)
		  {
			  kp[0]=0.8; kp[1]=0.8; kp[2]=0.8; kp[3]=0.8;//1.194
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

	  if(model_choose==88)
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
obstacel_amount_total=0;/////////TODO
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
	    r_vt_original=sqrt(iRobot_original_x*iRobot_original_x+iRobot_original_y*iRobot_original_y);//20161224相机坐标系内与目标的相对距离     要

	    if(model_choose==1)
	    {
			iRobot_x=iRobot_x+ distance_ahead_45*cos(iRobot_theta)-cos(yaw_now)*fabs(-0.132*sin(-cvmGet(att_right_now,1,0))+0.12*cos(-cvmGet(att_right_now,1,0)));//不太懂
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

		pos_now[2]=jrh_drone->ultrasonic.ranges[0];///飞行高度
		printf("ssssssssssssssssss%f",jrh_drone->ultrasonic.ranges[1]);
		fprintf(input_txt," %f %f  %f  %f  %d %d %f %f  %f  %f\n ",tic(),iRobot_x,iRobot_y,iRobot_theta,iRobot_num,land,jrh_drone->ultrasonic.ranges[0],r_vt,r_vt_original,yaw_now);
///////////////////////////////////////////相机高度，规划高度		
		if(model_choose==1)
		{
			if(fabs(r_vt_original-0.1)>0.4)
			{
				height_H=r_vt_original*1.5+0.42;
				height_L=r_vt_original*1.5+0.195;
				height_flag=1;
			}
			else if(fabs(r_vt_original-0.1)>0.1 && fabs(r_vt_original-0.1)<0.4)
			{
				height_H=fabs(r_vt_original-0.1)*2.05+0.345;
				height_L=r_vt_original*1.5+0.195;
				height_flag=2;
			}
			else if(fabs(r_vt_original-0.1)<0.1)
			{
				height_H=0.55;
				height_L=0.495;
				height_flag=3;
			}
			else 
				height_flag=0;

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
		//高度自保措施 以后注意修改！TODO
		if(comd_height>height_limited)
		{
			comd_height= height_limited;
		}

		///////////////////////////////////////////选择控制模式：1改进平行接近法 2PID控制 3垂直降落///////////////////////进入land
		PID_control_of_xy=true;
		if(model_choose==1)//////20161224  判断满足降落条件及下降
		{
			printf("r_vt  %f  error[2] %f height %f\n",r_vt,error[2],jrh_drone->ultrasonic.ranges[0]);
			fprintf(model_txt,"r_vt  %f  error[2] %f height %f\n",r_vt,error[2],jrh_drone->ultrasonic.ranges[0]);
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
								error[3]=comd_height-jrh_drone->ultrasonic.ranges[0];
			}
		}
		if(land)
		{
			if(model_choose==1)
			{
				if(time_now-remember_time<parallel_land_time_45_first)////20161224  时间控制
				{
					comd_height=landheight_1;//0.24;
				}
				else
				{
					comd_height=landheight_2;//0.18;
				}
				error[0]=0;
				error[1]=0;
				error[2]=0;
				error[3]=comd_height-jrh_drone->ultrasonic.ranges[0];

				if (tic() - remember_time>parallel_land_time_45_second)////20161224  定时再起飞
				{
					land = false;
					take_off = true;
					model_choose = 88;
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
		printf("yaw_err  %f\n",error[2]);
		fprintf(model_txt,"yaw_err  %f\n",error[2]);
///////////////////////////////////////////	
	////////////////////////////////////////////计算kp项指令大小
		comd_p[0]=error[0]*kp[0];
		comd_p[1]=error[1]*kp[1];
		comd_p[2]=error[2]*kp[2];
		comd_p[3]=error[3]*kp[3];

	////////////////////////////////////////////计算ki项指令大小
		if(dt==0.0f)
		{
			comd_i[0]=0.0f;
			comd_i[1]=0.0f;
			comd_i[2]=0.0f;
			comd_i[3]=0.0f;

		}
		else
		{
			comd_i[0]=(comd_i[0]+error[0]*dt)*ki[0];
			comd_i[1]=(comd_i[1]+error[1]*dt)*ki[1];
			comd_i[2]=(comd_i[2]+error[2]*dt)*ki[2];
			comd_i[3]=(comd_i[3]+error[3]*dt)*ki[3];
		}
	////////////////////////////////////////////计算kd项指令大小
		if(dt==0.0f)
		{
			comd_d[0]=0.0f;
			comd_d[1]=0.0f;
			comd_d[2]=0.0f;
			comd_d[3]=0.0f;

		}
		else
		{
			comd_d[0]=kd[0]*(error[0]-last_error[0])/dt;
			comd_d[1]=kd[1]*(error[1]-last_error[1])/dt;
			comd_d[2]=kd[2]*(error[2]-last_error[2])/dt;
			comd_d[3]=kd[3]*(error[3]-last_error[3])/dt;

		}
	////////////////////////////////////////////整合指令的大小
		printf("PID_control_of_xy %d land %d \n ",PID_control_of_xy,land);
		fprintf(model_txt,"PID_control_of_xy %d land %d  \n ",PID_control_of_xy,land);
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
						comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0];  ////20161225   0.24及目标偏航角啥意思
						comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1];
						//comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0]+0.24*cos(iRobot_theta);  ////20161224   0.24及目标偏航角啥意思
						//comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1]+0.24*sin(iRobot_theta);
					}
					else
					{
						comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0];  ////20161225   0.12及目标偏航角啥意思
						comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1];
						//comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0]+0.12*cos(iRobot_theta);  ////20161224   0.12及目标偏航角啥意思
						//comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1]+0.12*sin(iRobot_theta);
				     }
				}
				else
				{
					comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0];//+0.24*cos(iRobot_theta);
					comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1];//+0.24*sin(iRobot_theta);
				}
				if(fabs(iRobot_theta*180/PI-572.957795)<0.2)   /////20161224  目标偏航角方向错误，不使用
				{
					comd_angle_x=comd_p[0]+comd_i[0]+comd_d[0];///////////////////////////////
					comd_angle_y=comd_p[1]+comd_i[1]+comd_d[1];///////////////////////////////
				}
				comd_yaw_rate=comd_p[2]+comd_i[2]+comd_d[2];
				comd_height_v=comd_p[3]+comd_i[3]+comd_d[3];
				printf("comd_angle_x %f comd_angle_y %f comd_yaw_rate %f comd_height_v %f\n ",comd_angle_x,comd_angle_y,comd_yaw_rate,comd_height_v);
				fprintf(model_txt,"comd_angle_x %f comd_angle_y %f comd_yaw_rate %f comd_height_v %f\n ",comd_angle_x,comd_angle_y,comd_yaw_rate,comd_height_v);

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
				fprintf(model_txt,"goal_comd x %f y%f\n",comd_angle_x,comd_angle_y);

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
				fprintf(model_txt,"[45]PID Start!!\n");
			comd_angle_x=0+velcity_land_45*cos(remember_theta);///////////？！！！！！！！！！！！！！！！！！！！！！
			comd_angle_y=0+velcity_land_45*sin(remember_theta);////////！！！！！！！！！！！！！！！！！！！
			comd_yaw_rate=0;
			comd_height_v=comd_p[3]+comd_i[3]+comd_d[3];
				}
			if(model_choose==2)
			{
				printf("[180]Land Start!!\n");
				fprintf(model_txt,"[180]PID Start!!\n");
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

		last_error[0]=error[0];last_error[1]=error[1];last_error[2]=error[2];last_error[3]=error[3];

		last_time=time_now;


		////////////////////////////////////////////将指令从赛场动坐标系到地理坐标系
		float comd_vx=comd_angle_x*cvmGet(R_init,0,0)+comd_angle_y*cvmGet(R_init,0,1)+comd_height_v*cvmGet(R_init,0,2);
		float comd_vy=comd_angle_x*cvmGet(R_init,1,0)+comd_angle_y*cvmGet(R_init,1,1)+comd_height_v*cvmGet(R_init,1,2);
		float comd_zv=comd_angle_x*cvmGet(R_init,2,0)+comd_angle_y*cvmGet(R_init,2,1)+comd_height_v*cvmGet(R_init,2,2);
	////////////////////////////////////////////发送指令地理坐标系中
		printf("comd_vx %f comd_vy %f comd_vz %f\n",comd_vx,comd_vy,comd_zv);
		fprintf(model_txt,"comd_vx %f comd_vy %f comd_vz %f\n",comd_vx,comd_vy,comd_zv);
		fprintf(output_txt,"time=%f comd_vx=%f comd_vy=%f comd_vz=%f comd_height=%f height_flag=%d comd_angle_x=%f comd_angle_y=%f\n",tic(),comd_vx,comd_vy,comd_zv,comd_height,height_flag,comd_angle_x,comd_angle_y);
		drone->velocity_control(0x49,comd_vx,comd_vy,comd_zv,comd_yaw_rate);
		}
	}
else if(model_choose==0)//跟随模式哦
{
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
		obstacel_amount_total=0;

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


else if(model_choose==88)//如果180度感应成功，则起飞
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

	if(model_choose==88)//model_choose==88的处理：包括紧急起飞处理、飞行中丢目标处理
	{

		  error[3]=take_off_height-jrh_drone->ultrasonic.ranges[0];
			printf("com_height%f\n",take_off_height);
			fprintf(model_txt,"com_height%f",take_off_height);
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


		comd_yaw_rate=0;
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
					  comd_height=landheight_1;//0.24;
				  }
				  else
				  {
					  comd_height=landheight_2;//0.18;
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
