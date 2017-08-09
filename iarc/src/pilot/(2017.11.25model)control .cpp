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
target_info raw_info;               // raw target information from hy
target_info info;                   // filered target information
int lost_target_counter = 0;        // if target is lost, continue the previous routine for 10 frames
float target_dir = 0.;
// target variables ends


// 180 interact variables ends

float timer_180_1=0.;               // timer_1 for 180 interact
float timer_180_2=0.;               // timer_2 for 180 interact
int stage_180=0;                    // 
int counter_180_2=0;
// 180 interact variables ends

// 45 interact variables ends

float timer_45_1=0.;                // timer_1 for 45 interact
float timer_45_2=0.;                // timer_2 for 45 interact
int stage_45=0;                     // 
int counter_45_2=0;
// 45 interact variables ends

//main strategy variables ends
int mode_45_count=0;
int mode_180_count=0;

//main strategy variables ends


// CvMat *q_right_now=cvCreateMat(4,1,CV_32FC1);

// CvMat *q_init=cvCreateMat(4,1,CV_32FC1);
// CvMat *att_init=cvCreateMat(3,1,CV_32FC1);
// CvMat *R_init=cvCreateMat(3,3,CV_32FC1);
// CvMat *R_init_i=cvCreateMat(3,3,CV_32FC1);

// CvMat *q=cvCreateMat(4,1,CV_32FC1);
// CvMat *atti=cvCreateMat(3,1,CV_32FC1);
// CvMat *R_now_i=cvCreateMat(3,3,CV_32FC1);

// CvMat *att_right_now=cvCreateMat(3,1,CV_32FC1);

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
_patrol patrolr={0,0};

unsigned int period = 0;


/***
test flags
***/
bool fisrt_time=1;
float tar_x=0.,tar_y=0.;


float dx=0;
float dy=0;
float tar_dx=0;
float tar_dy=0;
float time0000=0.;

float obs_dx = 0;
float obs_dy = 0;
float height_quad = 1.2;
bool fisrtshift=1;
/*******
test flag ends
*******/

int decision(int distance)
{
    cout<<distance<<endl;
    return 0;
}
int patrol_stage=0;
double patrol_time=0.;
iarc::vehicle_pos position;


pid_ctrl pid_x,pid_y,pid_z,pid_yaw,pid_pitch,pid_roll;
target_processor t_p;
//pid_ctrl




void wny_45()
{
    return;
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


void ascend( float current_height, int ascend_mode,float direction=-1000)
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
            velocity_arena.vh     = pid_z.out(1.3,current_height,0);
            velocity_arena.vx     = 0.;
            velocity_arena.vy     = 0.;
            velocity_arena.dyaw   = 0.;
            attitude_control = false;
            break;
        }

        case 2:
        {
            velocity_arena.vh = 0.1*(1.3-current_height);
            if(direction>-1000)
            {
                velocity_arena.vx     = 0.;
                velocity_arena.vy     = 0.;
            }
            else
            {
                velocity_arena.vx     = 0.33*cos(direction);
                velocity_arena.vy     = 0.33*sin(direction);
            }
            velocity_arena.dyaw   = 0.;
            cout << "vvh="<<velocity_arena.vh;
            attitude_control = false;
            break;
        }

        case 3:
        {
            if(1.3-current_height>=0.5)
                velocity_command.vh = 0.3*(1.3-current_height);
            else
                velocity_command.vh = 0.4*(1.3-current_height);
            if(direction>-1000)
            {
                velocity_command.vx     = 0.;
                velocity_command.vy     = 0.;
            }
            else
            {
                velocity_command.vx     = -0.33*cos(direction);
                velocity_command.vy     = -0.33*sin(direction);
            }
            velocity_command.dyaw   = 0.;
            attitude_control = false;
            break;
        }
        
        case 4:
        {
            velocity_command.vh     = pid_z.out(0.9,current_height,0);
            velocity_command.vx     = 0.;
            velocity_command.vy     = 0.;
            velocity_command.dyaw   = 0.;
            cout << "vvh="<<velocity_command.vh;
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


void track(float x,float y,float vx_r,float vy_r,float h,float h_ideal,bool att_track,float direction)
{
    if(att_track) // Attitude control
    {/*
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        attitude_control=true;
        attitude_command.dyaw   = 0.;
        velocity_arena.vh       = pid_z.out(h_ideal,h,0);
        velocity_arena.vx       = pid_x.out(0,x,1);
        velocity_arena.vy       = pid_y.out(0,y,1); 
        velocity_arena.vx       = velocity_arena.vx;
        velocity_arena.vy       = velocity_arena.vy; 
        if(period%2==0)
        {
             attitude_arena.pitch =   pid_pitch.out(velocity_arena.vx,vx_r,0);          // velocity from vision
             attitude_arena.roll  =   pid_roll.out(velocity_arena.vy,vy_r,0);           //当前速度可以用1.guidance读取2.位置差分
        }*/
    }
    else//输出速度指令
    {
        
        velocity_arena.dyaw   = 0.;
        velocity_arena.vx       = pid_x.out(0,x,1);
        velocity_arena.vy       = pid_y.out(0,y,1);
        velocity_arena.vh       = pid_z.out(h_ideal,h,0);
        attitude_control = false;
    }

}



void interact_180_noobs( float x,float y,float h,float h_ideal,float direction )
{
    attitude_control=false;
    cout << h-h_ideal << "\t" << sqrt(x*x+y*y) << endl;
    if(h>=h_ideal || sqrt(x*x+y*y) >= 0.1 ) // Attitude control
    {
        velocity_arena.dyaw     = 0.;
        velocity_arena.vh       = pid_z.out(h_ideal,h,0);
        velocity_arena.vx       = pid_x.out(0,x,1);
        velocity_arena.vy       = pid_y.out(0,y,1);
        velocity_arena.vy       = -1*velocity_arena.vy;
        velocity_arena.vx       = -1*velocity_arena.vx;
        cout <<"vx"<< velocity_arena.vx <<"\t"<<"vx"<< velocity_arena.vy << endl;
        stage_180               = 1;
    }
    else
    {
        cout << "stage 1 ends"<< endl;

        stage_180               = 2;
        timer_180_1             = tic();
        //pid_z.clean();
    }
}


void interact_180_noobs(float h,float h_ideal,float direction )
{
    counter_180_2++;
    if(h>=h_ideal && stage_180==2 && counter_180_2<=75)
    {
        velocity_arena.dyaw     = 0.;
        velocity_arena.vh       = pid_z.out(h_ideal,h,0);
        velocity_arena.vx       = 0.3*cos(direction);
        velocity_arena.vy       = 0.3*sin(direction); 
    }
    else 
    {
        if(stage_180==2 && counter_180_2>=150)
        {
            global_status       = 4;
            control_mode        = 5;
            stage_180           = 0;
            counter_180_2       = 0;

        }
        velocity_arena.dyaw     = 0.;
        velocity_arena.vh       = pid_z.out(0.2,h,0);
        velocity_arena.vx       = pid_x.out(0,0,1);
        velocity_arena.vy       = pid_y.out(0,0,1); 
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

void test_att(float dx, float dy,float h,float h_ideal)
{
        
        attitude_control=true;
        attitude_command.roll=0.;
        attitude_command.pitch=0.;
        attitude_command.vh=pid_z.out(h_ideal,h,0);
        attitude_command.dyaw=0.;
}

void control()
{
    int timeee=0;
    int landmode=0;
    time_t nowtime;
    nowtime = time(NULL);
    ostringstream oss;
    tm *ltm = localtime(&nowtime);
    oss << ltm->tm_mon+1 << "月"<< ltm->tm_mday << "日"<< ltm->tm_hour << "时"<<ltm->tm_min;
    string path = "/home/hitcsc/catkin_ws/log/iarc/pilot/" + oss.str();//string(ctime(&nowtime));
    string file = path + "/control.txt";
    string temp_file = path + "/height.txt";
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
    ros::NodeHandle pilot;
    DJIDrone* drone = new DJIDrone(pilot);
    LEODrone* wsq_drone=new LEODrone(pilot);
    float time0 = tic();
    pid_x.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidx.txt");
    pid_y.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidy.txt");
    pid_z.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidz.txt");
    pid_pitch.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidpitch.txt");
    pid_roll.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidroll.txt");    
    pid_yaw.open((char *)"/home/hitcsc/catkin_ws/src/iarc/cfg/pidyaw.txt");
    t_p.clean();
    control_mode = -1;



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

    // test variables starts
        int target_0=0;
        int target_1=0;
        int target_2=0;
    // test variables ends
    control_mode=3;
    ros::Rate rate(50);
    while(ros::ok())
    {
        //ROS_INFO("Here comes the control part");
        
        //1.If controller is not in F mode, release contorl and exit the control program.
        ros::spinOnce();
        while((abs(drone->rc_channels.mode+8000.0)<1)||abs(drone->rc_channels.mode)<1)
        {
            ros::spinOnce();
            drone->release_sdk_permission_control();
            ROS_INFO("Release control.");
            control_log.close();
            height_log.close();
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
        // test use only begins
        target_2 = target_1;
        target_1 = target_0;
        target_0 = wsq_drone->object_pos_dynamic.target_num;
        int target_number;
        int obstacle_number;
        int tar_dir_x=0;
        int tar_dir_y=0;
        if(target_2>=1 && target_0>=1 && target_1>=1)
        {
            target_number = target_0;
        }
        else
        {
            target_2 = 0;
            target_1 = 0;
            target_0 = 0;
            target_number = 0;
        }
        // test use only ends

        
        target_number = wsq_drone->target_dynamic.target.target_num;
        wsq_drone->target_dynamic.target_x.resize(target_number>0?target_number:2);
        wsq_drone->target_dynamic.target_y.resize(target_number>0?target_number:2);
        wsq_drone->target_dynamic.target_dir_x.resize(target_number>0?target_number:2);
        wsq_drone->target_dynamic.target_dir_y.resize(target_number>0?target_number:2);
        wsq_drone->ultrasonic.ranges.resize(2);
        obstacle_number = wsq_drone->obstacle_dynamic.num;
        wsq_drone->obstacle_dynamic.o_x.resize(1);
        wsq_drone->obstacle_dynamic.o_y.resize(1);
        height_quad = wsq_drone->ultrasonic.ranges[1];

        if(target_number>=1)
        {
            dx = wsq_drone->target_dynamic.target_x[0];
            dy = wsq_drone->target_dynamic.target_y[0];
            tar_x = wsq_drone->target_dynamic.target_dir_x[0];
            tar_y = wsq_drone->target_dynamic.target_dir_y[0];
        }
        if(obstacle_number>=1)
        {
            obs_dx = wsq_drone->obstacle_dynamic.o_x[0];
            obs_dy = wsq_drone->obstacle_dynamic.o_y[0];
        }


        /// Leo strategy recommended
        // cout <<"tard" <<tar_x <<"\t" << tar_y << "\t";
        // cout <<"obs" <<obs_dx <<"\t" << obs_dy << "\t";
        // cout << "x "<<dx <<" y "<<dy<<endl;       
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
        if(target_number==1 && control_mode==4)//from patrol to 45 degree
        {
             control_mode=1;
        }
        if(target_number==1 && wsq_drone->ultrasonic.ranges[1] >=1.25 && control_mode==5)//from patrol to 45 degree
        {
             ;///control_mode=2;
        }
        if(target_number==1 && obstacle_number>=1 )
        {
            if(sqrt((obs_dx-dx)*(obs_dx-dx)+(obs_dy-dy)*(obs_dy-dy))<=1.3)
            {
                    if(sqrt((obs_dx-dx)*(obs_dx-dx)+(obs_dy-dy)*(obs_dy-dy))<=1.2 && control_mode==1 && fisrtshift)
                    {
                        control_mode=6;
                    }
                    else
                    {
                        control_mode=1;
                        fisrtshift = 0;
                    }
            } 
            else
            {
                control_mode=1;
            }
            //cout << sqrt((obs_dx-dx)*(obs_dx-dx)+(obs_dy-dy)*(obs_dy-dy)) << " tar obs dis";
        }
        /*main strategy*/


        /*main strategy(second try: circle strategy)*/
        
        /*main strategy*/

        control_mode=3;
        switch (control_mode)
        {
            case 1:     //45 degree
            {
                if(pid_flag)
                {
                    //cout << stage_45 <<"\t";
                    if( ( stage_45==0 || stage_45==1 ))
                    {
                        float height_45=wsq_drone->ultrasonic.ranges[1];
                        if(height_45==0)
                        {
                            height_45=1.2;
                        }
                        stage_45=1;
                        float cmd_height = sqrt(dx*dx+dy*dy)+0.1;
                        interact_45_noobs(dx,dy,height_45,cmd_height,0,tar_x,tar_y);
                        
                        global_status   = 2;
                        target_dir = info.dir;
                    }
                    else if(stage_45==2)
                    {
                        // interact_45_noobs(wsq_drone->ultrasonic.ranges[1],0.20,target_dir);
                        float height_45=wsq_drone->ultrasonic.ranges[1];
                        //interact_45_noobs(dx,dy,height_45,0.15,0);
                        interact_45_noobs(height_45,0.12,0.,tar_x,tar_y);
                        //interact_45_noobs(dx,dy,wsq_drone->ultrasonic.ranges[1],1,0);
                        cout << "test failed 3"<< endl;

                    }
                    else
                    {
                        control_mode = 5;
                        cout << "test failed 2"<< endl;
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
                    //target_number>=1 &&
                    cout << stage_180 <<"\t";
                    if( ( stage_180==0 || stage_180==1 ))
                    {
                        float height_180=wsq_drone->ultrasonic.ranges[1];
                        if(height_180==0)
                        {
                            height_180=1.2;
                        }

                        interact_180_noobs(-dx,-dy,height_180,1,0);
                        global_status   = 2;
                        target_dir = info.dir;
                    }
                    else if(stage_180==2)
                    {
                        interact_180_noobs(wsq_drone->ultrasonic.ranges[1],0.5,target_dir);
                        cout << "test failed 3"<< endl;

                    }
                    else
                    {
                        control_mode = 5;
                        cout << "test failed 2"<< endl;
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
            case 3:     // track 
            {              
                float height_test=1.0;
               cout << "test1"<< endl;
                if(target_number==1 && landmode==0)
                {
                    fisrt_time = 1;      
                    cout << "test2"<< endl;  
                    //cout << <<tic()-time0 <<"\t"<< target_number<<"\t"<<info.x<<"\t"<<info.y<<"\t"<<info.vx<<"\t"<<info.vy<<"\t" <<endl;           
                    control_log<<tic()-time0 <<"\t"<< target_number<<"\t"<<-dx<<"\t"<<-dy<<"\t"<<"\t";
                    if(0)//(!trust_guidance)
                    {
                        track(info.x,info.y,info.vx,info.vy,wsq_drone->ultrasonic.ranges[1],height_test,1,0.);  // attitude track
                    }
                    else
                    {
                        track(-dx,-dy,0.,0.,wsq_drone->ultrasonic.ranges[1],height_test,0,0.);              // Velocity track
                    }
                }
                
                else
                {
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
                        control_log<<tic()-time0 <<"\t"<< target_number<<"\t"<<0<<"\t"<<0<<"\t";
                    }
                    
                }
                
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
                    track(-tar_dx,-tar_dy,0.,0.,wsq_drone->ultrasonic.ranges[1],height_test,0,0.);
                    if(tic()-time0-time0000>2)
                        landmode=2;
                    
                }
                if(landmode==2)
                {
                    height_test=0.2;
                    track(0,0,0.,0.,wsq_drone->ultrasonic.ranges[1],height_test,0,0.);
                }
                control_log<<tic()-time0 <<"\t"<< time0000<<"\t"<< landmode<<"\t";
                attitude_control = 0; 
                break;
            }

            case 4:     // patrol (for line strategy)
            {
                /*if(patrol_random)
                {
                    patrol_r(1,1,0.5,0.5,2.15-wsq_drone->local_position.z);
                }
                else
                {
                    int bb=patrol(wsq_drone->velocity,wsq_drone->attitude_quaternion,position,2.14);
                    if(bb==1)
                    {
                        return;
                    }
                }                
                break;*/
                velocity_arena.vx = 0;
                velocity_arena.vy = -0.2;
                velocity_arena.dyaw = 0;
                attitude_command.vh = pid_z.out(1.3,wsq_drone->ultrasonic.ranges[1],0);
                break;
            }

            case 5:     // ascend
            {
                ascend(wsq_drone->ultrasonic.ranges[1],1);
                /*
                if(target_number==1)
                {
                    control_mode = 3;
                }*/
                control_log<<tic()-time0 <<"\t"<< target_number<<"\t"<<0<<"\t"<<0<<"\t";
                break;
            }
            case 6:
            {
                float cmd_height = sqrt(dx*dx+dy*dy)+0.2;
                safe_patrol(dx,dy,obs_dx,obs_dy,height_quad,cmd_height);
                break;
            }
        }

        //4. Send the control command to M100.
        if(attitude_control)
        {/*
            
            // TODO
            // test only
            //attitude_arena.pitch   = 0.0;
            //attitude_arena.roll    = 1.0;
            //attitude_arena.vh      = 0.;
            //attitude_arena.dyaw    = 0.;
            // test only

            attitude_ground.pitch  = -1*(attitude_arena.pitch*cvmGet(R_init,0,0)+attitude_arena.roll*cvmGet(R_init,0,1));
            attitude_ground.roll   = -1*(attitude_arena.pitch*cvmGet(R_init,1,0)+attitude_arena.roll*cvmGet(R_init,1,1));

            attitude_command.pitch  = 0.0+attitude_ground.pitch*cvmGet(R_now_i,0,0)+attitude_ground.roll*cvmGet(R_now_i,0,1);   // -0.7 is the pitch compensate 
            attitude_command.roll   = -0.+attitude_ground.pitch*cvmGet(R_now_i,1,0)+attitude_ground.roll*cvmGet(R_now_i,1,1);  // -0.7 is the roll compensate 
            attitude_command.dyaw   = attitude_arena.dyaw;
            attitude_command.vh     = attitude_arena.vh;

            /*case 2: 去掉目标出现第一帧的相对速度读取

            drone->attitude_control(    Flight::HorizontalLogic::HORIZONTAL_ANGLE|
									    Flight::VerticalLogic::VERTICAL_VELOCITY |
									    Flight::YawLogic::YAW_PALSTANCE |
									    Flight::HorizontalCoordinate::HORIZONTAL_BODY |	
									    Flight::SmoothMode::SMOOTH_DISABLE,
									    -1*attitude_command.roll, attitude_command.pitch, attitude_command.vh, attitude_command.dyaw);
       */ }
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
            control_log<<setiosflags(ios::fixed)<<attitude_command.roll<<"\t"<< attitude_command.pitch<<"\t"<<attitude_command.vh<<"\t"<<attitude_command.dyaw<<"\t"<<-cvmGet(att_init,0,0)+cvmGet(att_right_now,0,0)<<"\t"<<-cvmGet(att_init,1,0)+cvmGet(att_right_now,1,0)<<"\t"<<-cvmGet(att_init,2,0)+cvmGet(att_right_now,2,0)<<"\t"<<velocity_arena.vx<<"\t"<<velocity_arena.vy<<endl;
            //cout << "attitude"<<attitude_command.roll<<"\t"<<-1*attitude_command.pitch<<"\t"<<velocity_arena.vx<<"\t"<<velocity_arena.vy<<endl;
        }
        else
        {
            control_log<<velocity_arena.vx    <<"\t"<<velocity_arena.vy <<"\t"<<velocity_arena.vh <<"\t"<<velocity_command.dyaw<<"\t"<<wsq_drone->ultrasonic.ranges[1]<<"\t"<<"1.3"<<"\t"<<"0.25"<<"\t"<<endl;
             cout <<"velocity "<<control_mode<<"\t"<<stage_45<<endl;//<<"\t"<<velocity_command.vx    <<"\t"<<velocity_command.vy <<"\t"<<velocity_command.vh <<"\t"<<velocity_command.dyaw<<"\t"<<wsq_drone->local_position.z<<endl;
        }
        wsq_drone->ultrasonic.ranges.resize(2);
        float heyyougay = wsq_drone->ultrasonic.ranges[1];
        height_log << heyyougay <<"\t" << wsq_drone->local_position.z<< endl;
        // Modifing the history data
        if(control_mode!=control_mode_previous)
        {
            pid_x.clean();
            pid_y.clean();
            pid_z.clean();
            pid_yaw.clean();
            pid_pitch.clean();
            pid_roll.clean();
            cout<< "Clean the integral and history data of pid when the mode is changed"<<endl;
        }
        control_mode_previous = control_mode;

        // Modification done

        rate.sleep();
    }
}