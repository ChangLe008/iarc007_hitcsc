#include <dji_sdk/dji_sdk.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <string>
#include <iarc007/jrh_model_choose.h>

#include <iarc007/jrh_pilot/JRH_math.h>
#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iarc007/jrh_pilot/pilotmain.h>
#include<iarc007/object.h>
#include<iarc007/obstacle.h>
#include<iarc007/vehicle_pos.h>

extern FILE *ultrasonic_original;
extern FILE *ultrasonic_low_pass;
extern FILE *acceleration_txt;
extern FILE *velocity_txt;
extern FILE *velocity_guidance_txt;
extern FILE *guidance_position_txt;


extern CvMat *R_init;

FILE *ultrasonic_original=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/ultrasonic_original.txt","w");
FILE *ultrasonic_low_pass=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/ultrasonic_low_pass.txt","w");
FILE *acceleration_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/acceleration.txt","w");
FILE *velocity_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/velocity.txt","w");
FILE *velocity_guidance_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/velocity_guidance.txt","w");
FILE *guidance_position_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/guidance_position.txt","w");

float wny_px,wny_py,wny_pz;
float wx_px,wx_py,wx_pz;
class YQTObstacle
{
    private:
		//float gloyqt_pos_x = wny_px;
		//float gloyqt_pos_y = wny_py;
		float processyqt_pos_x[4]  = {0.};
		float processyqt_pos_y[4]  =  {0.};
		double processyqt_pos_time  =  0.;
		float cal_processyqt_pos_x[4]  =  {0.};
		float cal_processyqt_pos_y[4]  =  {0.};
		float process_obstacle_distance = 0;
		float differ_obstacle_distance = 0;
		float velocity_yqt = 0.33;


		int obstacle_calculate_count = 0;
		int obstacle_initial_num = 0;
		int i;

    public:
		 iarc007::obstacle calculate_obstacle_yqt;

    private:
		double tic(){
			struct timeval t;
			gettimeofday(&t,NULL);
			return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
		}
    public:
		void calculate_obstacle(iarc007::obstacle &initial_obstacle_yqt)
		{
			//initial_obstacle_yqt.obstacle_x.resize(4);
			//initial_obstacle_yqt.obstacle_y.resize(4);
			printf("obstacle_calculate_count = %d\n",obstacle_calculate_count);
			printf("obstacle_initial_num = %d\n",obstacle_initial_num);
			printf("initial_obstacle_yqt.num = %d\n",initial_obstacle_yqt.num);
			printf("initial_obstacle_yqt.biaozhiwei = %d\n",initial_obstacle_yqt.biaozhiwei[0]);

			if(obstacle_initial_num == 0)
			{
				if(initial_obstacle_yqt.biaozhiwei[0] == 0)
				{
					return;
				}
				else
				{
					for(i =0; i<initial_obstacle_yqt.num;i++)
					{
						processyqt_pos_x[i] = initial_obstacle_yqt.obstacle_x[i];
						processyqt_pos_y[i] = initial_obstacle_yqt.obstacle_y[i];
						//processyqt_pos_x[i] = gloyqt_pos_x + processyqt_pos_x[i];
						//processyqt_pos_y[i] = gloyqt_pos_y + processyqt_pos_y[i];
						processyqt_pos_time = tic();
						printf("$$$$$$$$$$$$$$$$$$$$$$$$$processyqt_pos_time    %f \n",processyqt_pos_time);
						obstacle_calculate_count = initial_obstacle_yqt.num;
					}
				}
			}
			else
			{
				if(initial_obstacle_yqt.biaozhiwei[0] == 0)
				{
					//cal_processyqt_pos_x = processyqt_pos_x - 5.*sin(omega_yqt*(tic()-processyqt_pos_time))*tan((omega_yqt*((tic()-processyqt_pos_time)))/2);
					if(obstacle_calculate_count != 0)
					{
						for(i = 0 ; i < obstacle_calculate_count ; i++ )
						{
							process_obstacle_distance = sqrt((processyqt_pos_x[i]*processyqt_pos_x[i])+(processyqt_pos_y[i]*processyqt_pos_y[i]));
							printf("process_obstacle_distance = %f\n",process_obstacle_distance);
							if(process_obstacle_distance > 0.6)
							{
								cal_processyqt_pos_x[i] = ((process_obstacle_distance - (velocity_yqt*(tic() - processyqt_pos_time)))/process_obstacle_distance)*processyqt_pos_x[i];
								cal_processyqt_pos_y[i] = ((process_obstacle_distance - (velocity_yqt*(tic() - processyqt_pos_time)))/process_obstacle_distance)*processyqt_pos_y[i];

								printf("tic() = %f\n",tic() );
								printf("processyqt_pos_time = %f\n", processyqt_pos_time );
								printf("tic() - processyqt_pos_time[i] = %f\n",tic() - processyqt_pos_time);
								processyqt_pos_time = tic();
								processyqt_pos_x[i] = cal_processyqt_pos_x[i];
								processyqt_pos_y[i] = cal_processyqt_pos_y[i];
							}
							else
							{
								obstacle_calculate_count= obstacle_calculate_count - 1;
								if(obstacle_calculate_count < 0)
								{
									obstacle_calculate_count = 0;
								}
							}
						}
						initial_obstacle_yqt.num = obstacle_calculate_count;
						initial_obstacle_yqt.biaozhiwei[0] = 1;
					}
				}
				else
				{
					if(initial_obstacle_yqt.num < obstacle_calculate_count)
					{
						for(i =0; i<obstacle_calculate_count;i++)
						{
							if(i < initial_obstacle_yqt.num)
							{
								processyqt_pos_x[i] = initial_obstacle_yqt.obstacle_x[i];
								processyqt_pos_y[i] = initial_obstacle_yqt.obstacle_y[i];
							}
							else
							{
								process_obstacle_distance = sqrt((processyqt_pos_x[i]*processyqt_pos_x[i])+(processyqt_pos_y[i]*processyqt_pos_y[i]));
								printf("process_obstacle_distance = %f\n",process_obstacle_distance);
								if(process_obstacle_distance > 0.6)
								{
									cal_processyqt_pos_x[i] = ((process_obstacle_distance - (velocity_yqt*(tic() - processyqt_pos_time)))/process_obstacle_distance)*processyqt_pos_x[i];
									cal_processyqt_pos_y[i] = ((process_obstacle_distance - (velocity_yqt*(tic() - processyqt_pos_time)))/process_obstacle_distance)*processyqt_pos_y[i];
									printf("tic() = %f\n",tic() );
									printf("processyqt_pos_time = %f\n", processyqt_pos_time );
									printf("tic() - processyqt_pos_time[i] = %f\n",tic() - processyqt_pos_time);
									processyqt_pos_time = tic();
									processyqt_pos_x[i] = cal_processyqt_pos_x[i];
									processyqt_pos_y[i] = cal_processyqt_pos_y[i];
								}
								else
								{
									obstacle_calculate_count= obstacle_calculate_count - 1;
									if(obstacle_calculate_count < 0)
									{
										obstacle_calculate_count = 0;
									}
								}
							}
						}
						initial_obstacle_yqt.num = obstacle_calculate_count;
					}
					else
					{
						for(i =0; i<initial_obstacle_yqt.num;i++)
						{
							processyqt_pos_x[i] = initial_obstacle_yqt.obstacle_x[i];
							processyqt_pos_y[i] = initial_obstacle_yqt.obstacle_y[i];
						}
					}
				}
			}
			for(i = 0 ; i < initial_obstacle_yqt.num ; i++ )
			{
			 	 initial_obstacle_yqt.obstacle_x[i] = processyqt_pos_x[i];
			 	 initial_obstacle_yqt.obstacle_y[i] = processyqt_pos_y[i];
			}
			obstacle_initial_num++;
		}
};
YQTObstacle forecast_yqt;
class JRHDrone
{
private:
	 ros::Subscriber ultrasonic_subscriber;
	 ros::Subscriber acceleration_subscriber;
	 ros::Subscriber attitude_quaternion_subscriber;
     ros::Subscriber rc_channels_subscriber;
     ros::Subscriber velocity_subscriber;
     ros::Subscriber activation_subscriber;
	 ros::Subscriber sdk_permission_subscriber;
	 ros::Subscriber time_stamp_subscriber;
	 ros::Subscriber guidance_pos_subscriber;
	ros::Subscriber velocity_guidance_subscriber;
	ros::Subscriber object_subscriber;
	ros::Subscriber obstacle_subscriber;
	ros::Subscriber obstacle_yqt_subscriber;
	ros::Subscriber vehicle_pos;
	ros::Subscriber obstacle_bottom_num_subscriber;
    //ros::Subscriber power_status_subscriber;
    //ros::Publisher obstacle_pub = my_node.advertise<iarc007::obstacle>("yqt_obstacle",10);
public:
	ros::Publisher jrh_model_choose_publisher;
	sensor_msgs::LaserScan ultrasonic;
	dji_sdk::Acceleration acceleration;
	dji_sdk::AttitudeQuaternion attitude_quaternion;
	dji_sdk::RCChannels rc_channels;
	dji_sdk::Velocity velocity;
    dji_sdk::TimeStamp time_stamp;
    geometry_msgs::Vector3Stamped guidance_pos;
	geometry_msgs::Vector3Stamped velocity_guidance;
	iarc007::object object_pos_dynamic;
	iarc007::obstacle obstacle_pos_dynamic;
	iarc007::obstacle obstacle_pos_dynamic_yqt;
	iarc007::obstacle obstacle_bottom_num;
	iarc007::jrh_model_choose jrh_model_choose;
    //dji_sdk::PowerStatus power_status;
    bool sdk_permission_opened = false;
    bool activation = false;


private:
	    void ultrasonic_subscriber_callback(sensor_msgs::LaserScan g_ul)
	    {
	    	ultrasonic.ranges.resize(5);
	    	this->ultrasonic = g_ul;

	    	//printf("guidanceheight:        %f  \n",ultrasonic.ranges[0]);
	    	fprintf(ultrasonic_original,"time  %f  ultrasonic1  %f \n",ultrasonic.header.stamp.sec+ultrasonic.header.stamp.nsec*(1e-9),ultrasonic.ranges[0]);
	    	float original_ultrasonic_data=ultrasonic.ranges[0];
	    	float ultrasonic_low_pass_data=Low_Pass(ultrasonic.ranges[0]);
	    	ultrasonic.ranges[0]=ultrasonic_low_pass_data;
	    	ultrasonic.ranges[1]=original_ultrasonic_data;
	    	fprintf(ultrasonic_low_pass,"time  %f  ultrasonic1  %f \n",ultrasonic.header.stamp.sec+ultrasonic.header.stamp.nsec*(1e-9),ultrasonic_low_pass_data);
	    }


	    void guidance_pos_subscriber_callback( geometry_msgs::Vector3Stamped g_pos)
	    {
	    	this->guidance_pos=g_pos;
	    	fprintf(guidance_position_txt,"time  %f    x   %f   y   %f  z   %f\n",g_pos.header.stamp.sec+g_pos.header.stamp.nsec*(1e-9),g_pos.vector.x,g_pos.vector.y,g_pos.vector.z);
	    //	printf("time  %f    x   %f   y   %f  z   %f\n",g_pos.header.stamp.sec+g_pos.header.stamp.nsec*(1e-9),g_pos.vector.x,g_pos.vector.y,g_pos.vector.z);
	    }

	    void acceleration_subscriber_callback(dji_sdk::Acceleration acceleration)
		{
			this->acceleration = acceleration;
			fprintf(acceleration_txt,"time  %f  ax  %f  ay %f  az  %f \n",ultrasonic.header.stamp.sec+ultrasonic.header.stamp.nsec*(1e-9),acceleration.ax,acceleration.ay,acceleration.az);
		}

		void attitude_quaternion_subscriber_callback(dji_sdk::AttitudeQuaternion attitude_quaternion)
		{
			this->attitude_quaternion = attitude_quaternion;
			//printf("q0  %f   q1   %f   q2   %f   q3   %f",attitude_quaternion.q0,attitude_quaternion.q1,attitude_quaternion.q2,attitude_quaternion.q3);
		}

		void rc_channels_subscriber_callback(dji_sdk::RCChannels rc_channels)
		{
			this->rc_channels = rc_channels;
		}

		void velocity_subscriber_callback(dji_sdk::Velocity velocity)
		{
			this->velocity = velocity;
			float vx=cvmGet(R_init,0,0)*velocity.vx+cvmGet(R_init,0,1)*velocity.vy+cvmGet(R_init,0,2)*velocity.vz;
			float vy=cvmGet(R_init,1,0)*velocity.vx+cvmGet(R_init,1,1)*velocity.vy+cvmGet(R_init,1,2)*velocity.vz;
			float vz=cvmGet(R_init,2,0)*velocity.vx+cvmGet(R_init,2,1)*velocity.vy+cvmGet(R_init,2,2)*velocity.vz;
			velocity.vx=vx;
			velocity.vy=vy;
			velocity.vz=vz;
			fprintf(velocity_txt,"time   %d     vx    %f   vy   %f     vz    %f \n",velocity.ts,vx,vy,vz);
			//printf("%f     %f    %f\n",vx,vy,vz);

		}

		void activation_subscriber_callback(std_msgs::UInt8 activation)
		{
			this->activation = activation.data;
		}


		void sdk_permission_subscriber_callback(std_msgs::UInt8 sdk_permission)
		{
			this->sdk_permission_opened = sdk_permission.data;
		}

		void time_stamp_subscriber_callback(dji_sdk::TimeStamp time_stamp)
		{
			this->time_stamp = time_stamp;
		}

		void velocity_guidance_subscriber_callback(geometry_msgs::Vector3Stamped g_vo)
		{
			this->velocity_guidance=g_vo;
			float vx_guidance=cvmGet(R_init,0,0)*g_vo.vector.x+cvmGet(R_init,0,1)*g_vo.vector.y+cvmGet(R_init,0,2)*g_vo.vector.z;
			float vy_guidance=cvmGet(R_init,1,0)*g_vo.vector.x+cvmGet(R_init,1,1)*g_vo.vector.y+cvmGet(R_init,1,2)*g_vo.vector.z;
			float vz_guidance=cvmGet(R_init,2,0)*g_vo.vector.x+cvmGet(R_init,2,1)*g_vo.vector.y+cvmGet(R_init,2,2)*g_vo.vector.z;
			fprintf(velocity_guidance_txt,"time   %f     vx    %f   vy   %f     vz    %f \n",g_vo.header.stamp.sec+g_vo.header.stamp.nsec*(1e-9),vx_guidance,vy_guidance,vz_guidance);
			//printf("%f     %f    %f\n",vx_guidance,vy_guidance,vz_guidance);

		}
		void object_subscriber_callback(iarc007::object object_pos_dynamic)
		{
	this->object_pos_dynamic=object_pos_dynamic;
			object_pos_dynamic.target_x.resize(20);
			object_pos_dynamic.target_y.resize(20);
			object_pos_dynamic.target_dir.resize(20);	
//printf("x_t %f  y_t %f  theta_t  %f  num  %d\n",object_pos_dynamic.target_x[0]*0.01,object_pos_dynamic.target_y[0]*0.01,object_pos_dynamic.target_dir[0],object_pos_dynamic.target_num);
		}

		void obstacle_subscriber_callback(iarc007::obstacle obtacle_pos_dynamic)
		{
			//forecast_yqt.calculate_obstacle(obtacle_pos_dynamic);
	this->obstacle_pos_dynamic=obtacle_pos_dynamic;
	//printf("udfhialdniulaghdfuidsudhfguilUJV\n");
	obstacle_pos_dynamic.obstacle_x.resize(4);
	obstacle_pos_dynamic.obstacle_y.resize(4);
//printf("x_t %f  y_t %f  theta_t  %f  num  %d\n",object_pos_dynamic.target_x[0]*0.01,object_pos_dynamic.target_y[0]*0.01,object_pos_dynamic.target_dir[0],object_pos_dynamic.target_num);
		}

		void obstacle_yqt_subscriber_callback(iarc007::obstacle obtacle_pos_dynamic_yqt)
		{
			//forecast_yqt.calculate_obstacle(obtacle_pos_dynamic_yqt);
	this->obstacle_pos_dynamic_yqt=obtacle_pos_dynamic_yqt;
	obstacle_pos_dynamic_yqt.obstacle_x.resize(4);
	obstacle_pos_dynamic_yqt.obstacle_y.resize(4);
		}
		void vehicle_pos_callback(iarc007::vehicle_pos wny)
		{

			CvMat* pos_mat = cvCreateMat(3,3,CV_32FC1);

			cvmSet(pos_mat, 0, 0, wny.pos.x);
			cvmSet(pos_mat, 1, 0, wny.pos.y);
			cvmSet(pos_mat, 2, 0, wny.pos.z);

			printf("wny.pos.x====%f\n",wny.pos.x);
			printf("wny.pos.y====%f\n",wny.pos.y);	

			wny_px = cvmGet(pos_mat, 0, 0);
			wny_py = cvmGet(pos_mat, 1, 0);
			wny_pz = cvmGet(pos_mat, 2, 0);

			wx_px = cvmGet(pos_mat, 0, 0);
			wx_py = cvmGet(pos_mat, 1, 0);
			wx_pz = cvmGet(pos_mat, 2, 0);
			cvReleaseMat(&pos_mat);
		}


		void obstacle_bottom_num_subscriber_callback(iarc007::obstacle obstacle_bottom_num)
		{
			this->obstacle_bottom_num=obstacle_bottom_num;
		}
		/*
		void power_status_subscriber_callback(dji_sdk::PowerStatus power_status)
		{
			this->power_status = power_status;
		}
*/
public:
	JRHDrone(ros::NodeHandle& nh)
	{
			    ultrasonic_subscriber = nh.subscribe<sensor_msgs::LaserScan>("/guidance/ultrasonic", 1, &JRHDrone::ultrasonic_subscriber_callback, this);
		 	 	acceleration_subscriber = nh.subscribe<dji_sdk::Acceleration>("dji_sdk/acceleration", 10, &JRHDrone::acceleration_subscriber_callback, this);
		        attitude_quaternion_subscriber = nh.subscribe<dji_sdk::AttitudeQuaternion>("dji_sdk/attitude_quaternion", 10, &JRHDrone::attitude_quaternion_subscriber_callback, this);
		        rc_channels_subscriber = nh.subscribe<dji_sdk::RCChannels>("dji_sdk/rc_channels", 10, &JRHDrone::rc_channels_subscriber_callback, this);
		        velocity_subscriber = nh.subscribe<dji_sdk::Velocity>("dji_sdk/velocity", 10, &JRHDrone::velocity_subscriber_callback, this);
		        activation_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/activation", 10, &JRHDrone::activation_subscriber_callback, this);
		        sdk_permission_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/sdk_permission", 10, &JRHDrone::sdk_permission_subscriber_callback, this);
				time_stamp_subscriber = nh.subscribe<dji_sdk::TimeStamp>("dji_sdk/time_stamp", 10, &JRHDrone::time_stamp_subscriber_callback,this);
				guidance_pos_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>("/guidance/position", 1, &JRHDrone::guidance_pos_subscriber_callback,this);
			    velocity_guidance_subscriber= nh.subscribe<geometry_msgs::Vector3Stamped>("/guidance/velocity", 1, &JRHDrone::velocity_guidance_subscriber_callback,this);
			    object_subscriber=nh.subscribe<iarc007::object>("/hy_object", 1, &JRHDrone::object_subscriber_callback,this);
			    obstacle_subscriber=nh.subscribe<iarc007::obstacle>("/zy_obstacle", 1, &JRHDrone::obstacle_subscriber_callback,this);
			    obstacle_yqt_subscriber=nh.subscribe<iarc007::obstacle>("/yqt_obstacle", 1, &JRHDrone::obstacle_yqt_subscriber_callback,this);
			    obstacle_bottom_num_subscriber=nh.subscribe<iarc007::obstacle>("/yqt_obstacle_under", 1, &JRHDrone::obstacle_bottom_num_subscriber_callback,this);
			    vehicle_pos=nh.subscribe<iarc007::vehicle_pos>("iarc007/vehicle_pos",1,&JRHDrone::vehicle_pos_callback,this);
	            jrh_model_choose_publisher=nh.advertise<iarc007::jrh_model_choose>("/jrh_model_choose",10);
	            //power_status_subscriber = nh.subscribe<dji_sdk::PowerStatus>("dji_sdk/power_status", 10, &DJIDrone::power_status_subscriber_callback, this);
	}
};
 
