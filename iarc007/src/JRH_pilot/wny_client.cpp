#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <dji_sdk/dji_drone.h>
#include <time.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/Twist.h>//position
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <memory.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include <opencv/cv.h>

#include <sensor_msgs/Range.h>

#include <iarc007/vehicle_pos.h>
#include <iarc007/obstacle.h>

using namespace DJI::onboardSDK;
using namespace std;


CvMat* q_att;
CvMat* att;
CvMat* R_g_b;

float px,py,pz;
float px0,py0,pz0;

struct obstacle
{
	float x[4],y[4];
	int num;
	int flag;
};

struct obstacle obstacle;

void Quaternion_To_Euler(CvMat *q_att, CvMat *att)
{
		float r11,r12,r21,r31,r32,r1,r2,r3;
		float q[4] = { cvmGet(q_att,0,0),  cvmGet(q_att,1,0), cvmGet(q_att,2,0), cvmGet(q_att,3,0)};
    	r11 = 2.0f *(q[1] * q[2] + q[0] * q[3]);
    	r12 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2]  - q[3] * q[3] ;
    	r21 = -2.0f * (q[1] * q[3] - q[0] * q[2]);
    	r31 = 2.0f *( q[2] * q[3] + q[0]  * q[1]);
    	r32 = q[0] * q[0] - q[1] * q[1]  - q[2] * q[2] + q[3] * q[3] ;
    	float yaw= atan2( r11, r12 );
    	float pitch = asin( r21 );
    	float roll = atan2( r31, r32 );
    	cvmSet(att, 0, 0, roll);
    	cvmSet(att, 1, 0, pitch);
    	cvmSet(att, 2, 0, yaw);
//printf("%f  \n",asin(r21));
//printf("%f  \n",r21);
}

//euler to matrix  the matrix is from body to ground
void Euler_To_Matrix(float roll, float pitch, float yaw, CvMat *R)
{
	float cp = cosf(pitch);
	float sp = sinf(pitch);
	float sr = sinf(roll);
	float cr = cosf(roll);
	float sy = sinf(yaw);
	float cy = cosf(yaw);
	
	cvmSet(R,0, 0,cp * cy);
	cvmSet(R,0, 1, ((sr * sp * cy) - (cr * sy)));
	cvmSet(R,0, 2, ((cr * sp * cy) + (sr * sy)));
	cvmSet(R,1, 0, (cp * sy));
	cvmSet(R,1,1, ((sr * sp * sy) + (cr * cy)));
	cvmSet(R,1, 2, ( (cr * sp * sy) - (sr * cy)));
	cvmSet(R,2, 0,  -sp);
	cvmSet(R,2, 1,  sr * cp);
	cvmSet(R,2, 2,  cr * cp);
//printf("%f	%f	%f\n",cvmGet(R,0,0),cvmGet(R,0,1),cvmGet(R,0,2));
}

/* imu */
void imu_callback(const geometry_msgs::TransformStamped& g_imu)
{ 
	cvmSet(q_att,0,0,g_imu.transform.rotation.w);
	cvmSet(q_att,1,0,g_imu.transform.rotation.x);
	cvmSet(q_att,2,0,g_imu.transform.rotation.y);
	cvmSet(q_att,3,0,g_imu.transform.rotation.z);

	Quaternion_To_Euler(q_att, att);

	float roll = cvmGet(att,0,0);
	float pitch = cvmGet(att,1,0);
	float yaw = cvmGet(att,2,0);
	
	Euler_To_Matrix(roll,pitch,yaw,R_g_b);
}

/* motion */
void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
//	printf("mjh--frame_id:%s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
//	for (int i = 0; i < 3; i++){
//		printf("mjh--global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
//	}
//	fprintf(fp,"%s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
//	for (int i = 0; i < 3; i++){
		//fprintf(fp2,"x=%f\t y=%f \tz=%f\ttime=%f\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z,);
//	}
//	fprintf(fp,"\n");

	/*CvMat* pos_mat = cvCreateMat(3,3,CV_32FC1);

	cvmSet(pos_mat, 0, 0, g_pos.vector.x);
	cvmSet(pos_mat, 1, 0, g_pos.vector.y);
	cvmSet(pos_mat, 2, 0, g_pos.vector.z);

	cvGEMM(R_g_b,pos_mat,1,NULL,0,pos_mat);
	
	px= cvmGet(pos_mat, 0, 0);
	py = cvmGet(pos_mat, 1, 0);
	pz = cvmGet(pos_mat, 2, 0);

	cvReleaseMat(&pos_mat);*/
}

void vehicle_pos_callback(const iarc007::vehicle_pos& wny)
{
//	printf("mjh--frame_id:%s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
//	for (int i = 0; i < 3; i++){
//		printf("mjh--global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
//	}
//	fprintf(fp,"%s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
//	for (int i = 0; i < 3; i++){
		//fprintf(fp2,"x=%f\t y=%f \tz=%f\ttime=%f\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z,);
//	}
//	fprintf(fp,"\n");

	CvMat* pos_mat = cvCreateMat(3,3,CV_32FC1);

	cvmSet(pos_mat, 0, 0, wny.pos.x);
	cvmSet(pos_mat, 1, 0, wny.pos.y);
	cvmSet(pos_mat, 2, 0, wny.pos.z);

	printf("wny.pos.x====%f\n",wny.pos.x);
	printf("wny.pos.y====%f\n",wny.pos.y);	

	//cvGEMM(R_g_b,pos_mat,1,NULL,0,pos_mat);
	
	px = cvmGet(pos_mat, 0, 0);
	py = cvmGet(pos_mat, 1, 0);
	pz = cvmGet(pos_mat, 2, 0);

	cvReleaseMat(&pos_mat);
}

void yqt_obstacle(const iarc007::obstacle& wny)
{
	
	obstacle.num=wny.num;
	printf("obstacle.num====%d\n",obstacle.num);
	//wny.obstacle_x.resize(wny.num);
	//wny.obstacle_y.resize(wny.num);
	for(int p=0;p<wny.num;p++)
	{
		obstacle.x[p]=wny.obstacle_x[p];
		obstacle.y[p]=wny.obstacle_y[p];
		printf("obstacle(%d)_x===%f\n",p,obstacle.x[p]);
		printf("obstacle(%d)_y===%f\n",p,obstacle.y[p]);
	}

	obstacle.flag=wny.biaozhiwei[0];
	printf("obstacle.flag===%d\n",obstacle.flag);
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "flight_control");
    ros::NodeHandle nh;

    memset(&obstacle,0,sizeof(obstacle));

    DJIDrone* drone = new DJIDrone(nh);

    R_g_b = cvCreateMat(3,3,CV_32FC1);
    q_att = cvCreateMat(4,1,CV_32FC1);
    att = cvCreateMat(3,1,CV_32FC1);

    ros::Subscriber position_sub = nh.subscribe("/guidance/position",10, position_callback);
    ros::Subscriber imu_sub = nh.subscribe("/guidance/imu", 10, imu_callback);
    ros::Subscriber vehicle_pos_sub = nh.subscribe("/iarc007/vehicle_pos",10, vehicle_pos_callback);
    ros::Subscriber obstacle_pub = nh.subscribe("yqt_obstacle",10,yqt_obstacle);
       for(int i=0;i<10;i++)
		{
			drone->activate();
			usleep(10000);
			printf("adrone->activation==%d\n",drone->activation);
			if(!drone->activation)
			{
			    ROS_INFO("active success ok !!");
				break;

			}
		}

	/*all parameters based on the ground coordinate system*/
	/*units of all parameters are m and m/s */


	/*define the position*/

	float x_t=0.0;		// target
	float y_t=0.0;
	float x_o=-1.2;		// obstacle
	float y_o=-0.7;
	float x_q=0.0;		// quadrotor
	float y_q=0.0;


	/*define the velocity*/

	float v_tx=0.0;		// target_velocity 
	float v_ty=0.0;
	float v_ax=0.0;		// avoid_velocity
	float v_ay=0.0;
	float v_gx=0.0;		// guidance_velocity
	float v_gy=0.0;

	float v_x=0.0;		//complex_velocity
	float v_y=0.0;
	float v_h=0.0;


	/*define other parameters of target_velocity*/

	float k_target=1.0;	// proportionality of target
	float x_qt=0.0;
	float y_qt=0.0;
	float target_distance=0.0;


	/*define other parameters of avoid_velocity*/
	
	float r_obstacle=0.25;	// radius of obstacle itself
	float R_obstacle=0.12;	// radius of obstacle area
	float k_o=0.0;		// proportionality of obstacle itself
	float L_o=0.0;		// proportionality of obstacle area
	float U=0.8;
	float T=0.2;
	float a=0.0;
	float b=0.0;

	float x_wait=0.0;
	float y_wait=0.0;
	float k_obstacle=0.0;	// proportionality of obstacle
	float x_qo=0.0;
	float y_qo=0.0;
	float obstacle_distance=0.0;


	/*define other parameters of guidance_velocity*/

	float k_guidance=0.7;	// proportionality of guidance


	/*define the */

	int count=0;
	int num=0;
	float number=0.0;
	/*define the limitation of velocity signal*/

	float vx_limitation=1.0;
	float vy_limitation=1.0;


	ros::Rate m(50);
	while(ros::ok())
	{
		ros::spinOnce();


		if(drone->rc_channels.gear==-4545.0)
		{
			drone->request_sdk_permission_control();
			usleep(1000);

			/*set a circle trajectory of target*/
	
			count++;
			num=count%720;
			number=(float)num;
			x_t=1.0*cos(number*3.14/360.0)-1.0;		/*T=14.4s，r=2m，O=（0,0）*/
			y_t=1.0*sin(number*3.14/360.0);		/*x_limitation and y_limitation will adjust T*/

			/*read these parameters*/
	
			x_o=-1.2;  //obstacle
			y_o=-0.7;

			x_q=px;
			y_q=py;
			//x_q = px - px0;
			//y_q = py - py0;
			//printf("v_x=%f\n",v_x);
			//printf("v_y=%f\n",v_y);
			//cout << "p_x = " << x_q << "p_y = " << y_q << endl;

			/*calculate target_velocity*/
	
			x_qt=x_t-x_q;					
			y_qt=y_t-y_q;
			//target_distance=sqrt(x_qt*x_qt+y_qt*y_qt);

			v_tx=k_target*x_qt;
			v_ty=k_target*y_qt;


			/*calculate obstacle_velocity*/

			/*x_wait=obstacle.x[0];					
			y_wait=obstacle.y[0];
			for(int t=1;t<obstacle.num-1;t++)	//select from yqt
			{
				if((x_wait*x_wait+y_wait*y_wait)<(obstacle.x[t]*obstacle.x[t]+obstacle.y[t]*obstacle.y[t]))
				{
				}
				else
				{
					x_wait=-obstacle.x[t];		
					y_wait=-obstacle.y[t];
				}
			}
			x_qo=-x_wait;	//obstacle from yqt				
			y_qo=-y_wait;*/

			x_qo=x_q-x_o;
			y_qo=y_q-y_o;
			obstacle_distance=sqrt(x_qo*x_qo+y_qo*y_qo);
	
			k_o=U*k_target;
			L_o=R_obstacle/sqrt(U/T-1.0);

			if(obstacle_distance>(r_obstacle+R_obstacle))
			{
				k_obstacle=0.0;
			}	
			else if(obstacle_distance<(r_obstacle+R_obstacle)&&obstacle_distance>r_obstacle)
			{
				a=(obstacle_distance-r_obstacle)/L_o;
				k_obstacle=k_o/(1.0+a*a);
			}
			else if(obstacle_distance<r_obstacle)
			{
				b=obstacle_distance/r_obstacle;
				k_obstacle=k_o/(b*b);
			}	

			v_ax=k_obstacle*x_qo;
			v_ay=k_obstacle*y_qo;


			/*calculate guidance_velocity*/	

			v_gx=-k_guidance*v_ay;	// lacking the judgment of rotation direction 
			v_gy=k_guidance*v_ax;	


			/*calculate complex_velocity*/

			//v_x=v_tx+v_ax+v_gx;
			//v_y=v_ty+v_ay+v_gy;
			v_x=v_tx;
			v_y=v_ty;

			/*limitation of velocity signal*/
	
			if(v_x>vx_limitation)
			{
				v_x=vx_limitation;
			}
			else if(v_x<-vx_limitation)
			{
				v_x=-vx_limitation;
			}
			else
			{
			}

			if(v_y>vy_limitation)
			{
				v_y=vy_limitation;
			}
			else if(v_y<-vy_limitation)
			{
				v_y=-vy_limitation;
			}
			else
			{
			}

			/*keep M100 on a certain altitude*/

			/*if(drone->ultrasonic.ranges[0]<0.8 )
			{
                        	v_h=0.05;
			}
			else if(drone->ultrasonic.ranges[0]>1.0 )
			{
	                        v_h=-0.05;
			}
			else
			{
	                        v_h=0.0;
			}*/

			/*transmit velocity signal to M100*/
		
				
			drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_PALSTANCE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |			//BODY or GROUND
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            v_x,v_y,0.0,0.0);
                    
                }
               

		else
		{
			/*float x_t=2.5;		// target
			float y_t=2.1;
			float x_o=2.0;		// obstacle
			float y_o=3.0;
			float x_q=0.0;		// quadrotor
			float y_q=0.0;

			px0 = px;
			py0 = py;
			pz0 = pz;*/
		}
		m.sleep();
	}

	cvReleaseMat(&q_att);
	cvReleaseMat(&att);
	cvReleaseMat(&R_g_b);

	return 0;
}
