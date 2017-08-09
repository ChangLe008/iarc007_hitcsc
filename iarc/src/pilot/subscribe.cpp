#include <iarc/pilot/subscribe.h>
#include "iarc/pilot/leo_math.h"

using namespace std;
using namespace cv;

CvMat *q_right_now=cvCreateMat(4,1,CV_32FC1);

CvMat *q_init=cvCreateMat(4,1,CV_32FC1);
CvMat *att_init=cvCreateMat(3,1,CV_32FC1);
CvMat *R_init=cvCreateMat(3,3,CV_32FC1);
CvMat *R_init_i=cvCreateMat(3,3,CV_32FC1);

CvMat *q=cvCreateMat(4,1,CV_32FC1);
CvMat *atti=cvCreateMat(3,1,CV_32FC1);
CvMat *R_now_i=cvCreateMat(3,3,CV_32FC1);

CvMat *att_right_now=cvCreateMat(3,1,CV_32FC1);
Mat R_gn_ma=Mat::zeros(3,3, CV_32F);

#define H_ROBOT 10 //机器人高度
#define IMG_H 480 //图像高度
#define IMG_W 640 //图像宽度

float zeres[9]={0.,0.,0.,0.,0.,0.,0.,0.,0.};
// extern CvMat *R_init;
int last_heght_time=0;
float last_ultrasonic2=0.;

void LEODrone::height_callback(sensor_msgs::LaserScan ul)
{
    this->sona.ranges.resize(1);
	this->sona = ul;
}


void LEODrone::ultrasonic_subscriber_callback(sensor_msgs::LaserScan g_ul)
{
    this->ultrasonic.ranges.resize(3);
	this->ultrasonic = g_ul;
	float ultrasonic_original=this->ultrasonic.ranges[0];
	if(ultrasonic_original<=0.05)
	{
		this->ultrasonic.ranges[2] = last_height;
		if(last_height<=0.05)
		cout << "error-1"<<endl;
	}
	else if(abs(ultrasonic_original-0.5)<=0.05 ||abs(ultrasonic_original-0.63)<=0.02||abs(ultrasonic_original-0.8)<=0.02 || abs(ultrasonic_original-0.35)<=0.05 || abs(ultrasonic_original-0.58)<=0.03 || abs(ultrasonic_original-0.27)<=0.02 || abs(ultrasonic_original-0.93)<=0.03)
	{
		last_height += 2.5*(this->global_position.altitude-last_global_position.altitude);
		if(last_height<=0.05)
		cout << "error0"<<endl;
		this->ultrasonic.ranges[2] = last_height;
	}
	else
	{
		if(abs(ultrasonic_original-guidance_last)*20>=4 && guidance_last!=0)
		{
			last_height += 2.5*(this->global_position.altitude-last_global_position.altitude);
			if(last_height<=0.05)
			cout << "error1"<<endl;			
		}
		else
		{
			last_height = ultrasonic_original;
			if(last_height<=0.05)
			cout << "error2"<<endl;
		}
		this->ultrasonic.ranges[2] = last_height;
		if(this->ultrasonic.ranges[2]<=0.05)
		cout << "error3"<<endl;
	}
	

	if(ultrasonic_original<=0.05)
	{
		this->ultrasonic.ranges[1]=guidance_last;
	}
	else
	{
		guidance_last = ultrasonic_original;
		this->ultrasonic.ranges[1] = ultrasonic_original;
	}	
}


void LEODrone::acceleration_subscriber_callback(dji_sdk::Acceleration acceleration)
{
	float ax=cvmGet(R_init,0,0)*acceleration.ax+cvmGet(R_init,0,1)*acceleration.ay+cvmGet(R_init,0,2)*acceleration.az;
	float ay=cvmGet(R_init,1,0)*acceleration.ax+cvmGet(R_init,1,1)*acceleration.ay+cvmGet(R_init,1,2)*acceleration.az;
	float az=cvmGet(R_init,2,0)*acceleration.ax+cvmGet(R_init,2,1)*acceleration.ay+cvmGet(R_init,2,2)*acceleration.az;
	this->acceleration.ax=ax;
	this->acceleration.ay=ay;
	this->acceleration.az=az;	
}

void LEODrone::attitude_quaternion_subscriber_callback(dji_sdk::AttitudeQuaternion quaternion)
{
	
	this->attitude_quaternion = quaternion;
	cvmSet(q_right_now, 0, 0, attitude_quaternion.q0);
    cvmSet(q_right_now, 1, 0, attitude_quaternion.q1);
    cvmSet(q_right_now, 2, 0, attitude_quaternion.q2);
    cvmSet(q_right_now, 3, 0, attitude_quaternion.q3);	
    Quaternion_To_Euler(q_right_now,att_right_now);
	

    Euler_To_Matrix(-1*cvmGet(att_right_now,0,0),-1*cvmGet(att_right_now,1,0),-1*cvmGet(att_right_now,2,0),R_now_i);
}

void LEODrone::rc_channels_subscriber_callback(dji_sdk::RCChannels rc_channels)
{
	this->rc_channels = rc_channels;
}

void LEODrone::velocity_subscriber_callback(dji_sdk::Velocity velocity)
{
	float vx=cvmGet(R_init,0,0)*velocity.vx+cvmGet(R_init,0,1)*velocity.vy+cvmGet(R_init,0,2)*velocity.vz;
	float vy=cvmGet(R_init,1,0)*velocity.vx+cvmGet(R_init,1,1)*velocity.vy+cvmGet(R_init,1,2)*velocity.vz;
	float vz=cvmGet(R_init,2,0)*velocity.vx+cvmGet(R_init,2,1)*velocity.vy+cvmGet(R_init,2,2)*velocity.vz;
	this->velocity.vx=vx;
	this->velocity.vy=vy;
	this->velocity.vz=vz;
}

void LEODrone::activation_subscriber_callback(std_msgs::UInt8 activation)
{
	this->activation = activation.data;
}


void LEODrone::sdk_permission_subscriber_callback(std_msgs::UInt8 sdk_permission)
{
	this->sdk_permission_opened = sdk_permission.data;
}

void LEODrone::time_stamp_subscriber_callback(dji_sdk::TimeStamp time_stamp)
{
	this->time_stamp = time_stamp;
}

void LEODrone::velocity_guidance_subscriber_callback(geometry_msgs::Vector3Stamped g_vo)
{
	float vx_guidance=cvmGet(R_init,0,0)*g_vo.vector.x+cvmGet(R_init,0,1)*g_vo.vector.y+cvmGet(R_init,0,2)*g_vo.vector.z;
	float vy_guidance=cvmGet(R_init,1,0)*g_vo.vector.x+cvmGet(R_init,1,1)*g_vo.vector.y+cvmGet(R_init,1,2)*g_vo.vector.z;
	float vz_guidance=cvmGet(R_init,2,0)*g_vo.vector.x+cvmGet(R_init,2,1)*g_vo.vector.y+cvmGet(R_init,2,2)*g_vo.vector.z;
	this->velocity_guidance.vector.x = vx_guidance; 
	this->velocity_guidance.vector.y = vy_guidance; 
	this->velocity_guidance.vector.z = vz_guidance; 
}

void LEODrone::l_pose_callback(const dji_sdk::LocalPosition posi)
{
	this->local_position = posi;
}
/*
void LEODrone::acceleration_subscriber_callback(dji_sdk::Acceleration acceleration)
{
	float ax=cvmGet(R_init,0,0)*acceleration.ax+cvmGet(R_init,0,1)*acceleration.ay+cvmGet(R_init,0,2)*acceleration.az;
	float ay=cvmGet(R_init,1,0)*acceleration.ax+cvmGet(R_init,1,1)*acceleration.ay+cvmGet(R_init,1,2)*acceleration.az;
	float az=cvmGet(R_init,2,0)*acceleration.ax+cvmGet(R_init,2,1)*acceleration.ay+cvmGet(R_init,2,2)*acceleration.az;
	this->acceleration.ax=ax;
	this->acceleration.ay=ay;
	this->acceleration.az=az;	
}
*/
void LEODrone::g_pose_callback(const dji_sdk::GlobalPosition g_pos)
{
	last_global_position = this->global_position;
	this->global_position = g_pos;
}


void LEODrone::kalman_filter(iarc::object object_pos)
{

	object_prediction	= object_pos;
	return;
}


void LEODrone::transformer(Point2f img_point, Point3d& world_point, Mat R, Mat M, double height)
{
	if(height<=0.05)//TODO
	{
		std::cout<<"transformer_height error"<< height <<std::endl;
		world_point.z = height;
		world_point.y = -20;
		world_point.x = -20;
		return;
	}
	Mat point1, tmp_p, tmp_r, tmp_r2;
	double s;
	point1.create(3, 1, CV_32FC1);
	tmp_p.create(3, 1, CV_32FC1);
	tmp_r.create(3, 3, CV_32FC1);
	tmp_r2.create(3, 3, CV_32FC1);
	tmp_r=M*R;
	tmp_r2=tmp_r.inv(DECOMP_LU);
	point1 = (Mat_<float>(3,1) << img_point.x, img_point.y, 1);
	tmp_p=tmp_r2*point1;
	s=tmp_p.at<float>(2,0)/height;
	world_point.z=tmp_p.at<float>(2,0)/s;
	world_point.x=tmp_p.at<float>(0,0)/s;
	world_point.y=tmp_p.at<float>(1,0)/s;
}



void LEODrone::target_callback(iarc::target vision_target)
{
	int target_number = vision_target.target_num;
	FILE *hou = fopen("/home/hitcsc/catkin_ws/log/iarc/pilot/houge.txt","a");
	// /cout << target_number;
	vision_target.target_img_x.resize(target_number);
	vision_target.target_img_y.resize(target_number);
	vision_target.target_dir_x.resize(target_number);
	vision_target.target_dir_y.resize(target_number);
	vision_target.target_id.resize(target_number);
	vision_target.target_color.resize(target_number);
	
	sona.ranges.resize(1);

	target_dynamic.target.target_img_x.resize(target_number);
	target_dynamic.target.target_img_y.resize(target_number);
	target_dynamic.target.target_dir_x.resize(target_number);
	target_dynamic.target.target_dir_y.resize(target_number);
	target_dynamic.target.target_area.resize(target_number);
	target_dynamic.target.flag_xinrencike.resize(target_number);
	target_dynamic.target.target_id.resize(target_number);
	target_dynamic.target.target_color.resize(target_number);
	target_dynamic.target_x.resize(target_number);
	target_dynamic.target_y.resize(target_number);
	target_dynamic.target_z.resize(target_number);
	target_dynamic.target_dir_x.resize(target_number);
	target_dynamic.target_dir_y.resize(target_number);
	target_dynamic.img_x.resize(target_number);
	target_dynamic.img_y.resize(target_number);
	target_dynamic.target = vision_target;
	target_dynamic.target_num = target_number;
	// ///**********************
	// vision_target.target_area.resize(target_number);
	
	// cv::Matx33d intrinsics_f;//z:相机内参

    // ifstream intrinsicfile("/home/hitcsc/catkin_ws/src/iarc007/doc/intrinsics_front5_16.txt");
   
	// for(int i=0; i<3; ++i)
	// {
	// 	for(int j=0; j<3; ++j)
	// 	{
	// 		intrinsicfile >> intrinsics_f(i,j);
	// 	}
	// 	cout<<endl;
		
	// }
	// int k = 0;
	// double fx = intrinsics_f(0,0);
	// double fy = intrinsics_f(1,1);
	// double h1 = sona.ranges[0]*100.0;
	// double area1=0.,area2=0.;
	// //cout<<h1<<"=="<<sona.ranges[0]<<endl;
	// if (h1>H_ROBOT)
	// {
	// 	area1 = 15.0*25.0*fx*fy/(h1*h1);
	// 	area2 = 15.0*25.0*fx*fy/((h1-H_ROBOT)*(h1-H_ROBOT));
	// 	//cout << area1 << "sss"<<area2<<endl;
	// }
	// else
	// {
	// 	area1=640*480;
	// 	area2=640*480;
	// 	cout << "error" << endl;
	// }
	// for (int i = 0; i < vision_target.target_num; i++)
	// {
	// 	//cout<<vision_target.target_area[i]<<endl;
	// 	if (vision_target.target_area[i] > 1.2*area2 || vision_target.target_area[i] < area1 / 1.2)
	// 	{
	// 		target_dynamic.target.target_id[i] = -1;
	// 	}
	// 	else
	// 	{
	// 		k++;
	// 		target_dynamic.target.target_id[i] = k; 
			

	// 	}

	// }
	// target_dynamic.target_num=k;
	//cout<<"k"<<k<<endl;
	//cout<<target_dynamic.target_num<<endl;
	// ///**********************
	
	CvPoint real={0};
	Mat R=Mat::zeros(3,3, CV_32F);
	R.at<float>(0,1) = -1.;
	R.at<float>(1,0) = 1.;
	R.at<float>(2,2) = 1.;
	Mat R_ma_bo=Mat::zeros(3,3, CV_32F);
	Mat R_ma_ca=Mat::zeros(3,3, CV_32F);	
	R_ma_bo = Mat(R_now_i);
	//cout <"123";
	// cout<<"R_ma_bo"<<R_ma_bo<<endl;
	R_ma_ca =R_gn_ma*R_ma_bo*R;
	for(int i=0;i<target_number;i++)
	{
		int i_x = vision_target.target_img_x[i];
		int i_y = vision_target.target_img_y[i];
		// real = 	DistortionPoint(i_x,i_y,distortion,intrinsic);
		real.x = i_x;
		real.y = i_y;
		target_dynamic.target.target_img_x[i]=real.x;
		target_dynamic.target.target_img_y[i]=real.y;
		Point2f img_p;
		img_p.x = real.x;
		img_p.y = real.y;
		int x = real.x;
		int y = real.y;
		Point3d world_p;
		double h =sona.ranges[0];
		transformer(img_p, world_p, R_ma_ca, M, h);
		
		target_dynamic.target_x[i]=world_p.x;
		target_dynamic.target_y[i]=world_p.y;
		target_dynamic.target_z[i]=world_p.z;
		// if(i==0)
		// {
		// 		fprintf(hou,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%lf\t%lf\t%lf\n", R_ma_ca.at<float>(0,0),R_ma_ca.at<float>(0,1),R_ma_ca.at<float>(0,2),R_ma_ca.at<float>(1,0),R_ma_ca.at<float>(1,1),R_ma_ca.at<float>(1,2),R_ma_ca.at<float>(2,0),R_ma_ca.at<float>(2,1),R_ma_ca.at<float>(2,2),h,(float)x,(float)y,-1*world_p.x,-1*world_p.y,-1*world_p.z);
		// 		fclose(hou);
		// }
	}
	Mat R_ma_am=Mat::zeros(3,3, CV_32F);
	R_ma_am = Mat(R_init);
	// cout << M.rowRange(0,1).clone() <<"\t"<<M.rowRange(1,2).clone() <<"\t"<<M.rowRange(2,3).clone() <<"\t";
	
	for(int i=0;i<target_number;i++)
	{
		Mat point1, tmp_p, tmp_r,point2,tmp_p1;
		// -1*cvmGet(att_right_now,2,0)
		point1.create(3, 1, CV_32FC1);
		tmp_p.create(3, 1, CV_32FC1);
		tmp_p1.create(3, 1, CV_32FC1);
		tmp_r.create(3, 3, CV_32FC1);
		tmp_r=(R_ma_bo*R_ma_am).inv(DECOMP_LU);
		point1 = (Mat_<float>(3,1) << (float) vision_target.target_dir_x[i], (float) vision_target.target_dir_y[i], 0);
		point2 = (Mat_<float>(3,1) << (float)target_dynamic.target.target_img_x[i],(float)target_dynamic.target.target_img_y[i],0);
		tmp_p=tmp_r*point1;
		tmp_p1 = tmp_r*point2;

		float len = sqrt(tmp_p.at<float>(0,0)*tmp_p.at<float>(0,0)+tmp_p.at<float>(1,0)*tmp_p.at<float>(1,0));
		float len1 = sqrt(tmp_p1.at<float>(0,0)*tmp_p1.at<float>(0,0)+tmp_p1.at<float>(1,0)*tmp_p1.at<float>(1,0));
		if(len!=0)
		{
			target_dynamic.target_dir_y[i]=-1*tmp_p.at<float>(0,0)/len;
			target_dynamic.target_dir_x[i]=tmp_p.at<float>(1,0)/len;
		}
		if(i==0)
		{
				fprintf(hou,"%d\t%d\t%f\t%f\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%f\n",target_dynamic.target.target_dir_x[0],target_dynamic.target.target_dir_y[0],target_dynamic.target_dir_x[0],target_dynamic.target_dir_y[0],target_dynamic.target.flag_xinrencike[0], tmp_r.at<float>(0,0),tmp_r.at<float>(0,1),tmp_r.at<float>(0,2),tmp_r.at<float>(1,0),tmp_r.at<float>(1,1),tmp_r.at<float>(1,2),tmp_r.at<float>(2,0),tmp_r.at<float>(2,1),tmp_r.at<float>(2,2),target_dynamic.target.target_area[0],sona.ranges[0]);
				fclose(hou);
		}
		target_dynamic.img_x[i] = tmp_p1.at<float>(1,0)/len1;
		target_dynamic.img_y[i] = -1*tmp_p1.at<float>(0,0)/len1;
		
	}
	return;
}


CvPoint LEODrone::DistortionPoint(int x,int y,CvMat* distortion,CvMat* intrinsic)
{
	float k1=cvmGet(distortion,0,0);
	float k2=cvmGet(distortion,1,0);
	float p1=cvmGet(distortion,2,0);
	float p2=cvmGet(distortion,3,0);
	float k3=cvmGet(distortion,4,0);

	float cx=cvmGet(intrinsic,0,2);
	float cy=cvmGet(intrinsic,1,2);
	float fx=cvmGet(intrinsic,0,0);
	float fy=cvmGet(intrinsic,1,1);
	int flag=0;
	CvPoint ud={0};

	for (int nx = -100; nx < 640 + 100; nx++)
	{
		for (int ny = -100; ny < 640 + 100; ny++)
		{
			if(flag==1)
				break;
			float xx = (nx - cx) / fx;
			float yy = (ny - cy) / fy;
			float r2 = pow(xx, 2) + pow(yy, 2);
			float r4 = pow(r2, 2);
			float xxx = xx*(1 + k1*r2 + k2*r4) + 2 * p1*xx*yy + p2*(r2 + 2 * xx*xx);
			float yyy = yy*(1 + k1*r2 + k2*r4) + 2 * p1*xx*yy + p2*(r2 + 2 * yy*yy);
			float xxxx = xxx*fx + cx;
			float yyyy = yyy*fy + cy;
			if(fabs(xxxx-x*1.0)<1.0&&fabs(yyyy-y*1.0)<1.0)
			{
				ud.x=nx;
				ud.y=ny;
				flag=1;
			}
		}
	}
	return ud;

}

void LEODrone::position_callback(iarc::position position_from_lf)
{
	position_lf = position_from_lf;
}

void LEODrone::lidar_obstacle_callback(iarc::obstacle obs)
{
	int obsnum = obs.num;
	obstacle_lidar = obstacle_dynamic;
	Mat R_ma_bo=Mat::zeros(3,3, CV_32F);
	Mat R_ma_am=Mat::zeros(3,3, CV_32F);
	R_ma_bo = Mat(R_now_i);//R_now_i--R_body_ground
	R_ma_am = Mat(R_init);//R_init--R_ground_saichangd
	obs.o_x.resize(obsnum);
	obs.o_y.resize(obsnum);
	obstacle_dynamic.o_x.resize(obsnum);
	obstacle_dynamic.o_y.resize(obsnum);
	obstacle_dynamic = obs;
	int real_num = 0;
	for(int i=0;i<obsnum;i++)
	{
		if(sqrt(obs.o_x[i]*obs.o_x[i]+obs.o_y[i]*obs.o_y[i])>=0.5)
		real_num++;
		{
			Mat point1, tmp_p, tmp_r;
			point1.create(3, 1, CV_32FC1);
			tmp_p.create(3, 1, CV_32FC1);
			tmp_r.create(3, 3, CV_32FC1);
			//tmp_r=(R_ma_bo*R_ma_am).inv(DECOMP_LU);
			tmp_r=(R_ma_am*R_ma_bo).inv(DECOMP_LU);
			point1 = (Mat_<float>(3,1) << obs.o_x[i], obs.o_y[i], 0);
			tmp_p=tmp_r*point1;
			 obstacle_dynamic.o_x[i]=tmp_p.at<float>(0,0);
			 obstacle_dynamic.o_y[i]=tmp_p.at<float>(1,0);

		// 	cout<<"R_ma_am"<<R_ma_am<<endl;
		// //	cout<<R_ma_bo<<endl;
		// 	 cout<<"="<<tmp_r<<endl<<endl;

		// 	 cout<<obs.o_x[i]<<","<<obs.o_y[i]<<endl;
		// 	cout<<i<<"="<<obstacle_dynamic.o_x[i]<<","<<obstacle_dynamic.o_y[i]<<endl;
			//obstacle_dynamic.o_x[i] =  cos(cvmGet(att_right_now,2,0)-cvmGet(att_init,2,0))*obs.o_x[i] - sin(cvmGet(att_right_now,2,0)-cvmGet(att_init,2,0))*obs.o_y[i];
			//obstacle_dynamic.o_y[i] =  sin(cvmGet(att_right_now,2,0)-cvmGet(att_init,2,0))*obs.o_x[i] + cos(cvmGet(att_right_now,2,0)-cvmGet(att_init,2,0))*obs.o_y[i];
		}
	}

}

void LEODrone::opti_position_callback(network_client::Optitrack_data position)
{
	this->opti_pos = position;               
}

LEODrone::LEODrone(ros::NodeHandle& nh)
	{
			    ultrasonic_subscriber = nh.subscribe<sensor_msgs::LaserScan>("/guidance/ultrasonic", 1, &LEODrone::ultrasonic_subscriber_callback, this);
			    velocity_guidance_subscriber= nh.subscribe<geometry_msgs::Vector3Stamped>("/guidance/velocity", 1, &LEODrone::velocity_guidance_subscriber_callback,this);
		 	 	acceleration_subscriber = nh.subscribe<dji_sdk::Acceleration>("dji_sdk/acceleration", 10, &LEODrone::acceleration_subscriber_callback, this);
		        attitude_quaternion_subscriber = nh.subscribe<dji_sdk::AttitudeQuaternion>("dji_sdk/attitude_quaternion", 10, &LEODrone::attitude_quaternion_subscriber_callback, this);
		        rc_channels_subscriber = nh.subscribe<dji_sdk::RCChannels>("dji_sdk/rc_channels", 10, &LEODrone::rc_channels_subscriber_callback, this);
		        velocity_subscriber = nh.subscribe<dji_sdk::Velocity>("dji_sdk/velocity", 10, &LEODrone::velocity_subscriber_callback, this);
		        activation_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/activation", 10, &LEODrone::activation_subscriber_callback, this);
		        sdk_permission_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/sdk_permission", 10, &LEODrone::sdk_permission_subscriber_callback, this);
				time_stamp_subscriber = nh.subscribe<dji_sdk::TimeStamp>("dji_sdk/time_stamp", 10, &LEODrone::time_stamp_subscriber_callback,this);
				l_position		= nh.subscribe("/dji_sdk/local_position",10,&LEODrone::l_pose_callback,this);
				g_position		= nh.subscribe("/dji_sdk/global_position",10,&LEODrone::g_pose_callback,this);
				target_subscriber = nh.subscribe("/target",10,&LEODrone::target_callback,this);

				obstacle_subscriber = nh.subscribe<iarc::obstacle>("/obstacle/lidar",10,&LEODrone::lidar_obstacle_callback,this);
				position_sub	= nh.subscribe("/iarc/pilot_position",10,&LEODrone::position_callback,this);

				Opti_pos      = nh.subscribe("/network_client/network_optitrack_data", 10, &LEODrone::opti_position_callback,this);

				height_subscriber = nh.subscribe("/ultrasonic",10,&LEODrone::height_callback,this);


				ifstream init_orientation("/home/hitcsc/catkin_ws/src/iarc/cfg/init_ori.cfg");
				
				if(init_orientation.is_open())
				{
					cout << "Init orientation initation success"<< endl;
				}
				else
				{
					cout << "Init orientation error"<< endl;
					return;
				}
				float q0, q1, q2, q3;
				init_orientation >> q0;
				init_orientation >> q1;
				init_orientation >> q2;
				init_orientation >> q3;
				init_orientation.close();
				cvmSet(q_init, 0, 0, q0);
				cvmSet(q_init, 1, 0, q1);
				cvmSet(q_init, 2, 0, q2);
				cvmSet(q_init, 3, 0, q3);
				Quaternion_To_Euler(q_init,att_init);
				Euler_To_Matrix(cvmGet(att_init,0,0),cvmGet(att_init,1,0),cvmGet(att_init,2,0),R_init);
				Euler_To_Matrix(-1*cvmGet(att_init,0,0),-1*cvmGet(att_init,1,0),-1*cvmGet(att_init,2,0),R_init_i);
				R_gn_ma = Mat( R_init);	   
	}

	LEODrone::~LEODrone()
	{
		cvReleaseMat(&att_init);
		cvReleaseMat(&q_init);
		cvReleaseMat(&R_init);
	}
