#include <ros/ros.h>

#include <sys/time.h>
#include <unistd.h>
#include <time.h>
#include <fstream> 

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/GlobalPosition.h>

#include <iarc/raw_position.h>

#include "iarc/navigation/common.h"
#include "iarc/navigation/math.h"

Mat ImageSource;

int ImageNumber;


Mat R_cz_pic = Mat(3, 3, CV_32FC1, Scalar(0));

Mat R_g_bQuaternion = Mat(4, 1, CV_32FC1, Scalar(0));
Mat R_g_b = Mat(3, 3, CV_32FC1, Scalar(0));
Mat R_cz_g = Mat(3, 3, CV_32FC1, Scalar(0));
Mat R_cz_b = Mat::ones(3, 3, CV_32FC1);

Mat PilotPosition = Mat(3, 1, CV_32FC1, Scalar(0));

float Height;

bool FirstFlag = true;

float StartX;
float StartY;

ros::Subscriber wide_angle_sub;
ros::Subscriber position_sub;
ros::Subscriber attitude_sub;
ros::Subscriber ultrasonic_sub;
ros::Subscriber gloabal_position_sub;
/************************************高度滤波***********************************/
float HeightKF = 0.15;

sensor_msgs::LaserScan CurrentUltrasonic;
dji_sdk::GlobalPosition LastGlobalPosition;
dji_sdk::GlobalPosition CurrentGlobalPosition;

float LastHeight = 0;

float GuidanceLast = 0;

bool FirstTimeHeight = true;

float HeightPre = 0;
float LastHeightKF = 0;

float P0_Height=0.1;
float Ph_Pre=0;
float Q_Height=0.0001;
float Kg_Height=0;
float R_Height=0.08;

int Ah=1,Bh=1,Hh=1,Ih=1;

void KalmanHeight(float CurrentAltitude, float LastAltitude, float Ultrasonic)
{
    if(!FirstTimeHeight)
    {
	HeightPre = Ah * LastHeightKF + Bh * (CurrentAltitude-LastAltitude);
	Ph_Pre = Ah * P0_Height * Ah + Q_Height;
	Kg_Height = Ph_Pre * Hh / (Hh * Ph_Pre * Hh + R_Height);
	HeightKF = HeightPre + Kg_Height * (Ultrasonic - Hh * HeightPre);
	P0_Height = (Ih - Kg_Height * Hh) * Ph_Pre;
	LastHeightKF = HeightKF; 
    }
    else
    {
	LastHeightKF = Ultrasonic;
	P0_Height = 0.1;
	R_Height = 0.08;
	Q_Height = 0.0001;
	FirstTimeHeight = false;
    }
}

/************************************高度滤波***********************************/
void wide_angle_callback(sensor_msgs::Image img_msg)
{ 
  cv_bridge::CvImagePtr img_bridge_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);       
  img_bridge_ptr->image.copyTo(ImageSource);   
  ImageNumber++;
}
void attitude_callback(const dji_sdk::AttitudeQuaternion& Attitude)
{
    R_g_bQuaternion.at<float>(0, 0) = Attitude.q0;
    R_g_bQuaternion.at<float>(1, 0) = Attitude.q1;
    R_g_bQuaternion.at<float>(2, 0) = Attitude.q2;
    R_g_bQuaternion.at<float>(3, 0) = Attitude.q3;
    
    Quaternion2Matrix(R_g_bQuaternion,  R_g_b);
    R_cz_b = R_cz_g * R_g_b;
}

void raw_position_callback(const iarc::raw_position& position)
{
    PilotPosition.at<float>(0, 0) = position.x;
    PilotPosition.at<float>(1, 0) = position.y;
    PilotPosition.at<float>(2, 0) = position.z;
    
    PilotPosition = R_cz_g * PilotPosition;
    
    if(FirstFlag)
    {
	StartX = PilotPosition.at<float>(0,0);
	StartY = PilotPosition.at<float>(1,0);
	FirstFlag = false;
    }
    PilotPosition.at<float>(0, 0) -= StartX; 
    PilotPosition.at<float>(1, 0) -= StartY;
}

void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{ 
    CurrentUltrasonic.ranges.resize(3);
    
    CurrentUltrasonic = g_ul;
    
    float UltrasonicOriginal = CurrentUltrasonic.ranges[0];
    
    if(UltrasonicOriginal <= 0.05)
    {
	CurrentUltrasonic.ranges[2] = LastHeight;
	
	if(LastHeight <= 0.05)
	{
	    cout << "error -1" << endl;
	}
    }
    else if(fabs(UltrasonicOriginal - 0.5) <= 0.05 || fabs(UltrasonicOriginal - 0.63) <= 0.02 || fabs(UltrasonicOriginal - 0.8) <= 0.02||fabs(UltrasonicOriginal - 0.35) <= 0.05 || fabs(UltrasonicOriginal - 0.58) <= 0.03||fabs(UltrasonicOriginal - 0.27) <= 0.02 ||fabs(UltrasonicOriginal - 0.93) <= 0.03)
    {
	LastHeight += 2.5 * (CurrentGlobalPosition.altitude - LastGlobalPosition.altitude);
	if(LastHeight <= 0.05)
	{
	    cout << "error 0" << endl;
	}
	CurrentUltrasonic.ranges[2] = LastHeight;
    }
    else
    {
	if(fabs(UltrasonicOriginal - GuidanceLast) * 20 >= 4 && GuidanceLast != 0)
	{
	    LastHeight += 2.5 * (CurrentGlobalPosition.altitude - LastGlobalPosition.altitude);
	    if(LastHeight <= 0.05)
	    {
		cout << "error 1" << endl;
	    }
	}
	else
	{
	    LastHeight = UltrasonicOriginal;
	    if(LastHeight <= 0.05)
	    {
		cout << "error 2" << endl;
	    }
	}
	
	CurrentUltrasonic.ranges[2] = LastHeight;
	
	if(CurrentUltrasonic.ranges[2] <= 0.05)
	{
	    cout << "error 3" << endl;
	}
    }
    if(UltrasonicOriginal <= 0.05)
    {
	CurrentUltrasonic.ranges[1] = GuidanceLast;
    }
    else
    {
	GuidanceLast = UltrasonicOriginal;
	CurrentUltrasonic.ranges[1] = UltrasonicOriginal;
    }
}
void global_position_callback(const dji_sdk::GlobalPosition& g_pos)
{
    LastGlobalPosition = CurrentGlobalPosition;
    CurrentGlobalPosition = g_pos;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mjh_vision_pos");
    ros::NodeHandle my_node;
    ros::Rate rate(50);
    
    ifstream Q_Init("/home/hitcsc/catkin_ws/src/iarc/cfg/init_ori.cfg");
    if(!Q_Init.is_open())
    {
	cout << "Can't open Q_INIT.txt!" << endl;
	exit(-1);
    }
    string s;
    Mat R_g_czQuaternion = Mat(4, 1, CV_32FC1, Scalar(0));
    for(int i = 0; i < 4; i++)
    {
	Q_Init >> s;
	R_g_czQuaternion.at<float>(i, 0) = atof(s.c_str());
    }
    Mat R_g_cz = Mat(3, 3, CV_32FC1, Scalar(0));
    Quaternion2Matrix(R_g_czQuaternion, R_g_cz);
    R_cz_g = R_g_cz.t();
    
    CvMat* R_ca_chess0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/Rotation.xml");
    Mat R_ca_chess = Mat(R_ca_chess0, true);
    if(!R_ca_chess.data)
    {
	cout << "Can't open Rotation.xml" << endl;
	exit(-1);
    }
    CvMat* R_chess_b0=(CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/R_chess_b.xml");//TODO
    Mat R_chess_b = Mat(R_chess_b0, true);
    if(!R_chess_b.data)
    {
	cout << "Can't open R_chess_b.xml!" << endl;
	exit(-1);
    };
    
    Mat R_ca_b = R_ca_chess * R_chess_b;
    Mat R_b_ca = R_ca_b.t();
    
    CvMat* Intrinsic0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/Intrinsics.xml");
    Mat Intrinsic = Mat(Intrinsic0, true);
    if(!Intrinsic.data)
    {
	cout << "Can't open Intrinsic!" << endl;
	exit(-1);
    }
    Mat R_ca_pic = Intrinsic.inv();
    Mat R_b_pic = R_b_ca * R_ca_pic;
    CvMat* Distortion0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/Distortion.xml");
    Mat Distortion = Mat(Distortion0, true);
    if(!Distortion.data)
    {
	cout << "Can't open Distortion!" << endl;
    }
    
    wide_angle_sub = my_node.subscribe<sensor_msgs::Image>("/iarc/wide_angle_video",2, wide_angle_callback);
    attitude_sub = my_node.subscribe("/dji_sdk/attitude_quaternion", 10, attitude_callback);
    position_sub = my_node.subscribe("/iarc/raw_position",20, raw_position_callback);
    ultrasonic_sub = my_node.subscribe("/guidance/ultrasonic", 20, ultrasonic_callback);
    gloabal_position_sub = my_node.subscribe("/dji_sdk/global_position", 20, global_position_callback);
    
    int CurrentImageNumber = -1;
    
    int NF = 0;
    ofstream HeightF("/home/hitcsc/bianjie/Height.txt");
    ofstream RF("/home/hitcsc/bianjie/R.txt");
    ofstream PF("/home/hitcsc/bianjie/Position.txt");
    
    while(ros::ok())
    {
	ros::spinOnce();
	rate.sleep();
	
	if(!ImageSource.data)
	{
	    cout << "No image!" << endl;
	    continue;
	}
	if(CurrentImageNumber == ImageNumber)
	{
	    cout << "Next Image doesn't come!" << endl;
	    continue;
	}
	
	R_cz_pic = R_cz_b * R_b_pic;
	
	for(int i = 0; i < 3; i++)
	{
	    for(int j = 0; j < 3; j++)
	    {
		RF << R_cz_pic.at<float>(i, j) << "\t";
	    }
	}
	RF << endl;
	KalmanHeight(CurrentGlobalPosition.altitude, LastGlobalPosition.altitude, CurrentUltrasonic.ranges[0]);
	HeightF << HeightKF << endl;
	PF << PilotPosition.at<float>(0, 0) << "\t" << PilotPosition.at<float>(1, 0) << endl;
	char Name[100];
	sprintf(Name, "/home/hitcsc/bianjie/Image/%d.jpg", NF);
	imwrite(Name, ImageSource);
	imshow("原图", ImageSource);
	waitKey(1);
	NF++;
	CurrentImageNumber = ImageNumber;
	cout << NF << endl;
    }
    return 0;
}