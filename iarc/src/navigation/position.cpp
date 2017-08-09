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
#include <dji_sdk/Velocity.h>
#include <dji_sdk/GlobalPosition.h>
#include <network_client/Optitrack_data.h>

#include "iarc/navigation/common.h"
#include "iarc/navigation/threshold_image.h"
#include "iarc/navigation/line_classified.h"
#include "iarc/navigation/math.h"
#include "iarc/navigation/test.h"
#include "iarc/navigation/find_boundary.h"
#include "iarc/navigation/line_inside.h"
#include "iarc/position.h"

/*********************************全局变量********************************/
int Start0Num = 2; //y
int Start90Num = 1; //x

float Check90 = 0;
float Check0 = 0;

Mat ImageSource;
int ImageNumber;

Mat R_g_bQuaternion = Mat(4, 1, CV_32FC1, Scalar(0));
Mat Eurler = Mat(3, 1, CV_32FC1, Scalar(0));
Mat R_g_b = Mat(3, 3, CV_32FC1, Scalar(0));

double LastTime = 0;
double CurrentTime = 0;
double TimeInterval = 0;
Mat RawPilotPosition = Mat(3, 1, CV_32FC1,Scalar(0));
float CurrentVelocityX = 0;
float CurrentVelocityY = 0;
float CurrentVelocityZ = 0;

float Height;

float Optitrack_yaw;
/************************************************************************/

ros::Subscriber velocity_sub;
ros::Subscriber wide_angle_sub;
ros::Subscriber attitude_sub;
ros::Subscriber height_sub;
ros::Subscriber optitrack_sub;

ros::Publisher position_pub;

/*********************************回调函数********************************/
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
    
    Quaternion2Euler(R_g_bQuaternion, Eurler); //TODO:删除
    Euler2Matrix(Eurler,  R_g_b);
}

void velocity_callback(const dji_sdk::Velocity& g_vo)
{   
    CurrentTime = (double)g_vo.header.stamp.toNSec();
    TimeInterval = (CurrentTime - LastTime) / 1e9;
    RawPilotPosition.at<float>(0, 0) += CurrentVelocityX * TimeInterval;
    RawPilotPosition.at<float>(1, 0) += CurrentVelocityY * TimeInterval;
    RawPilotPosition.at<float>(2, 0) += CurrentVelocityZ * TimeInterval;
    CurrentVelocityX = g_vo.vx;
    CurrentVelocityY = g_vo.vy;
    CurrentVelocityZ = g_vo.vz;
    LastTime = CurrentTime;
}

void height_callback(const sensor_msgs::LaserScan& g_ul)
{
    if(g_ul.ranges[0] > 0)
    {
	Height = g_ul.ranges[0] + 0.11;
	//cout << "Height\t" << Height << endl;
    }
}

void optitrack_callback(const network_client::Optitrack_data Optitrack)
{
    Optitrack_yaw = Optitrack.yaw / CV_PI * 180;
}
/************************************************************************/


int main(int argc, char** argv)
{
    ros::init(argc, argv, "position");
    ros::NodeHandle my_node;
    ros::Rate rate(10);
    
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
    Mat R_cz_g = R_g_cz.t();
    
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
    CvMat* Intrinsic0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/NewIntrinsics.xml");
    Mat Intrinsic = Mat(Intrinsic0, true);
    if(!Intrinsic.data)
    {
	cout << "Can't open Intrinsic!" << endl;
	exit(-1);
    }
    Mat R_ca_pic = Intrinsic.inv();
    Mat R_b_pic = R_b_ca * R_ca_pic;
    
    wide_angle_sub = my_node.subscribe<sensor_msgs::Image>("/iarc/wide_angle_video",2, wide_angle_callback);
    attitude_sub = my_node.subscribe("/dji_sdk/attitude_quaternion", 10, attitude_callback);
    velocity_sub = my_node.subscribe("/dji_sdk/velocity", 100, velocity_callback);
    height_sub = my_node.subscribe("/ultrasonic", 10, height_callback);
    optitrack_sub = my_node.subscribe("/network_client/network_optitrack_data", 10, optitrack_callback);
    position_pub = my_node.advertise<iarc::position>("iarc/position", 10);
    iarc::position pilot_position;
    
    /***********************************保存数据************************************/
    ofstream EurlerF("/home/hitcsc/Data/Eurler.txt");
    ofstream OptitrackF("/home/hitcsc/Data/Optitrack.txt");
    ofstream CompensateAngleF("/home/hitcsc/Data/CompensateAngle.txt");
    ofstream Check90F("/home/hitcsc/Data/Check90.txt");
    ofstream Check0F("/home/hitcsc/Data/Check0.txt");
    ofstream PositionF("/home/hitcsc/Data/Position.txt");
    ofstream FrequencyF("/home/hitcsc/Data/Frequency.txt");
    ofstream HeightF("/home/hitcsc/Data/Height.txt");
    ofstream RawPositionF("/home/hitcsc/Data/RawPosition.txt");
    char ImageName[100];
/********************************************************************************/

    int CurrentImageNumber = -1;
    
    for(int i = 0; i < 4; i++)
    {
	pilot_position.border[i] = false;
    }
    
    float Start90 = Line90RealCoordinate[Start90Num];
    float Start0 =  Line0RealCoordinate[Start0Num];

    pilot_position.x = Start90;
    pilot_position.y = Start0;
    int NUM = 0;
    
     while(ros::ok())
     {
	 double RosTime1 = (double)ros::Time::now().nsec * 1e-9 + (double)ros::Time::now().sec;
	 clock_t time;
	 time = clock();
	 double Timer = (double)time / CLOCKS_PER_SEC;
	 ros::spinOnce();
	 
	 if(!ImageSource.data)
	{
	    cout << "没有图片!" << endl;
	    continue;
	}
	
	Mat CompensateMatrix = (Mat_<float>(3, 3) << cosf(CompensateAngle), -sinf(CompensateAngle), 0,
					                                                 sinf(CompensateAngle),  cosf(CompensateAngle), 0,
					                                                 0, 0, 1);
	Mat R_cz_b = R_cz_g * R_g_b;
	
	R_cz_b = CompensateMatrix * R_cz_b;
	Mat R_cz_pic = R_cz_b * R_b_pic;
	
	Mat PilotPosition = Mat(3, 3, CV_32FC1, Scalar(0));
	PilotPosition = R_cz_g * RawPilotPosition;
	
	PilotPosition.at<float>(0, 0) = PilotPosition.at<float>(0, 0) + Start90;
	PilotPosition.at<float>(1, 0) = PilotPosition.at<float>(1, 0) + Start0;
	
	/************************************未校正位置***************************************/
	RawPositionF << Timer << "\t" << PilotPosition.at<float>(0, 0) << "\t" << PilotPosition.at<float>(1, 0) << endl;
	/************************************************************************************/
	
	PilotPosition.at<float>(0, 0) = PilotPosition.at<float>(0, 0) - Check90;
	PilotPosition.at<float>(1, 0) = PilotPosition.at<float>(1, 0) - Check0;
	
	PilotPosition.at<float>(2, 0) = Height;
	
	pilot_position.x = PilotPosition.at<float>(0, 0);
	pilot_position.y = PilotPosition.at<float>(1, 0);
	
	Mat MyR_g_bQuaternion = Mat(4,1, CV_32FC1, Scalar(0));
	Matrix2Quernion(R_cz_b, MyR_g_bQuaternion);
	pilot_position.q0 = MyR_g_bQuaternion.at<float>(0, 0);
	pilot_position.q1 = MyR_g_bQuaternion.at<float>(1, 0);
	pilot_position.q2 = MyR_g_bQuaternion.at<float>(2, 0);
	pilot_position.q3 = MyR_g_bQuaternion.at<float>(3, 0);
	
	position_pub.publish(pilot_position);
	
	//ResultTest(ImageSource, PilotPosition);
	waitKey(1);
	/**************************保存数据*****************************/
	EurlerF  << Timer << "\t" << Eurler.at<float>(0, 0) / CV_PI * 180 << "\t" << Eurler.at<float>(1, 0) / CV_PI * 180 << "\t" <<Eurler.at<float>(2, 0) / CV_PI * 180 << endl;
	CompensateAngleF << Timer << "\t" << CompensateAngle / CV_PI * 180  << endl;
	OptitrackF << Timer << "\t" << Optitrack_yaw << endl;
	Check90F << Timer << "\t" << Check90 << endl;
	Check0F << Timer << "\t" << Check0 << endl;
	PositionF << Timer << "\t" << PilotPosition.at<float>(0, 0) << "\t" << PilotPosition.at<float>(1, 0) << endl;
	HeightF << Timer << "\t" << Height << endl;
	
	sprintf(ImageName, "/home/hitcsc/Data/Image/%d.jpg", NUM);
	imwrite(ImageName, ImageSource);
	NUM++;
	/**************************************************************/
	if(CurrentImageNumber == ImageNumber)
	{
	    cout << "下一张图片还没到！" << endl;
	    continue;
	}
	if(Height < 0)
	{
	    cout << "高度低了！" << endl;
	    for(int i = 0; i <4; i++)
	    {
		pilot_position.border[i] = false;
	    }
	    continue;
	}    
	
	Mat ImageWhiteThreshold;
	ThresholdWhite(ImageSource, ImageWhiteThreshold);
	//imshow("白色阈值后", ImageWhiteThreshold);
	
	vector<Vec4i> Lines;
	HoughLinesP(ImageWhiteThreshold, Lines, 1, CV_PI/180, 80, 30, 50);
	
	//AllLineTest(Lines, ImageSource, R_cz_pic, Height);
	
	vector<Vec4f> Line90;
	vector<Vec4f> Line0;
	
	ClassifiedByAngle(Lines, Line0, Line90, R_cz_pic, Height);
	//AngleClassifiedTest(Line0, Line90, R_cz_pic, Height, ImageSource);
	
	vector<vector<Vec4f>> LineD0;
	vector<vector<Vec4f>> LineD90;
	
	ClassifiedByDistance(Line0, LineD0);
	ClassifiedByDistance(Line90, LineD90);
	
	//DistanceTest(Line0, R_cz_pic, Height, ImageSource, 0);
	//DistanceTest(Line90, R_cz_pic, Height, ImageSource, 90);
	
	sort(LineD0.begin(), LineD0.end(), Comp0);
	sort(LineD90.begin(), LineD90.end(), Comp90);
	
	//DistanceClassifiedTest(LineD0, R_cz_pic, Height, ImageSource, 0);
	//DistanceClassifiedTest(LineD90, R_cz_pic, Height, ImageSource, 90);
	
	int FunctionFlag;
	int Boundary90Found = 0;
	int Boundary0Found = 0;
	if(LineD90.size()  > 1)
	{
	    Vec4f Boundary90;
	    Boundary90Found = FindWhiteBoundary90(LineD90, LineD0, Boundary90, FunctionFlag);
	    if(Boundary90Found != 0)
	    {
		float Boundary90MiddleX = (Boundary90[0] + Boundary90[2]) / 2;
		float XMeasure = PilotPosition.at<float>(0, 0);
		float XReal;
		if(Boundary90Found == 2)
		{
		    XReal = Line90RealCoordinate[Line90Max] - Boundary90MiddleX;
		    //BoundaryTest(Boundary90, R_cz_pic, Height, ImageSource, 2, FunctionFlag);
		    pilot_position.border[1] = true;
		    pilot_position.border[3] = false;
		}
		else if(Boundary90Found == 4)
		{
		    XReal =  - Boundary90MiddleX;
		    //BoundaryTest(Boundary90, R_cz_pic, Height, ImageSource, 4, FunctionFlag);
		    pilot_position.border[1] = false;
		    pilot_position.border[3] = true;
		}
		else if(Boundary90Found == 5)// 通过位置判断
		{
		    if(XMeasure >= Line90RealCoordinate[Line90Max] / 2)
		    {
			XReal = Line90RealCoordinate[Line90Max] - Boundary90MiddleX;
			//BoundaryTest(Boundary90, R_cz_pic, Height, ImageSource, 2, 3);
			pilot_position.border[1] = true;
			pilot_position.border[3] = false;
		    }
		    else
		    {
			XReal = -Boundary90MiddleX;
			//BoundaryTest(Boundary90, R_cz_pic, Height, ImageSource, 4, 3);
			pilot_position.border[1] = false;
			pilot_position.border[3] = true;
		    }
		}
		Check90 += XMeasure - XReal;
	    }
	    else
	    {
		pilot_position.border[1] = false;
		pilot_position.border[3] = false;
	    }
	}
	else
	{
	    pilot_position.border[1] = false;
	    pilot_position.border[3]  = false;
	}
	/**************************************************************************/
	
	/************************************0度白边界*******************************/
	if(LineD0.size() > 1)
	{
	    Vec4f Boundary0;
	    Boundary0Found = FindWhiteBoundary0(LineD0, LineD90, Boundary0, FunctionFlag);
	    if(Boundary0Found != 0)
	    {
		float Boundary0MiddleY = (Boundary0[1] + Boundary0[3]) / 2;
		float YMeasure = PilotPosition.at<float>(1, 0);
		float YReal;
		if(Boundary0Found == 3)
		{
		    YReal = Line0RealCoordinate[Line0Max] - Boundary0MiddleY;
		    //BoundaryTest(Boundary0, R_cz_pic, Height, ImageSource, 3, FunctionFlag);
		    pilot_position.border[0] = false;
		    pilot_position.border[2] = true;
		}
		else if(Boundary0Found == 1)
		{
		    YReal =  - Boundary0MiddleY;
		    //BoundaryTest(Boundary0, R_cz_pic, Height, ImageSource, 1, FunctionFlag);
		    
		    pilot_position.border[0] = true;
		    pilot_position.border[2] = false;
		}
		else if(Boundary0Found == 5)
		{
		    if(YMeasure >= Line0RealCoordinate[Line0Max] / 2)
		    {
			YReal = Line0RealCoordinate[Line0Max] - Boundary0MiddleY;
			//BoundaryTest(Boundary0, R_cz_pic, Height, ImageSource, 3, 3);
			pilot_position.border[0] = false;
			pilot_position.border[2] = true;
		    }
		    else
		    {
			YReal = -Boundary0MiddleY;
			//BoundaryTest(Boundary0, R_cz_pic, Height, ImageSource, 1, 3);
			pilot_position.border[0] = true;
			pilot_position.border[2] = false;
		    }
		}
		Check0 += YMeasure - YReal;
	    }
	    else
	    {
		pilot_position.border[0] = false;
		pilot_position.border[2] =false;
	    }
	}
	else
	{
	    pilot_position.border[0] = false;
	    pilot_position.border[2] =false;
	}
	/******************************************************************/
	
	/*****************************红边界********************************/
/*	
	Mat ImageRedThreshold;
	
	ThresholdRed(ImageSource, ImageRedThreshold);
	
	vector<Vec4i> LineRed;
	HoughLinesP( ImageRedThreshold, LineRed, 1, CV_PI/180, 80, 30, 50);
	
	//AllLineTest(LineRed, ImageSource, R_cz_pic, Height);
	
	int BoundaryRedFound = 0;
	if(LineRed.size() >= 1)
	{
	    vector<Vec4f> LineRed0;
	    ClassifiedByAngle(LineRed, LineRed0, R_cz_pic, Height);
	    if(LineRed0.size() >= 1)
	    {
		 vector<vector<Vec4f>> LineRedD0;
		 ClassifiedByDistance(LineRed0, LineRedD0);
		 
		 sort(LineRedD0.begin(), LineRedD0.end(), Comp0);
		 
		 
		 Vec4f BoundaryRed;
		 
		 BoundaryRedFound = FindRGBoundary0(LineRedD0, BoundaryRed, 3);
		 
		 if(BoundaryRedFound == 3)
		 {
		     pilot_position.border[2] = true;
		     BoundaryTest(BoundaryRed, R_cz_pic, Height, ImageSource, 3, 0);
		     float BoundaryRedMiddleY;
		     BoundaryRedMiddleY = (BoundaryRed[1] + BoundaryRed[3]) / 2;
		     
		     float YMeasure = PilotPosition.at<float>(1, 0);
		     float YReal = Line0RealCoordinate[Line0Max] - BoundaryRedMiddleY;
		     Check0 += YMeasure - YReal;
		}	
		else
		{
		    pilot_position.border[2] = false;
		}
	    }
	    else
	    {
		pilot_position.border[2] = false;
	    }
	}
	else
	{
	    pilot_position.border[2] = false;
	}*/
	/*****************************************************************/
	
	/*****************************绿边界********************************/
/*	
	Mat ImageGreenThreshold;
	
	ThresholdGreen(ImageSource, ImageGreenThreshold);
	
	vector<Vec4i> LineGreen;
	HoughLinesP( ImageGreenThreshold, LineGreen, 1, CV_PI/180, 80, 30, 50);
	
	//AllLineTest(LineGreen, ImageSource, R_cz_pic, HeightKF);
	
	int BoundaryGreenFound = 0;
	if(LineGreen.size() >= 1)
	{
	    vector<Vec4f> LineGreen0;
	    ClassifiedByAngle(LineGreen, LineGreen0, R_cz_pic, Height);
	    if(LineGreen0.size() >= 1)
	    {
		 vector<vector<Vec4f>> LineGreenD0;
		 ClassifiedByDistance(LineGreen0, LineGreenD0);
		 
		 sort(LineGreenD0.begin(), LineGreenD0.end(), Comp0);
		 
		 
		 Vec4f BoundaryGreen;
		 
		 BoundaryGreenFound = FindRGBoundary0(LineGreenD0, BoundaryGreen, 1);
		 
		 if(BoundaryGreenFound == 1)
		 {
		     pilot_position.border[0] = true;
		     BoundaryTest(BoundaryGreen, R_cz_pic, Height, ImageSource, 1, 0);
		     float BoundaryGreenMiddleY;
		     BoundaryGreenMiddleY = (BoundaryGreen[1] + BoundaryGreen[3]) / 2;
		     
		     float YMeasure = PilotPosition.at<float>(1, 0);
		     float YReal =  - BoundaryGreenMiddleY;
		     Check0 += YMeasure - YReal;
		}	 
	    }
	}
	if(BoundaryGreenFound == 0)
	{
	    pilot_position.border[0] = false;
	}*/
	/*****************************************************************/
	
	/*****************************0度内部直线校正********************************/
	if(Boundary0Found == 0 && LineD0.size() >= 1) // TODO:红绿还是白边    没有找到0度边界
	{
	    vector<vector<Vec4f>> LineD0Filter;
	    LineInsideFilter(LineD0, LineD0Filter);
	 
	    vector<Vec4f> Line0AfterFit;
	    Vec4f Line0Temp;
	    for(int i = 0; i < LineD0Filter.size(); i++)
	    {
		vector<Point2f> PointInLine;
		for(int j = 0; j < LineD0Filter[i].size(); j++)
		{
		    PointInLine.push_back(Point2f(LineD0Filter[i][j][0], LineD0Filter[i][j][1]));
		    PointInLine.push_back(Point2f(LineD0Filter[i][j][2], LineD0Filter[i][j][3]));
		}
		LineFit0(PointInLine, Line0Temp);
		Point2f Point01;
		Point2f Point02;
		Point01.x = Line0Temp[0];
		Point01.y = Line0Temp[1];
		Point02.x = Line0Temp[2];
		Point02.y = Line0Temp[3];
		
		float Length0;
		Length0 = (Point01.x - Point02.x) * (Point01.x - Point02.x) + (Point01.y - Point02.y) * (Point01.y - Point02.y);
		Length0 = sqrt(Length0);
		if(Length0 > LengthThreshold)
		{
		    Line0AfterFit.push_back(Line0Temp);
		}
	    }
	    //AllLineInsideTest(Line0AfterFit, 0, ImageSource, R_cz_pic, Height);
	    float Error0;
	    int Inside0ToCorrect;
	    int FirstLineNum;
	    Inside0ToCorrect = LineInside0(PilotPosition.at<float>(1, 0), Line0AfterFit, Error0, FirstLineNum);
	    if(Inside0ToCorrect == 1)
	    {
		if(Error0 < InsideLineErrorThreshold)
		{
		    //LineInSideCorrectTest(Line0AfterFit, FirstLineNum, Error0, 0, R_cz_pic, Height, ImageSource);
		    Check0 += Error0;
		}
		else
		{
		    //LineInSideCorrectTest(Line0AfterFit, FirstLineNum, Error0, 0, R_cz_pic, Height, ImageSource, 0);
		}
	    }
	    else if(Inside0ToCorrect == 2)
	    {
		//LineInSideCorrectTest(Line0AfterFit, FirstLineNum, 100, 0, R_cz_pic, Height, ImageSource);
	    }
	    else
	    {
		//LineInSideCorrectTest(Line0AfterFit, FirstLineNum, -100, 0, R_cz_pic, Height, ImageSource);
	    }
	}
	/*****************************************************************************/
	
	/****************************90度内部直线校正********************************/
	if(Boundary90Found == 0 && LineD90.size() >= 1) // TODO:   双白还是一白一红   没有找到90度边界
	{
	    vector<vector<Vec4f>> LineD90Filter;
	    LineInsideFilter(LineD90, LineD90Filter);
	    vector<Vec4f> Line90AfterFit;
	    Vec4f Line90Temp;
	    for(int i = 0; i < LineD90Filter.size(); i++)
	    {
		vector<Point2f> PointInLine;
		for(int j = 0; j < LineD90Filter[i].size(); j++)
		{
		    PointInLine.push_back(Point2f(LineD90Filter[i][j][0], LineD90Filter[i][j][1]));
		    PointInLine.push_back(Point2f(LineD90Filter[i][j][2], LineD90Filter[i][j][3]));
		}
		LineFit90(PointInLine, Line90Temp);
		Point2f Point901;
		Point2f Point902;
		Point901.x = Line90Temp[0];
		Point901.y = Line90Temp[1];
		Point902.x = Line90Temp[2];
		Point902.y = Line90Temp[3];
		
		float Length90;
		Length90 = (Point901.x - Point902.x) * (Point901.x - Point902.x) + (Point901.y - Point902.y) * (Point901.y - Point902.y);
		Length90 = sqrt(Length90);
		if(Length90 > LengthThreshold)
		{
		    Line90AfterFit.push_back(Line90Temp);
		}
	    }
	   //AllLineInsideTest(Line90AfterFit, 90, ImageSource, R_cz_pic, Height);
	    float Error90;
	    int Inside90ToCorrect;
	    int FirstLineNum;
	    Inside90ToCorrect = LineInside90(PilotPosition.at<float>(0, 0), Line90AfterFit, Error90, FirstLineNum);
	    if(Inside90ToCorrect ==  1)
	    {
		if(Error90 < InsideLineErrorThreshold)
		{
		    //LineInSideCorrectTest(Line90AfterFit, FirstLineNum, Error90, 90, R_cz_pic, Height, ImageSource);
		    Check90 += Error90;
		}
		else
		{
		    //LineInSideCorrectTest(Line90AfterFit, FirstLineNum, Error90, 90, R_cz_pic, Height, ImageSource, 0);
		}
	    }
	    else if(Inside90ToCorrect == 2)
	    {
		//LineInSideCorrectTest(Line90AfterFit, FirstLineNum, 100, 90, R_cz_pic, Height, ImageSource);
	    }
	    else
	    {
		//LineInSideCorrectTest(Line90AfterFit, FirstLineNum, -100, 90, R_cz_pic, Height, ImageSource);
	    }
	} 
	/*****************************************************************************/
	
	CurrentImageNumber = ImageNumber;
	
	rate.sleep();
	double RosTime2 = (double)ros::Time::now().nsec * 1e-9+ (double)ros::Time::now().sec;
	//double RosTime2 = (double)ros::Time::now().toSec();
	double Frequency = 1.0 / (RosTime2 - RosTime1);
	cout <<  Frequency  << "Hz"<< endl;
	/**************************************保存频率*********************************/
	FrequencyF << Frequency << endl;
	/*******************************************************************************/
    }
}
