#include <ros/ros.h>

#include <sys/time.h>
#include <unistd.h>
#include <time.h>
#include <fstream> 


#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "iarc007/mjh_pos/common.h"
#include "iarc007/mjh_pos/threshold_image.h"
#include "iarc007/mjh_pos/mjh_math.h"
#include "iarc007/mjh_pos/find_line.h"

Mat ImageSource;
Mat ImageSource1;

Mat PilotPosition = Mat(3, 1, CV_32FC1, Scalar(0));  

Mat R_g_bQuaternion = Mat(4, 1, CV_32FC1, Scalar(0));
Mat R_g_bEurler = Mat(3, 1, CV_32FC1, Scalar(0));
Mat R_g_b = Mat(3, 3, CV_32FC1, Scalar(0));
Mat R_cz_g = Mat(3, 3, CV_32FC1, Scalar(0));
Mat R_cz_b = Mat::ones(3, 3, CV_32FC1);
Mat R_ca_cz = Mat::ones(3, 3, CV_32FC1);
Mat R_cz_ca = Mat::ones(3, 3, CV_32FC1);


Mat R_g_cz = Mat(3, 3, CV_32FC1, Scalar(0));
Mat R_ca_chess = Mat(3, 3, CV_32FC1, Scalar(0));
Mat R_chess_b = Mat(3, 3, CV_32FC1, Scalar(0));
Mat R_ca_b = Mat(3, 3, CV_32FC1, Scalar(0));
Mat R_b_ca = Mat(3, 3, CV_32FC1, Scalar(0));
Mat R_ca_pic = Mat(3, 3, CV_32FC1, Scalar(0)); 
Mat R_cz_pic = Mat(3, 3, CV_32FC1, Scalar(0));
Mat R_b_pic = Mat(3, 3, CV_32FC1, Scalar(0));

float Height = 0;
float StartX = 1;
float StartY = 1;
float MiddleX = 2.5;
float MiddleY = 3.5;

int PixelThreshold = 200;

ros::Subscriber mono_video_sub;
ros::Subscriber position_sub;
ros::Subscriber imu_sub;
ros::Subscriber ultrasonic_sub;


bool FirstFlag = true;
float FirstX;
float FirstY;
float FirstZ;

int ImageNumber = 0;

void mono_sub_callback(sensor_msgs::Image img_msg)
{ 
	cv_bridge::CvImagePtr img_bridge_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);       
	img_bridge_ptr->image.copyTo(ImageSource);   
	ImageSource.copyTo(ImageSource1);
	ImageNumber++;
}
void imu_callback(const geometry_msgs::TransformStamped& g_imu)
{ 
	R_g_bQuaternion.at<float>(0, 0) = g_imu.transform.rotation.w;
	R_g_bQuaternion.at<float>(1, 0) = g_imu.transform.rotation.x;
	R_g_bQuaternion.at<float>(2, 0) = g_imu.transform.rotation.y;
	R_g_bQuaternion.at<float>(3, 0) = g_imu.transform.rotation.z;

	Quaternion2Euler(R_g_bQuaternion, R_g_bEurler);
	Euler2Matrix(R_g_bEurler, R_g_b);
	
	R_cz_b = R_cz_g * R_g_b;
}
void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
	PilotPosition.at<float>(0, 0) = g_pos.vector.x;
	PilotPosition.at<float>(1, 0) = g_pos.vector.y;
	PilotPosition.at<float>(2, 0) = g_pos.vector.z;
	
	PilotPosition = R_cz_g * PilotPosition;
	
	if(FirstFlag)
	{ 
		FirstX = PilotPosition.at<float>(0, 0);
		FirstY = PilotPosition.at<float>(1, 0);
		FirstZ = PilotPosition.at<float>(2, 0);
		FirstFlag = false;
	}
	//cout << "Position0: " << PilotPosition.at<float>(0, 0) << "\t" << PilotPosition.at<float>(1, 0) << endl;
	PilotPosition.at<float>(0, 0) += StartX - FirstX; 
	PilotPosition.at<float>(1, 0) += StartY - FirstY;
	
	//cout << "Position: " << PilotPosition.at<float>(0, 0) << "\t" << PilotPosition.at<float>(1, 0) << endl;
}
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{ 
	if(g_ul.ranges[0] > 0.)
	{
		Height = g_ul.ranges[0];
		PixelThreshold = 40 / Height;
		cout << PixelThreshold << endl;
	}

} 
void Point_cz_pic(Point _PointInca, Point3f &_PointIncz, Mat R, float h)
{
	Mat PointInca = Mat(3, 1, CV_32FC1, Scalar(0));
	
	PointInca.at<float>(0, 0) = _PointInca.x;
	PointInca.at<float>(1, 0) = _PointInca.y;
	PointInca.at<float>(2, 0) = 1;
	
	Mat PointIncz = R * PointInca;
	
	float Scale = PointIncz.at<float>(2, 0) / h;
	
	_PointIncz.x = PointIncz.at<float>(0, 0) / Scale;
	_PointIncz.y = PointIncz.at<float>(1, 0) / Scale;
	
	_PointIncz.z =  PointIncz.at<float>(2, 0) / Scale;	
	
}
void Point_pic_cz(Point3f _PointIncz, Point &PointInpic, Mat R)
{
	Mat PointIncz = Mat(3, 1, CV_32FC1, Scalar(0));
	
	PointIncz.at<float>(0, 0) = _PointIncz.x;
	PointIncz.at<float>(1, 0) = _PointIncz.y;
	PointIncz.at<float>(2, 0) = _PointIncz.z;
	Mat RInv = R.inv();
	Mat Point = Mat(3, 1, CV_32FC1, Scalar(0));
	Point = RInv * PointIncz;
	
	float Scale = Point.at<float>(2, 0);
	float x = Point.at<float>(0, 0) / Scale;
	float y = Point.at<float>(1, 0) / Scale;
	PointInpic.x = x < ImageCols ? x : ImageCols;
	PointInpic.y = y < ImageRows ? y : ImageRows;
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "mjh_vision_pos");
	ros::NodeHandle my_node;
	
	ifstream Q_Init("/home/hitcsc/catkin_ws/src/iarc/cfg/init_ori.cfg");
	if(!Q_Init.is_open())
	{
		cout << "Can't open init_ori.cfg!" << endl;
		exit(-1);
	}
	
	string s;
	
	Mat R_g_czQuaternion = Mat(4, 1, CV_32FC1, Scalar(0));
	Mat R_g_czEurler = Mat(3, 1, CV_32FC1, Scalar(0));
	
	for(int i = 0; i < 4; i++)
	{
		Q_Init >> s;
		R_g_czQuaternion.at<float>(i, 0) = atof(s.c_str());
	}
	
	Quaternion2Euler(R_g_czQuaternion, R_g_czEurler);
	Euler2Matrix(R_g_czEurler, R_g_cz);
	R_cz_g = R_g_cz.t();
	
	CvMat* R_ca_chess0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/Rotation.xml");
	R_ca_chess = Mat(R_ca_chess0, true);
	if(!R_ca_chess.data)
	{
		cout << "Can't open Rotation.xml" << endl;
		exit(-1);
	}
	CvMat* R_chess_b0=(CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/R_chess_b.xml");
	R_chess_b = Mat(R_chess_b0, true); 
	if(!R_chess_b.data)
	{
		cout << "Can't open R_chess_b.xml!" << endl;
		exit(-1);
	};
	
	R_ca_b = R_ca_chess * R_chess_b;
	R_b_ca = R_ca_b.t();
	PilotPosition.at<float>(1, 0) += StartY;
	CvMat* Intrinsic0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/Intrinsics.xml");
	
	Mat Intrinsic = Mat(Intrinsic0, true);
	
	R_ca_pic = Intrinsic.inv();
	
	R_b_pic = R_b_ca * R_ca_pic;
	
	mono_video_sub = my_node.subscribe<sensor_msgs::Image>("/iarc007/mono_video1",2,mono_sub_callback);
	position_sub = my_node.subscribe("/guidance/position",20, position_callback);
	imu_sub = my_node.subscribe("/guidance/imu", 10, imu_callback);
	ultrasonic_sub = my_node.subscribe("/guidance/ultrasonic", 1, ultrasonic_callback);
	
	
	ros::Rate rate(5);
	int CurrentImageNumber = 0;
	int N = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if(!ImageSource.data)
		{
			cout << "No Image!" << endl;
			continue;
		}
		if(CurrentImageNumber == ImageNumber)
		{
			cout << "CurrentImageNumber: " << CurrentImageNumber <<endl;
			cout << "ImageNumber: " << ImageNumber <<endl;
			cout << "Next image dosen't come!" << endl;
			continue;
		}
		/*CurrentImageNumber = ImageNumber;
		char Name[100];
		sprintf(Name, "/home/hitcsc/pic/%d.jpg", N);
		/*imwrite(Name, ImageSource);
		imshow("hahah", ImageSource);
		waitKey(1);
		N++;
		continue;
		ImageSource = imread(Name);
		ImageSource.copyTo(ImageSource1);
		cout << N << endl;
		N++;*/
		GaussianBlur(ImageSource, ImageSource ,Size(7, 7), 0, 0);
		
		bool LineFinded;
		vector<Vec4i> OutFitLine;
		int FindLineNumber = 0;
		//白线
		LineFinded = FindLine(ImageSource, OutFitLine, FindWhiteLine, PixelThreshold);
		if(LineFinded)
		{
			cout << "White Line" << endl;
			DrawLine(ImageSource1, OutFitLine);
			FindLineNumber += OutFitLine.size();
		}
		//红线
		LineFinded = FindLine(ImageSource, OutFitLine, FindRedLine, PixelThreshold);
		if(LineFinded)
		{
			DrawLine(ImageSource1, OutFitLine);
			FindLineNumber += OutFitLine.size();
		}
		//绿线
		LineFinded = FindLine(ImageSource, OutFitLine, FindGreenLine, PixelThreshold);
		if(LineFinded)
		{
			DrawLine(ImageSource1, OutFitLine);
			FindLineNumber += OutFitLine.size();
		}
		R_cz_pic = R_cz_b * R_b_pic;
		cout << "Find  " << FindLineNumber << " lines in all!" << endl;
		imshow("直线", ImageSource1);
		//imshow("所有直线", Image);
		char Key = waitKey(1);
		/*while(Key != 'q' && Key != 'p')
		{
			
		}
		if(Key == 'q')
		{
			N = N - 2;
		}*/
	}
	cvReleaseMat(&R_ca_chess0);
	cvReleaseMat(&R_chess_b0);
	cvReleaseMat(&Intrinsic0);
	return 0;
}
