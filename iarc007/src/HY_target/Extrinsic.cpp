#include <iostream>
#include <fstream>
//#include <io.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <dji_sdk/AttitudeQuaternion.h>

#include <network_client/Optitrack_data.h>
using namespace std;
using namespace cv;

#define PI 3.14159265

Mat img_sub;

float yaw,roll,pitch;
float opt_roll, opt_yaw, opt_pitch;
float wx,wy,wz;

network_client::Optitrack_data optitrack_data;

void Quaternion_To_Euler(Mat q_att1, Mat& att1)
{
		float r11,r12,r21,r31,r32,r1,r2,r3;
		float q[4] = { q_att1.at<float>(0,0),q_att1.at<float>(1,0),q_att1.at<float>(2,0),q_att1.at<float>(3,0)};
    	r11 = 2.0f *(q[1] * q[2] + q[0] * q[3]);
    	r12 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2]  - q[3] * q[3] ;
    	r21 = -2.0f * (q[1] * q[3] - q[0] * q[2]);
    	r31 = 2.0f *( q[2] * q[3] + q[0]  * q[1]);
    	r32 = q[0] * q[0] - q[1] * q[1]  - q[2] * q[2] + q[3] * q[3] ;
    	float yaw= atan2( r11, r12 );
    	float pitch = asin( r21 );
    	float roll = atan2( r31, r32 );
    	att1.at<float>(0,0)=roll;
    	att1.at<float>(1,0)=pitch;
    	att1.at<float>(2,0)=yaw;

//printf("%f  \n",asin(r21));
//printf("%f  \n",r21);
}

//euler to matrix  the matrix is from body to ground
void Euler_To_Matrix(float roll, float pitch, float yaw, Mat& R)
{
	float cp = cosf(pitch);
	float sp = sinf(pitch);
	float sr = sinf(roll);
	float cr = cosf(roll);
	float sy = sinf(yaw);
	float cy = cosf(yaw);
	
	R.at<float>(0,0)=cp * cy;
	R.at<float>(0,1)=((sr * sp * cy) - (cr * sy));
	R.at<float>(0,2)=((cr * sp * cy) + (sr * sy));
	R.at<float>(1,0)=(cp * sy);
	R.at<float>(1,1)=((sr * sp * sy) + (cr * cy));
	R.at<float>(1,2)=( (cr * sp * sy) - (sr * cy));
	R.at<float>(2,0)=-sp;
	R.at<float>(2,1)=sr * cp;
	R.at<float>(2,2)=cr * cp;
//printf("%f	%f	%f\n",cvmGet(R,0,0),cvmGet(R,0,1),cvmGet(R,0,2));
}

void mono_sub_callback(sensor_msgs::Image img_msg)
{ 
	
	cv_bridge::CvImagePtr img_bridge_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);       
	img_bridge_ptr->image.copyTo(img_sub);   
}

/* imu from aerial pilot*/
FILE *yaws_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/yaws.txt","w");

void attitude_quaternion_subscriber_callback(dji_sdk::AttitudeQuaternion attitude_quaternion)
{

	Mat q_att(4,1,CV_32FC1);
	Mat att(3,1,CV_32FC1);
	Mat R_imu(3,3,CV_32FC1);
	
	q_att.at<float>(0,0)=attitude_quaternion.q0;
	q_att.at<float>(1,0)=attitude_quaternion.q1;
	q_att.at<float>(2,0)=attitude_quaternion.q2;
	q_att.at<float>(3,0)=attitude_quaternion.q3;
	
	wx=attitude_quaternion.wx/PI*190.0;
	wy=attitude_quaternion.wy/PI*180.0;
	wz=attitude_quaternion.wz/PI*180.0;
	//printf("aerial pilot : %f %f %f %f\n",attitude_quaternion.q0,attitude_quaternion.q1,attitude_quaternion.q2,attitude_quaternion.q3);
	Quaternion_To_Euler(q_att, att);
	
	
	roll = att.at<float>(0,0);
	pitch = att.at<float>(1,0);
	yaw = att.at<float>(2,0);
	
	Euler_To_Matrix(roll, pitch, yaw, R_imu); 
	
	cout<<R_imu<<endl;
	Rodrigues(R_imu,att);
	att = att/PI*180.0;
	roll = att.at<float>(0,0);
	pitch = att.at<float>(1,0);
	yaw = att.at<float>(2,0);
	//printf(" %f  %f \n",ros::Time::now().sec+ros::Time::now().nsec*(1e-9),yaw*180/PI);
	

}
///////////////////////////////////////////////////////////////////
void position_callback(network_client::Optitrack_data position)
{
	Mat R_opt(3,3,CV_32FC1);
	Mat att(3,1,CV_32FC1);
	
	
	optitrack_data = position; 
	opt_roll = optitrack_data.roll;
    opt_yaw = optitrack_data.yaw;
    opt_pitch = optitrack_data.pitch; 
    
    Euler_To_Matrix(opt_roll, opt_pitch, opt_yaw, R_opt);  
    
    Rodrigues(R_opt,att);
	att = att/PI*180.0;
	opt_roll = att.at<float>(0,0);
	opt_pitch = att.at<float>(1,0);
	opt_yaw = att.at<float>(2,0);
}  


int main(int argc, char** argv)
{
	//************************************
	
	
	ros::init(argc,argv,"mono_video_sub");
	ros::NodeHandle my_node;
	
	FILE *attitude_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/doc/attitude_txt.txt","w");
	if(!attitude_txt)
	{
		cout << "File open failed !" << std::endl;
		return 0;
	}
	ros::Subscriber mono_video_sub = my_node.subscribe<sensor_msgs::Image>("/iarc/wide_angle_video",2,mono_sub_callback);
	ros::Subscriber attitude_quaternion_subscriber = my_node.subscribe<dji_sdk::AttitudeQuaternion>("dji_sdk/attitude_quaternion", 1, attitude_quaternion_subscriber_callback);
	ros::Subscriber Opti_pos=my_node.subscribe("/network_client/network_optitrack_data", 10, position_callback);; 
	//************************************
	int board_w = 11;
	int board_h = 7;
	int cap_num=0;
	int boardSize = 70;//mm
	if(argc>2)
	{
		board_w=atoi(argv[1]);
		board_h=atoi(argv[2]);
	}
	if(argc>3)
	{
		cap_num=atoi(argv[3]);
	}
	cout << "board_w=" << board_w << endl;
	cout << "board_h=" << board_h << endl;
	
	
	//VideoCapture capture(cap_num);
	const int NPoints = board_w * board_h;//棋盘格内角点总数

    Mat image,grayimage;
    Size ChessBoardSize = cv::Size(board_w, board_h);
    vector<Point2f> tempcorners;
	vector<Point3f> object;
	
    for (int j = 0; j < NPoints; j++)
    {
        object.push_back(Point3f((j % board_w) * boardSize, (j / board_w) * boardSize, 0)); 
    }

    Size corrected_size(640, 480);
    Mat mapx, mapy;
    Mat corrected;

    //ofstream intrinsicfile("intrinsics.txt");
    //ofstream disfile("dis_coeff.txt");
  Mat intrinsic(3, 3, CV_32FC1,Scalar(0));
  ifstream IntrinsicF("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/NewIntrinsics.xml"); 
  
  if(!IntrinsicF.is_open())
  {
    cout << "相机内参文件不存在！"<< endl;
    exit(-1);
  }
  else
  {
    for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < 3; j++)
      {
		IntrinsicF >> intrinsic.at<float>(i, j);
      }
    }
  }
  cout << "intrinsic\t" << intrinsic<< endl;
  ifstream DistortionF("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/Distortion.xml");
  Mat distortion(1, 5, CV_32FC1,Scalar(0));
  if(!DistortionF.is_open())
  {
    cout << "相机校正文件不存在！" << endl;
    exit(-1);
  }
  else
  {
    for(int i = 0; i < 5; i++)
    {
      DistortionF >> distortion.at<float>(0, i);
    }
  }
  cout << "distortion\t" << distortion << endl;
	Mat rvec;
	Mat tvec;
	
	while(ros::ok())
	{
		
		//capture >> image;
 		
		if(img_sub.data)
		{
			img_sub.copyTo(image);
			if (image.empty())
		    {
		    	cout<<"empty"<<endl;
		    	continue;
		    	//break;
		    }
		       
		    cvtColor(image,grayimage,CV_BGR2GRAY);
		    bool find_corners_result = findChessboardCorners(grayimage, ChessBoardSize, tempcorners, 3);
		    imshow("image",image);
		    if (find_corners_result)
		    {
		    
		        cornerSubPix(grayimage, tempcorners, cvSize(5, 5), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		        drawChessboardCorners(image, ChessBoardSize, tempcorners, find_corners_result);
		        imshow("corner_image", image);

			    solvePnP(object, tempcorners, intrinsic, distortion, rvec, tvec, false, ITERATIVE);
			    cout<<rvec*180/PI<<endl;
			    Mat rotation;
			    Rodrigues(rvec,rotation);
			    cout<<"waican"<<endl; 
    			cout<<rotation<<endl;
			    rvec = rvec/PI*180.0;
			    fprintf(attitude_txt," %f\t%f\t%f\t%f\t%lf\t%lf\t%lf\t%f\t%f\t%f\t%lf\t%lf\t%lf\t%f\t%f\t%f\n",ros::Time::now().sec+ros::Time::now().nsec*(1e-9),roll,pitch,yaw,rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0),wx,wy,wz,tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0),opt_roll,opt_pitch,opt_yaw);
		    }
		    tempcorners.clear();
		}

        
        char c=waitKey(10);
        if(c==27)
        	break;
        ros::spinOnce();
	}
	fclose(attitude_txt);
	
	return 0;
}
