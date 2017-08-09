#include <iostream>
#include <string>
#include <fstream>

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv/cv.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

int WIDTH=640;
int HEIGHT=480;

#define IMAGE_SIZE (WIDTH*HEIGHT)

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"mono_video_pub");
	ros::NodeHandle nh;

       
	VideoCapture cap;

	
	{
		cout<<string(argv[1])<<endl;
		cap.open(atoi(argv[1]));

	}
		
	if( !cap.isOpened() )
	{
		cout << endl << "Can not open camera or video file" << endl;
		return -1;
	}    

	// Mat tmp_frame;
	// cap >> tmp_frame;

	// if(!tmp_frame.data)
	// {
	// 	cout << "can not read data from the video source" << endl;
	// 	return -1;
	// }

	//CvCapture* capture=cvCreateCameraCapture(7);
	//VideoCapture cap(7);

	//if(!capture)
	if(!cap.isOpened())
	{  
		cout << "No stream in hy_opencv_pub" << endl;
		return -1;
	}
	/*
	IplImage* image = cvQueryFrame(capture);  
	//cvCreateImage(cvSize(tmp_frame.cols,tmp_frame.rows),IPL_DEPTH
	if(!image)
	{ 
		cout << "No image in hy_opencv_pub" << endl; 
		return -1;
	}*/

	ros::Publisher mono_video_pub = nh.advertise<sensor_msgs::Image>("/iarc007/mono_video",5);
	ros::Publisher mono_video_pub1 = nh.advertise<sensor_msgs::Image>("/iarc007/mono_video1",5);

	cv_bridge::CvImage img_bridge;
	cv_bridge::CvImage img_bridge1; 

	//Mat tmp_frame(image->width,image->height,CV_8UC3, Scalar(0,0,255));
	//Mat tmp_frame1(image->width,image->height,CV_8UC3, Scalar(0,0,255)); 

	Mat tmp_frame;
	//************************************************************************************
	cv::Matx33d intrinsics_f,R_f;//z:相机内参
    cv::Matx33d P_f;
    cv::Vec4d distortion_coeff;//z:相机畸变系数
    cv::Matx33d new_intrinsics;//z:相机新内参
    
    Size corrected_size(640, 480); 
    Mat mapx_f, mapy_f;
    Mat corrected;
    
    ifstream intrinsicfile("/home/hitcsc/catkin_ws/src/iarc007/doc/intrinsics_front5_16.txt");
    ifstream distfile("/home/hitcsc/catkin_ws/src/iarc007/doc/dis_coeff_front5_16.txt");
    
   
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			intrinsicfile >> intrinsics_f(i,j);
			new_intrinsics(i,j) = intrinsics_f(i,j);
			cout<<intrinsics_f(i,j)<<"\t";
			
		}
		cout<<endl;
		
	}
	for(int i=0; i<4; ++i)
	{
		distfile >> distortion_coeff(i);
	}
	intrinsicfile.close();
	distfile.close();
	
	
	R_f = cv::Matx33d::eye();
	fisheye::estimateNewCameraMatrixForUndistortRectify(intrinsics_f, distortion_coeff,corrected_size,R_f,P_f,0.0,corrected_size,1.0);
	
	fisheye::initUndistortRectifyMap(intrinsics_f, distortion_coeff, R_f, new_intrinsics, corrected_size, CV_16SC2, mapx_f, mapy_f);
	
	
	cout << "-----calibrate successfully----" << endl;
	//**************************************************************************************************//
	ros::Rate rate(30);
   
	//cvNamedWindow("image",1);
    	cout << "hy_opencv_pub: start" << endl;

	while(ros::ok())
	{
		cap >> tmp_frame;
		
		//image=cvQueryFrame(capture);      
		
		if( !tmp_frame.data )
		{
		 	cout << "No Data from Mono Cam Anymore" << endl;
			break;  
		}
		
		//tmp_frame = Mat(image);
		
		//tmp_frame.copyTo(img_bridge.image);

		//img_bridge.header.frame_id = "mono_video";
		//img_bridge.header.stamp = ros::Time::now();
		//img_bridge.encoding = sensor_msgs::image_encodings::BGR8;

		//mono_video_pub.publish(img_bridge.toImageMsg());

		//imshow("mono_video_pub0",tmp_frame);

		//cvRemap(image,image,mapx,mapy);
		remap(tmp_frame, corrected, mapx_f, mapy_f, INTER_LINEAR, BORDER_TRANSPARENT);

		//tmp_frame1 = Mat(image);

		//imshow("mono_video_pub",tmp_frame1);
		int keycode = waitKey(1);
		if( keycode == 27 )
		    break;

		//tmp_frame1.copyTo(img_bridge1.image);
		corrected.copyTo(img_bridge1.image);
		//imshow("mono_video_pub",corrected);
		img_bridge1.header.frame_id = "mono_video1";
		img_bridge1.header.stamp = ros::Time::now();
		img_bridge1.encoding = sensor_msgs::image_encodings::BGR8;

		mono_video_pub1.publish(img_bridge1.toImageMsg());

		rate.sleep();	
	}

	cout << "hy_opencv_pub: stop" << endl;


	return 0;
}
