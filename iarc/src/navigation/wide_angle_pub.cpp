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

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wide_angle_pub");
  ros::NodeHandle my_node;
  
  VideoCapture Capture(0);
   
  if(!Capture.isOpened())
  {
    cout << "No Camera!" << endl;
    return -1;
  }
  Mat Image0;
  Capture >> Image0;
  if(!Image0.data)
  {
    cout << "No Image!" << endl;
    return -1;
  }
  CvMat* Intrinsic0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/Intrinsics.xml");
  Mat Intrinsic = Mat(Intrinsic0, true);
  CvMat* Distortion0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/Distortion.xml");
  Mat Distortion = Mat(Distortion0, true);
  CvMat* NewIntrinsic0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/NewIntrinsics.xml");
  Mat NewIntrinsic = Mat(Intrinsic0, true);
  
  ros::Publisher wide_angle_pub = my_node.advertise<sensor_msgs::Image>("/iarc/wide_angle_video",5);
  
  Mat R=Mat::eye(3,3,CV_32FC1);
  Size size=Size(640,480);
  Mat Map1(640, 480, CV_32FC1);
  Mat Map2(640, 480, CV_32FC1);
  initUndistortRectifyMap( Intrinsic, Distortion, R,  NewIntrinsic, size, CV_32FC1,  Map1,  Map2);
  
  cv_bridge::CvImage ImageBridge;
  ros::Rate rate(30);
  cout << "wide_angle_pub starts!" << endl;

  Mat Image;
  while(ros::ok())
  {
      Capture >> Image0;
      imshow("原图", Image0);
      remap(Image0 ,Image, Map1, Map2, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));
      
      Image.copyTo(ImageBridge.image);
      ImageBridge.header.frame_id = "wide_angle_video";
      ImageBridge.header.stamp = ros::Time::now();
      ImageBridge.encoding = sensor_msgs::image_encodings::BGR8;
      wide_angle_pub.publish(ImageBridge.toImageMsg());
      imshow("wide_angle_video", Image);
      waitKey(1);
      rate.sleep();
  }
  cout << "wide_angle_pub ends!" << endl;
  cvReleaseMat(&Intrinsic0);
  cvReleaseMat(&Distortion0);
  return 0;
}
