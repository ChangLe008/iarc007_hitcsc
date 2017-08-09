#include <ros/ros.h>

#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv; 
using namespace std;

int BoardW = 11;
int BoardH = 7;
int BoardSize = 70;
Size ChessBoardSize = Size(BoardW, BoardH);
ros::Subscriber wide_angle_sub;

Mat Image;
void wide_angle_callback(sensor_msgs::Image img_msg)
{ 
    cv_bridge::CvImagePtr img_bridge_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
    img_bridge_ptr->image.copyTo(Image);
}
int main(int argc, char** argv)
{
    ros::init(argc,argv,"external_parameter");
    ros::NodeHandle my_node;
    ros::Rate rate(30);
    
    CvMat* Intrinsic0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/NewIntrinsics.xml");
    Mat Intrinsic = Mat(Intrinsic0, true);
    
    vector<Point2f> TempCorners;
    vector<Point3f> Object;
    Mat Rvec;
    Mat Tvec;
    int NPoints = BoardW * BoardH;
    for (int j = 0; j < NPoints; j++)
    {
        Object.push_back(Point3f((j % BoardW) * BoardSize, (j / BoardW) * BoardSize, 0)); 
    }
    char Key = 'q';
    wide_angle_sub = my_node.subscribe<sensor_msgs::Image>("/iarc/wide_angle_video", 2, wide_angle_callback);
    while(ros::ok())
    {
	ros::spinOnce();
	if(!Image.data)
	{
	    cout << "没有图片" << endl;
	    continue;
	}
	imshow("image", Image);
	Mat ImageGray;
	cvtColor(Image, ImageGray, CV_BGR2GRAY);
	
	bool FindEnoughCorners = findChessboardCorners(ImageGray, ChessBoardSize, TempCorners, 3);
	
	if(FindEnoughCorners)
	{
	     cornerSubPix(ImageGray, TempCorners, cvSize(5, 5), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	     drawChessboardCorners(Image, ChessBoardSize, TempCorners, FindEnoughCorners);
	     imshow("corner_image", Image);
	     solvePnP(Object, TempCorners, Intrinsic, Mat(), Rvec, Tvec, false, ITERATIVE);
	     Mat Rotation(3,3, CV_64FC1);
	     Rodrigues(Rvec, Rotation);
	     CvMat* Rotation1 = cvCreateMat(3, 3, CV_64FC1);
	     CvMat temp = Rotation;
	     cout << "hahahahhhhah" << endl;
	     cvCopy(&temp, Rotation1);
	     cvSave("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/Rotation.xml", Rotation1);
	     cvReleaseMat(&Rotation1);
	 }
    }
}