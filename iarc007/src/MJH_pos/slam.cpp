#include <ros/ros.h>

#include <sys/time.h>
#include <unistd.h>
#include <time.h>
#include <fstream> 

#include   <iostream>  

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <network_client/Optitrack_data.h>  


using namespace std ;
using namespace cv;

Mat ImageSource;
Mat imu  = Mat(3,1,CV_32FC1);
Mat optitrack  = Mat(6,1,CV_32FC1);

int ImageNumber = 0;

ros::Subscriber mono_video_sub;
ros::Subscriber position_sub;
ros::Subscriber optitrack_sub;
ros::Subscriber imu_sub;
ros::Subscriber left_image_sub;

/*void mono_sub_callback(const sensor_msgs::ImageConstPtr& left_img)
{ 
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
        cv_ptr->image.copyTo(ImageSource);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    ImageNumber++;
}*/
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
        cv_ptr->image.copyTo(ImageSource);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    ImageNumber++;
}
void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
    cout << "hhahah" << endl;
	imu.at<float>(0, 0) = g_pos.vector.x;
	imu.at<float>(1, 0) = g_pos.vector.y;
	imu.at<float>(2, 0) = g_pos.vector.z;
}

void optitrack_callback(network_client::Optitrack_data position)
{
	optitrack.at<float>(0,0) = position.posx;
	optitrack.at<float>(1,0) = position.posy;
	optitrack.at<float>(0,0) = position.posz;
	
	optitrack.at<float>(0,0) = position.roll;
	optitrack.at<float>(0,0) = position.yaw;
	optitrack.at<float>(0,0) = position.pitch;
	
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "mjh_vision_pos");
	ros::NodeHandle my_node;
		
	//mono_video_sub = my_node.subscribe("/guidance/left_image",2,mono_sub_callback);
	position_sub = my_node.subscribe("/guidance/position",20, position_callback);
	optitrack_sub = my_node.subscribe("/network_client/network_optitrack_data",10, optitrack_callback);
	left_image_sub = my_node.subscribe("/guidance/left_image",  10, left_image_callback);

	
	ros::Rate rate(5);
	int CurrentImageNumber = 0;
	
	ofstream imuFile;
	imuFile.open("/home/hitcsc/lifeng/imu.txt");
	
	ofstream optitrackFile;
	optitrackFile.open("/home/hitcsc/lifeng/optitrack.txt");
	char ImageName[100];
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
			cout << "Next Image does not come!" << endl;
			continue;
		}
		
		sprintf(ImageName,"/home/hitcsc/lifeng/image/%d.jpg",N);
		imwrite(ImageName, ImageSource);
		imshow("haha",ImageSource);
		waitKey(1);
		
		for(int i= 0; i < 2;i++)
		{
			imuFile << imu.at<float>(i,0) << "\t";
		}
		imuFile << imu.at<float>(2,0) << endl;
		
		for(int i = 0; i < 5; i++)
		{
			optitrackFile << optitrack.at<float>(i, 0) << "\t";
		}
		optitrackFile << optitrack.at<float>(5, 0) << endl;
		N++;
		cout << N << endl;
		
		CurrentImageNumber = ImageNumber;
	}
	return 0;
}
