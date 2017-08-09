#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

#include <iostream>
#include <string>

#include <iarc007/obstacle.h>
#include <iarc007/zy_obstacle/obs_utility.h>

#define CAMERA_NUM 12
#define DEPTH_NUM 6

using namespace std;
using namespace cv;

obs_camera_index img_left_index = obs_g1_left;
obs_camera_index img_right_index = obs_g1_right;
obs_depth_index img_depth_index = obs_g1;

cv::Mat img_temp[CAMERA_NUM];
cv::Mat img_depth[DEPTH_NUM];


/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
        cv_ptr->image.copyTo(img_temp[img_left_index]);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

/* right greyscale image */
void right_image_callback(const sensor_msgs::ImageConstPtr& right_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::MONO16);
        cv_ptr->image.copyTo(img_temp[img_right_index]);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

/* depth greyscale image */
void depth_image_callback(const sensor_msgs::ImageConstPtr& depth_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::MONO16);
        cv_ptr->image.convertTo(img_depth[img_depth_index],CV_8UC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

/* imu */
void imu_callback(const geometry_msgs::TransformStamped& g_imu)
{ 
    printf( "frame_id: %s stamp: %d\n", g_imu.header.frame_id.c_str(), g_imu.header.stamp.sec );
    printf( "imu: [%f %f %f %f %f %f %f]\n", g_imu.transform.translation.x, g_imu.transform.translation.y, g_imu.transform.translation.z, 
						g_imu.transform.rotation.x, g_imu.transform.rotation.y, g_imu.transform.rotation.z, g_imu.transform.rotation.w );
}

/* velocity */
void velocity_callback(const geometry_msgs::Vector3Stamped& g_vo)
{ 
    printf( "frame_id: %s stamp: %d\n", g_vo.header.frame_id.c_str(), g_vo.header.stamp.sec );
    printf( "velocity: [%f %f %f]\n", g_vo.vector.x, g_vo.vector.y, g_vo.vector.z );
}

/* obstacle distance */
void obstacle_distance_callback(const sensor_msgs::LaserScan& g_oa)
{ 
    printf( "frame_id: %s stamp: %d\n", g_oa.header.frame_id.c_str(), g_oa.header.stamp.sec );
    printf( "obstacle distance: [%f %f %f %f %f]\n", g_oa.ranges[0], g_oa.ranges[1], g_oa.ranges[2], g_oa.ranges[3], g_oa.ranges[4] );
}

/* ultrasonic */
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{ 
    printf( "frame_id: %s stamp: %d\n", g_ul.header.frame_id.c_str(), g_ul.header.stamp.sec );
    for (int i = 0; i < 5; i++)
        printf( "ultrasonic distance: [%f]  reliability: [%d]\n", g_ul.ranges[i], (int)g_ul.intensities[i] );
}

/* motion */
void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
	printf("frame_id: %s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
	for (int i = 0; i < 5; i++)
		printf("global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"zy_obstacle");
    ros::NodeHandle my_node;
        
    ros::Publisher obs_pub = my_node.advertise<iarc007::obstacle>("zy_obstacle",5);
    
    ros::Subscriber left_image_sub        = my_node.subscribe("/guidance/left_image",  10, left_image_callback);
    ros::Subscriber right_image_sub       = my_node.subscribe("/guidance/right_image", 10, right_image_callback);
    ros::Subscriber depth_image_sub       = my_node.subscribe("/guidance/depth_image", 10, depth_image_callback);
    ros::Subscriber imu_sub               = my_node.subscribe("/guidance/imu", 1, imu_callback);
    ros::Subscriber velocity_sub          = my_node.subscribe("/guidance/velocity", 1, velocity_callback);
    ros::Subscriber obstacle_distance_sub = my_node.subscribe("/guidance/obstacle_distance", 1, obstacle_distance_callback);
    ros::Subscriber ultrasonic_sub = my_node.subscribe("/guidance/ultrasonic", 1, ultrasonic_callback);
    ros::Subscriber position_sub = my_node.subscribe("/guidance/position", 1, position_callback);
    
    ros::Rate rate(10);
    
    iarc007::obstacle obs;

    while(ros::ok())
    {
        ros::spinOnce();

        if(img_temp[img_left_index].data){
             imshow("sub_left_video", img_temp[img_left_index]);
        }
        
        if(img_temp[img_right_index].data){
             imshow("sub_right_video", img_temp[img_right_index]);
        }
        
        if(img_depth[img_depth_index].data){
             imshow("sub_depth_video",img_depth[img_depth_index]);
        }

        int keycode = waitKey(1);
        if( keycode == 27 )
            break;
      
	obs_pub.publish(obs);

        rate.sleep();      
    }

    return 0;
}
