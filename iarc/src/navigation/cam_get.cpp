#include <iostream>
#include <string>

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv/cv.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/CameraInfo.h>

#include <yaml-cpp/yaml.h> // use to parse YAML calibration file
#include <fstream> // required to parse YAML 

int WIDTH=640;
int HEIGHT=480;

#define IMAGE_SIZE (WIDTH*HEIGHT)

using namespace cv;
using namespace std;

static const char CAM_YML_NAME[]    = "camera_name";
static const char WIDTH_YML_NAME[]  = "image_width";
static const char HEIGHT_YML_NAME[] = "image_height";
static const char K_YML_NAME[]      = "camera_matrix";
static const char D_YML_NAME[]      = "distortion_coefficients";
static const char R_YML_NAME[]      = "rectification_matrix";
static const char P_YML_NAME[]      = "projection_matrix";
static const char DMODEL_YML_NAME[] = "distortion_model";

Mat img_src;

/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
	cv_ptr->image.copyTo(img_src);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("left_image", cv_ptr->image);
    cv::waitKey(1);
}

bool pubCameraInfo(sensor_msgs::SetCameraInfo::Request  &req, sensor_msgs::SetCameraInfo::Response &res)
{
	//sensor_msgs::CameraInfo info;

	//info = req.camera_info;

	res.success = true;	
	res.status_message = "OK";

	//ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	//cameraInfo.publish(req.camera_info);	

	ROS_INFO("Calibration OK !\n");

	return true;
}
/*
struct SimpleMatrix
{
    int rows;
    int cols;
    double* data;

    SimpleMatrix(int rows, int cols, double* data)
        : rows(rows), cols(cols), data(data)
    {}
};

void transfer_SimpleMatrix_from_YML_to_ROSmsg(const YAML::Node& node, SimpleMatrix& m)
{
    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows*cols; ++i)
    {
        m.data[i] = data[i].as<double>();
    }
}

void read_params_from_yaml_and_fill_cam_info_msg(std::string& file_name, sensor_msgs::CameraInfo& cam_info)
{   
    std::ifstream fin(file_name.c_str());
    //std::ifstream fin("/home/exbot/catkin_ws/src/iarc007/doc/ost.yaml");
    YAML::Node doc = YAML::Load(fin);

    //cout << "End" << endl;	
    cam_info.width = doc[WIDTH_YML_NAME].as<int>();
    cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

    SimpleMatrix K_(3, 3, &cam_info.K[0]);
    transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[K_YML_NAME], K_);
    SimpleMatrix R_(3, 3, &cam_info.R[0]);
    transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[R_YML_NAME], R_);
    SimpleMatrix P_(3, 4, &cam_info.P[0]);
    transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[P_YML_NAME], P_);

    
    cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

    const YAML::Node& D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_rows = D_node["rows"].as<int>();
    D_cols = D_node["cols"].as<int>();
    const YAML::Node& D_data = D_node["data"];
    cam_info.D.resize(D_rows*D_cols); 
    for (int i = 0; i < D_rows*D_cols; ++i)
    {
        cam_info.D[i] = D_data[i].as<float>();  
    }
}
*/
int main(int argc, char** argv)
{
	ros::init(argc,argv,"mono_video");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");	
 
/*	CvCapture* capture=cvCreateCameraCapture(1);

	if(!capture)
	{
		cout << "No stream in hy_opencv_pub" << endl;
		return -1;
	}

	IplImage* image = cvQueryFrame(capture);  

	if(!image)
	{
		cout << "No image in hy_opencv_pub" << endl;
		return -1;
	}
*/
	VideoCapture cap(0);

	ros::Subscriber left_image_sub = nh.subscribe("/guidance/left_image",  10, left_image_callback);

	ros::ServiceServer service = nh.advertiseService("/my_mono/set_camera_info", pubCameraInfo);
	ROS_INFO("Ready to publish CameraInfo !\n");

	ros::Publisher mono_video_pub = nh.advertise<sensor_msgs::Image>("/my_mono/image_raw",10);
	ros::Publisher cameraInfo = nh.advertise<sensor_msgs::CameraInfo>("/my_mono/camera_info",20);

	//ros::Publisher mono_video_pub = nh.advertise<sensor_msgs::Image>("/camera/image_raw",10);
	//ros::Publisher cameraInfo = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info",20);


	sensor_msgs::CameraInfo camera_info;

	std::string camera_params;// = n.getParam("/mono_video/camera_params",camera_params);
	ros::param::param<std::string>("/mono_video/camera_params", camera_params, "/home/hitcsc/catkin_ws/src/iarc007/doc/ost.yaml");

	cout << camera_params << endl;;

	camera_info.binning_x = 0;
	camera_info.binning_y = 0;

	camera_info.roi.x_offset	= 0;
	camera_info.roi.y_offset	= 0;
	camera_info.roi.width = 0;
	camera_info.roi.height = 0;
	camera_info.roi.do_rectify = false;
	
	cv_bridge::CvImage img_bridge; 

	Mat tmp_frame;//(image->width,image->height,CV_8UC3, Scalar(0,0,255)); 
	
	ros::Rate rate(60);

    	cout << "hy_opencv_pub: start" << endl;

	//read_params_from_yaml_and_fill_cam_info_msg(camera_params, camera_info);

	while(ros::ok())
	{
		cap >> tmp_frame;
		//cout << "to enter while" << endl;
		//image=cvQueryFrame(capture); 

		if(!tmp_frame.empty()){
			img_bridge.header.stamp = camera_info.header.stamp = ros::Time::now();
	     		camera_info.header.frame_id = "/my_mono/camera_info";
			//camera_info.header.frame_id = "/camera/camera_info";

			cameraInfo.publish(camera_info);
	
			//tmp_frame = Mat(image);

			imshow("mono_video_pub",tmp_frame);
			int keycode = waitKey(1);
			if( keycode == 27 )
			    break;
	
			cout << tmp_frame.cols << " " << tmp_frame.rows << endl;
			tmp_frame.copyTo(img_bridge.image);
			//img_src.copyTo(img_bridge.image);


			img_bridge.header.frame_id = "/my_mono/image_raw";
			//img_bridge.header.frame_id = "/camera/image_raw";
			//img_bridge.header.stamp = ros::Time::now();
			img_bridge.encoding = sensor_msgs::image_encodings::BGR8;

			mono_video_pub.publish(img_bridge.toImageMsg());
		}

		ros::spinOnce();

		rate.sleep();	
	}

	cout << "hy_opencv_pub: stop" << endl;
	
	return 0;
}
