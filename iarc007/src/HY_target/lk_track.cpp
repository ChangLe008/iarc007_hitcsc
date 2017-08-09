#include <ros/ros.h>

#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <cv.h>

#include <iarc007/object.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dji_sdk/dji_sdk.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

using namespace std;
using namespace cv;

struct target_area
{
	Point2f bl;
	Point2f br;
	Point2f tl;
	Point2f tr;
	Point2f center;
	double height;
	double width;


};

//*******************全局变量
bool addRemovePt = false;
Point2f point1, point2;
Rect obj_area;

double h;
Mat img_sub;

ros::Subscriber ultrasonic_sub;
ros::Subscriber mono_video_sub;

//*********************

/* Mono Video */
void mono_sub_callback(sensor_msgs::Image img_msg)
{ 
	cv_bridge::CvImagePtr img_bridge_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);       
	img_bridge_ptr->image.copyTo(img_sub);       
}

void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{ 

	if(g_ul.ranges[0] > 0.)
	{
		h = g_ul.ranges[0];
	}
	else
	{
		//h=h;
	}

}

static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		point1 = Point2f((float)x, (float)y);
		addRemovePt = false;
		//waitKey(0);
	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		point2 = Point2f((float)x, (float)y);
		obj_area.x = min(point1.x, point2.x);
		obj_area.y = min(point1.y, point2.y);
		obj_area.width = fabs(point1.x - point2.x);
		obj_area.height = fabs(point1.y - point2.y);
		addRemovePt = true;

	}
	else if(event == CV_EVENT_RBUTTONDOWN)
	{
		obj_area.x = 0.;
		obj_area.y =0.;
		obj_area.width = 0.;
		obj_area.height = 0.;
		addRemovePt = true;
	}
	else
	{
		;
	}
}

//将roi的坐标转换为原图坐标
void point_roi(vector<Point2f> point_roi, vector<Point2f>&point_p, Rect rect)
{
	point_p.clear();
	Point2f tmp_p;
	for (int i = 0; i < point_roi.size(); i++)
	{
		tmp_p.x = point_roi[i].x + rect.x;
		tmp_p.y = point_roi[i].y + rect.y;
		point_p.push_back(tmp_p);
	}
}

//判断roi是否出界，返回校正后的矩形
Rect roi_resize(Rect area)
{
	int img_height = 480;
	int img_width = 640;

	//**************************判断roi是否出界
	area.x = max(area.x, 0);
	area.y = max(area.y, 0);

	area.x = min(area.x, img_width - 1);
	area.y = min(area.y, img_height - 1);

	if (area.width + area.x > img_width)
	{
		if (area.x > img_width)
		{
			cout << "roi error" << endl;
			
		}
		else
		{
			area.width = img_width - area.x;
		}
	}
	if (area.height + area.y > img_height)
	{
		if (area.y > img_height)
		{
			cout << "roi error" << endl;
		}
		else
		{
			area.height = img_height - area.y;
		}
	}
	return area;
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"mono_video_sub");
	ros::NodeHandle my_node;
	mono_video_sub = my_node.subscribe<sensor_msgs::Image>("/iarc007/mono_video1",2,mono_sub_callback);
	ultrasonic_sub = my_node.subscribe("/guidance/ultrasonic", 1, ultrasonic_callback);


	//VideoCapture cap(0);
	Mat img1, gray, pregray;
	Mat roi;

	TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);
	const int MAX_COUNT = 50;//角点最大数目
	const int MIN_COUNT = 5;//角点最小数目
	bool isinit = true;

	target_area img_area;//目标在图像中的区域

	vector<Point2f> pre_point, now_point, corner_point;
	vector<Point2f> input, output;

	namedWindow("gray", 1);
	setMouseCallback("gray", onMouse, 0);
	Rect target_area;
	
	ros::Publisher object_pub = my_node.advertise<iarc007::object>("hy_object",10);
	iarc007::object obj;
	obj.target_img_x.resize(10);
	obj.target_img_y.resize(10);

	while (ros::ok())
	{
		//cap >> img2;
		//img_sub.copyTo()
		cout<<"lll"<<endl;
		
		
		if (img_sub.data)
		{
			if(addRemovePt == true)
			{
				isinit=true;
			}
			cout<<obj_area<<endl;
			
			img_sub.copyTo(img1);
			cvtColor(img1, gray, CV_BGR2GRAY);
			imshow("gray", gray);
			
			//信息初始化
			obj.target_num=0;
			obj.target_img_x[0]=0.;
			obj.target_img_y[0]=0.;
			if (isinit)
			{
				target_area = obj_area;
				target_area = roi_resize(target_area);

				roi = gray(target_area);

				//*****
				img_area.center.x = target_area.x + target_area.width / 2.0;
				img_area.center.y = target_area.y + target_area.height / 2.0;

				//******************

				if (target_area.height > 30 || target_area.width > 30)
				{
					imshow("roi", roi);

					//**********角点检测
					goodFeaturesToTrack(roi, corner_point, MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
					cornerSubPix(roi, corner_point, subPixWinSize, Size(-1, -1), termcrit);
					//isinit = false;
					point_roi(corner_point, pre_point, obj_area);
					for (int i = 0; i < pre_point.size(); i++)
					{
						circle(gray, pre_point[i], 3, Scalar(0, 255, 0), -1, 8);
					}
					if (pre_point.size()>MIN_COUNT)
					{
						obj.target_num=1;
						isinit = false;
						addRemovePt = false;
						obj_area.height = 0.;
						obj_area.width = 0.;
					}				
				}
			}
			else
			{
				//********光流
				vector<uchar> status;
				vector<float> err;
				if (pregray.empty())
					gray.copyTo(pregray);
				cout << " " << now_point.size() << "-" << pre_point.size() << endl;
				calcOpticalFlowPyrLK(pregray, gray, pre_point, now_point, status, err, winSize,
					3, termcrit, 0, 0.001);
				//************目标中心解算
				size_t i, k;
				for (i = k = 0; i < now_point.size(); i++)
				{
					if (!status[i])
						continue;

					now_point[k++] = now_point[i];
					pre_point[k - 1] = pre_point[i];

					circle(img1, now_point[i], 3, Scalar(0, 255, 0), -1, 8);
				}
				now_point.resize(k);
				pre_point.resize(k);
				if (k > MIN_COUNT)
				{
					obj.target_num=1;
					Mat transform = findHomography(pre_point, now_point, CV_RANSAC);
					cout << h << endl;

					input.push_back(img_area.center);
					perspectiveTransform(input, output, transform);
					img_area.center = output[0];
					input.clear();
					output.clear();
					line(img1, img_area.center, img_area.center, Scalar(255, 0, 0), 3, 8);
				}
				else
				{
					isinit = true;
				}
				
				imshow("lk", img1);
				
				std::swap(pre_point, now_point);
			}

			cv::swap(pregray, gray);
			
		}
		obj.target_img_x[0]=img_area.center.x;
		obj.target_img_y[0]=img_area.center.y;
		object_pub.publish(obj);
		ros::spinOnce();
		waitKey(10);
	}

	return 0;
}
