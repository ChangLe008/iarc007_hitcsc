#ifndef _L_TARGET_H_
#define _L_TARGET_H_


#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct target_description
{
	bool target_flag;
	cv::Point2f start_point;
	cv::Point2f end_point;
	cv::Point2f middle_point;
	cv::Rect roi_rect;
};

#define UINT int

using namespace std;
using namespace cv;

namespace ExtendCV
{
	//img_src_1为8位3通道彩色图
	//target.target_flag :true 视野中有目标，false 视野中无目标
	//target.start_point :前向点
	//target.end_point: 后向点
	//target.middle_point :中心点
	void Low_Target(Mat img_src,target_description& target,int robot_color);
	void High_Target(Mat img_src,RotatedRect min_rect,target_description& target);
}

#endif
