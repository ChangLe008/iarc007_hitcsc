/*
 * findcircle.h
 *
 *  Created on: 2016年8月20日
 *      Author: hou
 */

#ifndef INCLUDE_IARC007_HY_TARGET_FINDCIRCLE_H_
#define INCLUDE_IARC007_HY_TARGET_FINDCIRCLE_H_


#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <fstream>

struct circle_found
{
	float score;	//匹配分数
	cv::Vec3f circle;	//找到的圆
};
#define UINT int

using namespace std;
using namespace cv;

namespace ExtendCV
{
	//_image——输入的图像，必须为8位单通道，_circles——找到的圆，dp——cv::houghcircles中的dp，min_dist——cv::houghcircles中的minDist两圆最小距离
	//low_threshold——将_image预处理提取轮廓的canny低阈值，high_threshold——将_image预处理提取轮廓的canny高阈值
	//acc_threshold——cv::houghcircles中的param2累加器值，minRadius——圆的最小半径，maxRadius——圆的最大半径
	//minScore——找出的圆与现有的轮廓的重合率，作为分数
	//_contour_image——可选的输入轮廓图，如果这里非空，则将low_threshold与high_threshold忽略（方便轮廓图的进一步预处理）
	void FindCircles( cv::InputArray _image, cv::vector<circle_found>& _circles,float dp, int min_dist,
	int low_threshold, int high_threshold,int acc_threshold,int minRadius, int maxRadius,
	float minScore, cv::InputArray _contour_image=cv::Mat() );

}

#endif /* INCLUDE_IARC007_HY_TARGET_FINDCIRCLE_H_ */
