#include "iarc007/hy_target/L_target.h"

using namespace std;
using namespace cv;

void ExtendCV::High_Target(Mat img_src,RotatedRect min_rect,target_description& target)
{
	Point2f rect_points[4];
	min_rect.points(rect_points);

	Rect bound_rect = min_rect.boundingRect();

	int roi_min_x = bound_rect.x;
	int roi_max_x = bound_rect.x+bound_rect.width;
	int roi_min_y = bound_rect.y;
	int roi_max_y = bound_rect.y+bound_rect.height;

	Point2f middle_point[4];
	for( int j = 0; j < 4; j++ )
	{
		middle_point[j].x = ((rect_points[j].x+rect_points[(j+1)%4].x)/2);
		middle_point[j].y = ((rect_points[j].y+rect_points[(j+1)%4].y)/2);
	}
	float panel_centre_x = (middle_point[0].x+middle_point[2].x)/2;
	float panel_centre_y = (middle_point[0].y+middle_point[2].y)/2;

	float distance_1 = (middle_point[0].x-middle_point[2].x)*(middle_point[0].x-middle_point[2].x)+(middle_point[0].y-middle_point[2].y)*(middle_point[0].y-middle_point[2].y);

	distance_1 = sqrt(distance_1);

	float distance_2 = (middle_point[1].x-middle_point[3].x)*(middle_point[1].x-middle_point[3].x)+(middle_point[1].y-middle_point[3].y)*(middle_point[1].y-middle_point[3].y);

	distance_2 = sqrt(distance_2);

	if(distance_1>distance_2)
	{
		float temp_x = rect_points[0].x;
		float temp_y = rect_points[0].y;
		float middle_temp_x = middle_point[0].x;
		float middle_temp_y = middle_point[0].y;
		for(int m=0;m<3;m++)
		{
			rect_points[m].x = rect_points[m+1].x;
			rect_points[m].y = rect_points[m+1].y;
			middle_point[m].x = middle_point[m+1].x;
			middle_point[m].y = middle_point[m+1].y;
		}
		rect_points[3].x = temp_x;
		rect_points[3].y = temp_y;
		middle_point[3].x =middle_temp_x;
		middle_point[3].y =middle_temp_y;

		float distance_temp = distance_1;
		distance_1 = distance_2;
		distance_2 = distance_temp;
	}

	int threshold_num = 3;
	Point2f angle_point[4];

	angle_point[0].x = threshold_num;
	angle_point[0].y = threshold_num;
	angle_point[1].x = img_src.cols-threshold_num;
	angle_point[1].y = threshold_num;
	angle_point[2].x = img_src.cols-threshold_num;
	angle_point[2].y = img_src.rows-threshold_num;
	angle_point[3].x = threshold_num;
	angle_point[3].y = img_src.rows-threshold_num;

	Scalar color = Scalar(255,255,255);
	for(int i=0;i<4;i++)
	{
		line(img_src, angle_point[i],angle_point[(i+1)%4],color,1,8);
	}

	float result[4];
	bool result_check = true;
	for(int j = 0;j<4;j++)
	{
		result_check=result_check&(rect_points[j].x>angle_point[0].x);
		result_check=result_check&(rect_points[j].x<angle_point[1].x);
		result_check=result_check&(rect_points[j].y>angle_point[0].y);
		result_check=result_check&(rect_points[j].y<angle_point[2].y);
	}

	float LW = distance_1/distance_2;

	float Low_LW = 0.5;
	float High_LW = 0.7;

	result_check = result_check&&(LW>Low_LW);
	result_check = result_check&&(LW<High_LW);

	if(result_check)
	{
		target.target_flag = true;
		Point2f vector01;
		Point2f vector02;

		vector01.x = rect_points[1].x-rect_points[0].x;
		vector01.y = rect_points[1].y-rect_points[0].y;

		vector02.x = rect_points[2].x-rect_points[0].x;
		vector02.y = rect_points[2].y-rect_points[0].y;

		float vector01_x_vector02 = vector01.x*vector02.y-vector02.x*vector01.y;

		int rotation_direction;

		if(vector01_x_vector02>0)
		{
			rotation_direction = -1;
		}
		else
		{
			rotation_direction = +1;
		}

		float roi_radius;
		roi_radius = distance_1/8;
		Point2f roi_centre[4];
		float angle_i_next[4];

		float sqrt_2 = 1.414213562;

		for(int n = 0;n<4;n++)
		{
			angle_i_next[n] = atan2f(rect_points[(n+1)%4].y-rect_points[n].y,rect_points[(n+1)%4].x-rect_points[n].x);
			angle_i_next[n] = angle_i_next[n]-rotation_direction*CV_PI/4;

			roi_centre[n].x = rect_points[n].x+sqrt_2*roi_radius*cos(angle_i_next[n]);
			roi_centre[n].y = rect_points[n].y+sqrt_2*roi_radius*sin(angle_i_next[n]);

			circle(img_src,roi_centre[n],roi_radius,Scalar(255,255,255),1);

			circle(img_src,roi_centre[n],roi_radius,Scalar(255,255,255),1);

			line(img_src, rect_points[n], rect_points[(n+1)%4],Scalar(255,255,255), 1,8);

			/*char str_i[11];
			sprintf(str_i,"%d",n);
			putText(img_src_forever,str_i, roi_centre[n],CV_FONT_HERSHEY_COMPLEX,0.7,Scalar(0,255,0));*/
		}
		float circle_black[4]={0,0,0,0};
		for(int roi_x = roi_min_x;roi_x<roi_max_x;roi_x++)
		{
			for(int roi_y = roi_min_y;roi_y<roi_max_y;roi_y++)
			{
				if(img_src.at<uchar>(roi_y,roi_x)>10)
				{
					continue;
				}
				else
				{
					for(int circle_num=0;circle_num<4;circle_num++)
					{
						float roi_result;
						roi_result = (roi_x-roi_centre[circle_num].x)*(roi_x-roi_centre[circle_num].x)+(roi_y-roi_centre[circle_num].y)*(roi_y-roi_centre[circle_num].y);
						if(roi_result<=roi_radius*roi_radius)
						{
							circle_black[circle_num]++;
						}
					}
				}
			}
		}
		float black_num01 = circle_black[0]+circle_black[1];
		float black_num23 = circle_black[2]+circle_black[3];
		if(black_num01>black_num23)
		{
			circle(img_src,middle_point[0],10,Scalar(255,255,255),-1);
			target.start_point.x = middle_point[0].x;
			target.start_point.y = middle_point[0].y;
		}
		else
		{
			circle(img_src,middle_point[2],10,Scalar(255,255,255),-1);
			target.start_point.x = middle_point[2].x;
			target.start_point.y = middle_point[2].y;
		}
		target.middle_point.x = panel_centre_x;
		target.middle_point.y = panel_centre_y;
	}
	else
	{
		target.target_flag = false;
	}

	//namedWindow("轮廓");
	//imshow("轮廓",img_src);
	cvWaitKey(1);
}
