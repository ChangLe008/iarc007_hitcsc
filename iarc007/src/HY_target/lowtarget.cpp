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

using namespace std;
using namespace cv;

float ha = 0;
struct thre
{
	int thre_L;
	int thre_H;
};

int low = 30;
int high = 60;
float heng = 0.5;

void otsu(Mat img_gray,thre& thre_num)
{
    float histogram_H[256] = {0};
	int gray_num;
	int size_H = 0;
    for(int i=0; i < img_gray.rows; i++)
    {
        for(int j = 0; j < img_gray.cols; j++)
        {
			if(1)
			{
				gray_num = img_gray.at<uchar>(i,j);
				histogram_H[gray_num]++;
				size_H++;
			}
        }
    }

    for(int i = 0; i < 256; i++)
    {
        histogram_H[i] = histogram_H[i]/size_H;
    }

    float avgValue_H=0;
    for(int i=0; i < 256; i++)
    {
        avgValue_H += i * histogram_H[i];
    }

    int thre_H;
    float maxVariance_H=0;
    float w_H= 0, u_H= 0;
    for(int i = 0; i < 256; i++)
    {
        w_H += histogram_H[i];
        u_H += i * histogram_H[i];

        float t = avgValue_H * w_H - u_H;
        float variance = t * t / (w_H * (1 - w_H) );
        if(variance > maxVariance_H)
        {
            maxVariance_H = variance;
            thre_H = i;
        }
    }
	thre_H *= 0.85;
	Mat img_binary(img_gray.rows,img_gray.cols,CV_8UC1);
	threshold(img_gray,img_binary,thre_H,255,THRESH_BINARY_INV);

	cvNamedWindow("H");
	imshow("H",img_binary);

	float histogram_L[256] = {0};
	int size_L = 0;
	for(int i=0; i < img_gray.rows; i++)
	{
		for(int j = 0; j < img_gray.cols; j++)
		{
			if(img_binary.at<uchar>(i,j)<100)
			{
				continue;
			}
			else
			{
				gray_num = img_gray.at<uchar>(i,j);
				histogram_L[gray_num]++;
				size_L++;
			}
		}
	}



	for(int i = 0; i < 256; i++)
	{
		histogram_L[i] = histogram_L[i]/size_L;
	}

	float avgValue_L=0;
	for(int i=0; i < 256; i++)
	{
		avgValue_L += i * histogram_L[i];
	}

	int fangcha = 0 ;

	for(int i =0;i<256;i++)
	{
		if(i<thre_H)
		{
			fangcha += pow((i-avgValue_L ),2)*histogram_L[i];
		}
	}
	//printf("-------------------------------%d\n",fangcha);
	int thre_L;
	float maxVariance_L=0;
	float w_L = 0, u_L = 0;
	//float fangcha = 0;
	for(int i = 0; i < 256; i++)
	{
		w_L += histogram_L[i];
		u_L += i * histogram_L[i];

		//fangcha += pow((i-avgValue_L),2)*histogram_L[i];

		float t = avgValue_L * w_L - u_L;
		float variance = t * t / (w_L * (1 - w_L));
		if(variance > maxVariance_L)
		{
			maxVariance_L = variance;
		    thre_L = i;
		}  
	}

	//threshold(img_gray,img_binary,thre_L,255,THRESH_BINARY_INV);
	//imshow("hahah",img_binary);
	thre_L *=1.25;
	//ha = maxVariance_L;

	thre_num.thre_L = thre_L;
	thre_num.thre_H = thre_H;
	//printf("_______%d\t,%d\n",th1,th2);
}


Mat img_b(Mat img_src)
{
	Mat img_gray(img_src.rows,img_src.cols,CV_8UC1);
	Mat img_binary(img_src.rows,img_src.cols,CV_8UC1);

	cvtColor(img_src,img_gray,CV_BGR2GRAY);

	cvNamedWindow("hui",0);
	imshow("hui",img_gray);

	int histSize = 256;
    float range[] = {0,255};
    const float *histRange = {range};

	Mat grayHist;

	calcHist( &img_gray, 1, 0, Mat(), grayHist, 1, &histSize, &histRange,true,false);

    int histWidth = 400;
    int histHeight = 400;
    int binWidth = cvRound((double)histWidth/histSize);

 	Mat histImage(histHeight,histWidth,CV_8UC3,Scalar(0,0,0));

	normalize(grayHist,grayHist,0,histImage.rows,NORM_MINMAX,-1,Mat());
 	for( int i = 1; i < histSize; i++ )
   	{
     	line(histImage,Point( binWidth*(i-1),histHeight - cvRound(grayHist.at<float>(i-1)) ) ,
                       Point( binWidth*(i),  histHeight - cvRound(grayHist.at<float>(i)) ),
                      Scalar( 0, 0, 255), 2, 8, 0  );
    }


	thre thre_num;
	otsu(img_gray,thre_num);

	int thre_L = thre_num.thre_L;
	int thre_H = thre_num.thre_H;

	//printf("L:%d\tH:%d\n",thre_num.thre_L,thre_num.thre_H);

	line(histImage,Point(binWidth*thre_L,histHeight),
                       Point(binWidth*thre_L, 0),
                      Scalar( 255,0, 0), 2, 8, 0  );
	line(histImage,Point(binWidth*thre_H,histHeight) ,
                       Point( binWidth*thre_H, 0),
                      Scalar( 0, 255, 0), 2, 8, 0  );


	line(histImage,Point(binWidth*high,histHeight) ,
                       Point( binWidth*high, 0),
                      Scalar( 255, 255, 0), 2, 8, 0  );

	line(histImage,Point(binWidth*low,histHeight) ,
                       Point( binWidth*low, 0),
                      Scalar( 0, 255, 255), 2, 8, 0  );

	line(histImage,Point(0,histHeight*(1-heng)) ,
                       Point(histWidth, histHeight*(1-heng)),
                      Scalar(0, 255, 255), 2, 8, 0  );

	char str_23[11];
	sprintf(str_23,"%f",ha);
	putText(histImage,str_23,Point(50,100),CV_FONT_HERSHEY_COMPLEX,0.7,Scalar(255,0,255));

	imshow("直方图",histImage);


	float g_num =0;
	Mat img_bin(img_src.rows,img_src.cols,CV_8UC1);
	img_bin.setTo(cv::Scalar::all(0));
	for(int i=0;i<img_gray.rows;i++)
	{
			for(int j=0;j<img_gray.cols;j++)
			{
				int num = img_gray.at<uchar>(i,j);
				if(1)
				{
					if(num>low)
					{
						if(num<high)
						{
							img_bin.at<uchar>(i,j) = 255;
							g_num ++;
						}
					}
				}

			}
	}

	ha = g_num;
	img_binary.setTo(cv::Scalar::all(0));
	if(g_num>10)
	{
		threshold(img_gray,img_binary,thre_L,255,THRESH_BINARY_INV);

		cvNamedWindow("L",0);
		imshow("L",img_binary);


		int morph_size_1 = 1;
		Mat element_1 = getStructuringElement(0,Size(2*morph_size_1+1,2*morph_size_1+1), Point(morph_size_1,morph_size_1));
		/*erode(img_binary,img_binary,element_1);
		erode(img_binary,img_binary,element_1);
		erode(img_binary,img_binary,element_1);
		erode(img_binary,img_binary,element_1);
		erode(img_binary,img_binary,element_1);*/
		erode(img_binary,img_binary,element_1);
	}
	return img_binary;
}


Mat  RGB2HSI(Mat img_src)
{
	Mat img_binary(img_src.rows,img_src.cols,CV_8UC1);

	img_binary.setTo(cv::Scalar::all(0));

	int data_R,data_G,data_B;
	float data_H,data_S,data_I;
	Vec3b data;
	for(int i = 0;i<img_src.rows;i++)
	{
		for(int j= 0;j<img_src.cols;j++)
		{
			data = img_src.at<Vec3b>(i,j);
			data_B = data.val[0];
			data_G = data.val[1];
			data_R = data.val[2];

			data_I = (data_R+data_G+data_B)/3.0;
			int temp = (data_R>data_G)?data_G:data_R;
			temp = (temp>data_B)?data_B:temp;

			data_S = 1-3.0*temp/(data_R+data_G+data_B);

			float deno,nume;
			nume =(data_R-data_G+data_R-data_B)/2.0;
			deno = sqrtf((data_R-data_G)*(data_R-data_G)+(data_R-data_B)*(data_G-data_B));

			if(deno == 0)
			{
				data_H = 0;
			}
			else
			{
				float theta = acosf(nume/deno)*180/CV_PI;

				if(data_B <= data_G)
				{
					data_H = theta;
				}
				else
				{
					data_H = 360 - theta;
				}
			}

			if(fabsf(data_H-360)<30||(data_H<30))
			{
				if(data_S>0.1)
				{
					img_binary.at<uchar>(i,j)=255;
				}
			}
		}
	}
	int morph_size_1 = 1;
	Mat element_1 = getStructuringElement(0,Size(2*morph_size_1+1,2*morph_size_1+1), Point(morph_size_1,morph_size_1));
	erode(img_binary,img_binary,element_1);
	erode(img_binary,img_binary,element_1);
	erode(img_binary,img_binary,element_1);
	return img_binary;
}

void ExtendCV::Low_Target(Mat img_src,target_description& target,int robot_color)
{
	Mat img_binary(img_src.rows,img_src.cols,CV_8UC1);
	Mat img_binary_forever(img_src.rows,img_src.cols,CV_8UC1);

	if(robot_color == 0)
	{
		img_binary = RGB2HSI(img_src);
	}
	else
	{
		img_binary = img_b(img_src);
	}


	img_binary.copyTo(img_binary_forever);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(img_binary, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,Point(0, 0));

	double maxArea = 0;
	double area = 0;
	int maxAreaIdex = 0;
	vector<RotatedRect> minRect(contours.size());
	Scalar color = Scalar(0,0,255);
	Point2f rect_points[4];
	Point2f middle_point[4];

	Point2f XZ[4];
	Point2f ZD[4];

	float L_dis =0;
	float S_dis =0;

	for (int i = 0; i<contours.size(); i++)
	{
		minRect[i] = minAreaRect(Mat(contours[i]));
		minRect[i].points(rect_points);

		/*for(int j = 0;j<4;j++)
		{
			line(img_src, rect_points[j], rect_points[(j+1)%4],color, 1,8);
		}*/

		for( int j = 0; j < 4; j++ )
		{
			middle_point[j].x = ((rect_points[j].x+rect_points[(j+1)%4].x)/2);
			middle_point[j].y = ((rect_points[j].y+rect_points[(j+1)%4].y)/2);
		}

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

		float LW = distance_1/distance_2;
		float Low_LW = 0.5;
		float High_LW = 0.7;

		bool result_LW = true;
		result_LW = result_LW&&(LW>Low_LW);
		result_LW = result_LW&&(LW<High_LW);

		if(result_LW)
		{
			area = fabs(contourArea(contours[i]));
			if(area>maxArea)
			{
				maxArea = area;
				maxAreaIdex = i;
				for(int m =0;m<4;m++)
				{
					L_dis = distance_2;
					S_dis = distance_1;
					XZ[m].x = rect_points[m].x;
					XZ[m].y = rect_points[m].y;

					ZD[m].x = middle_point[m].x;
					ZD[m].y = middle_point[m].y;
				}
			}
		}
	}

	float area_check =3000;
	bool result_check = true;
	result_check = result_check&&(maxArea>area_check );

	int threshold_num = 1;
	Point2f angle_point[4];

	angle_point[0].x = threshold_num;
	angle_point[0].y = threshold_num;
	angle_point[1].x = img_binary.cols-threshold_num;
	angle_point[1].y = threshold_num;
	angle_point[2].x = img_binary.cols-threshold_num;
	angle_point[2].y = img_binary.rows-threshold_num;
	angle_point[3].x = threshold_num;
	angle_point[3].y = img_binary.rows-threshold_num;

	for(int i=0;i<4;i++)
	{
		line(img_src , angle_point[i],angle_point[(i+1)%4],Scalar(255,255,0),1,8);
	}

	//四个点都必须在视野里
	for(int j = 0;j<4;j++)
	{
		result_check=result_check&(XZ[j].x>angle_point[0].x);
		result_check=result_check&(XZ[j].x<angle_point[1].x);
		result_check=result_check&(XZ[j].y>angle_point[0].y);
		result_check=result_check&(XZ[j].y<angle_point[2].y);
	}

	//用椭圆限定，至少有三个点在椭圆里
	bool result_check_1 = false;
	bool re_check[4];
	float long_axis = 318;
	float short_axis = 238;
	float result[4];

	ellipse(img_src,Point(320,240),Size(long_axis,short_axis),0,0,360,Scalar(0,0,255),3,8,0);
	for(int i = 0;i<4;i++)
	{
		result[i]= (XZ[i].x-320)*(XZ[i].x-320)/(long_axis * long_axis)+(XZ[i].y-240)*(XZ[i].y-240)/(short_axis*short_axis);
	}
	for(int i=0;i<4;i++)
	{
		re_check[i] = (result[(i+1)%4]<1)&(result[(i+2)%4]<1)&(result[(i+3)%4]<1);
		result_check_1 = result_check_1||re_check[i];
	}
	result_check = result_check&result_check_1;

	if(result_check)
	{
		target.target_flag= true;

		float panel_centre_x = (ZD[0].x+ZD[2].x)/2;
		float panel_centre_y = (ZD[0].y+ZD[2].y)/2;
		circle(img_src,Point(panel_centre_x,panel_centre_y),2,Scalar(0,0,255),2);
		line(img_src,ZD[0], ZD[2],Scalar(0,0,255),3,8);

		int roi_min_x = 800;
		int roi_max_x = -1;
		int roi_max_y = -1;
		int roi_min_y = 500;
		for(int i=0;i<4;i++)
		{
			roi_min_x = (XZ[i].x<roi_min_x)?XZ[i].x:roi_min_x;
			roi_max_x = (XZ[i].x>roi_max_x)?XZ[i].x:roi_max_x;
			roi_min_y = (XZ[i].y<roi_min_y)?XZ[i].y:roi_min_y;
			roi_max_y = (XZ[i].y>roi_max_y)?XZ[i].y:roi_max_y;
		}


		Point2f vector01;
		Point2f vector02;

		vector01.x = XZ[1].x-XZ[0].x;
		vector01.y = XZ[1].y-XZ[0].y;

		vector02.x = XZ[2].x-XZ[0].x;
		vector02.y = XZ[2].y-XZ[0].y;

		float vector01_x_vector02 = vector01.x*vector02.y-vector02.x*vector01.y;

		int rotation_direction;//顺时针+1，逆时针-1
		if(vector01_x_vector02>0)
		{
			rotation_direction = -1;
		}
		else
		{
			rotation_direction = +1;
		}

		float roi_radius;
		roi_radius = S_dis/8;
		Point2f roi_centre[4];
		float angle_i_next[4];
		float sqrt_2 = 1.414213562;

		for(int n = 0;n<4;n++)
		{
			angle_i_next[n] = atan2f(XZ[(n+1)%4].y-XZ[n].y,XZ[(n+1)%4].x-XZ[n].x);
			angle_i_next[n] = angle_i_next[n]-rotation_direction*CV_PI/4;

			roi_centre[n].x = XZ[n].x+sqrt_2*roi_radius*cos(angle_i_next[n]);
			roi_centre[n].y = XZ[n].y+sqrt_2*roi_radius*sin(angle_i_next[n]);

			circle(img_src,roi_centre[n],roi_radius,Scalar(255,255,255),1);

			circle(img_binary,roi_centre[n],roi_radius,Scalar(255,255,255),1);

			line(img_src, XZ[n], XZ[(n+1)%4],Scalar(0,0,255),3,8);
			line(img_binary, XZ[n], XZ[(n+1)%4],Scalar(255,255,255), 1,8);
			/*char str_i[11];
			sprintf(str_i,"%d",n);
			putText(img_src_forever,str_i, roi_centre[n],CV_FONT_HERSHEY_COMPLEX,0.7,Scalar(0,255,0));*/
		}

		int circle_black[4] = {0,0,0,0};

		int x0 = roi_min_x>0?roi_min_x:0;
		int x1 = roi_max_x<img_src.cols?roi_max_x:img_src.cols;
		int y0 = roi_min_y>0?roi_min_y:0;
		int y1 = roi_max_y>img_src.rows?img_src.rows:roi_max_y;

		for(int roi_x = x0;roi_x<x1;roi_x++)
		{
			for(int roi_y = y0;roi_y<y1;roi_y++)
			{
				if(img_binary_forever.at<uchar>(roi_y,roi_x) > 100)
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
		
		target.roi_rect.x = x0;
		target.roi_rect.y = y0;

		target.roi_rect.width = x1-x0;

		target.roi_rect.height = y1-y0;

		float black_num01 = circle_black[0]+circle_black[1];
		float black_num23 = circle_black[2]+circle_black[3];

		if(black_num01>black_num23)
		{
			circle(img_src,ZD[0],5,Scalar(0,0,255),-1);
			target.start_point.x = ZD[0].x;
			target.start_point.y = ZD[0].y;

			target.end_point.x = ZD[2].x;
			target.end_point.y = ZD[2].y;
		}
		else
		{
			circle(img_src,ZD[2],5,Scalar(0,0,255),-1);
			target.start_point.x = ZD[2].x;
			target.start_point.y = ZD[2].y;

			target.end_point.x = ZD[0].x;
			target.end_point.y = ZD[0].y;
		}
		target.middle_point.x = panel_centre_x;
		target.middle_point.y = panel_centre_y;

		/*char str_01[11];
		sprintf(str_01,"%f",black_num01);
		putText(img_src_forever,str_01,Point(300,200),CV_FONT_HERSHEY_COMPLEX,0.7,Scalar(0,255,0));

		char str_23[11];
		sprintf(str_23,"%f",black_num23);
		putText(img_src_forever,str_23,Point(300,300),CV_FONT_HERSHEY_COMPLEX,0.7,Scalar(0,255,0));*/
	}
	else
	{
		target.target_flag= false;
	}
	imshow("src",img_src);
	waitKey(1);
}
