#ifndef _FILTER_STATISTIC_H_
#define _FILTER_STATISTIC_H_

#include "parameter.h"
//#include "common.h"

extern void Hist_Statistic_Init(Hist_Statistic* statistic);
//时间：2016.8.22
//功能：初始化直方图统计结构体
//参数：统计结构体指针
//说明：暂无
extern int Sig_Ch_Greater(Mat img, int threshold);
//时间：2016.8.22
//功能：计算某一单通道图像直方图中大于阈值的直方块个数
//参数：单通道图像
//说明：暂无
extern void Any_Ch_Greater(Mat img, Hist_Statistic *hist_statistic, int threshold, uchar flag);
//时间：2016.8.22
//功能：统计直方图的整个区间不为零的个数
//参数：原图，直方图统计结构体，阈值，二进制标志位
//说明：flag请使用宏定义，thanks
extern void Ch_Node_Set(Hist_Statistic *hist_statistic, int start[], int end[], uchar flag);
//时间：2016.8.22
//功能：设定统计区间
//参数：直方图结构体，开始数值，结束数值，二进制标志位
//说明：暂无
extern int Sig_Ch_Node_Greater(Mat img, int start, int end ,int threshold);
//时间：2016.8.22
//功能：单通道图像节点之间不为零数据统计
//参数：直方图结构体，开始数值，结束数值，阈值
//说明：暂无
extern void Any_Ch_Node_Greater(Mat img, Hist_Statistic *hist_statistic, int threshold, uchar flag);
//时间：2016.8.22
//功能：多通道图像节点之间不为零数据统计
//参数：原图，统计结构体，阈值，二进制标志位
//说明：暂无
extern float Sig_Ch_Variance(Mat img);
//时间：2016.8.22
//功能：统计单通道图像的方差
//参数：单通道图像
//说明：暂无
extern void Any_Ch_Variance(Mat img, Hist_Statistic *hist_statistic, uchar flag);
//时间：2016.8.23
//功能：统计各个通道的方差
//参数：统计结构体，二进制标志位
//说明：暂无
extern uchar Eight_NP_Cal(Mat img, int x, int y);
//时间：2016.8.24
//功能：图像八邻域均值计算
//参数：图像，x，y
//说明：暂无
extern float Sig_Ch_Entropy(Mat img);
//时间：2016.8.24
//功能：单通道图像二维熵计算
//参数：单通道图像
//说明：必须单通道
extern void Any_Ch_Entropy(Mat img, Hist_Statistic *hist_statistic, uchar flag);
//时间：2016.8.24
//功能：任意通道图像二维熵计算
//参数：图像
//说明：暂无
extern int Gre_Bar_Chose(Mat img, int r, Point center);
//时间：2016.9.3
//功能：绿色目标和障碍物的区分
//参数：图像，视野半径，视野中心，选择阈值
//说明：返回1说明是障碍物，返回0说明是绿色
extern void Bright_Adjust(Mat& src, Mat& dst, double dContrast, double dBright);
//时间：2016.9.3
//功能：图像亮度的调整
//参数：源图像、目标图像、比例缩放因子、平移
//说明：暂无
extern int Red_Gre_Cho(Mat img);
//时间：2016.9.13
//功能：红色目标的筛选
//参数：图像
//说明：暂无
#endif
