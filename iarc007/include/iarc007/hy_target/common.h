/*
 * common.h
 *
 *  Created on: 2016年8月15日
 *      Author: hou
 */
#ifndef _COMMON_H_
#define _COMMON_H_H

#include "iarc007/hy_target/parameter.h"

extern void Letter_Show(char *buf, Point point, Mat& img);
//time: 2016.8.14
//func: show some words on the image
//para: words, image
//tips: none
extern float Re_Coin_Degree(Rect r1, Rect r2, int *num);
//time: 2016.8.14
//func: calculate the proportion that the overlap part of the two rectangular area share of the smaller one
//para: two rectangles
//tips: return coincidence degree and the number of the smaller one
extern void Hist_Draw(Mat hist, float maxval, int yoffset);
//time: 2016.8.14
//func: draw the histogram of the ROI
//para: histogram
//tips: none

extern void SVM_Load(CvSVM *svm, char *filename);		//SVM专用
//时间：2016.8.2
//功能：加载SVM
//参数：文件目录
//说明：暂无
extern void Obj_MaxMinSize_Cal(float height, int Min_Fit_Met, int Max_Fit_Met, int minoff, int maxoff, int *sizemin, int *sizemax);//SVM专用
//时间：2016.8.6
//功能：在某一高度下拟合目标大小上下限
//参数：飞行器高度，最小值拟合方式，最大值拟合方式，最小值裕量，最大值裕量，输出最小值，输出最大值
//说明：暂无
extern int Obj_MinSize_Cal(float height, int Min_Fit_Met, int minoff);	//SVM专用
//时间：2016.8.6
//功能：在某一高度下拟合目标大小下限
//参数：飞行器高度，最小值拟合方式，最小值裕量，输出最小值
//说明：暂无
extern int Obj_MaxSize_Cal(float height, int Max_Fit_Met, int maxoff);	//SVM专用
//时间：2016.8.6
//功能：在某一高度下拟合目标大小上限
//参数：飞行器高度，最大值拟合方式，最大值裕量，
//说明：暂无
extern void Vec_Init(void);	//SVM专用
//时间：2016.8.12
//功能：初始化向量
//参数：暂无
//说明：暂无
extern void Obj_Filter(Mat src, int lmax, int lmin, vector<Rect>found, vector<Rect>&Obj_Out);	//SVM专用
//时间：2016.8.12
//功能：对机器学习得到的目标进行选择
//参数：图像，目标尺度最大值，目标尺度最小值，原结果向量，输出向量
//说明：暂无
extern void Obj_H_Filter(int lmax, int lmin, vector<Rect>&found);	//SVM专用
//时间：2016.8.12
//功能：用高度限制对目标进行选择
//参数：目标尺度最大值，目标尺度最小值，结果向量
//说明：暂无
extern void Obj_Classify_Filter(vector<Rect>&found, float threshold);	//SVM专用
//时间：2016.8.12
//功能：用分类目标进行选择
//参数：结果向量，分类阈值
//说明：暂无
extern void Obj_Ell_Filter(vector<Rect>&found, Point center, float a, float b);
//时间：2016.8.21
//功能：用椭圆视野对目标进行筛选
//参数：结果向量，椭圆中心， 椭圆长轴，椭圆短轴
//说明：暂无
extern void Obj_Statistic_Filter(Mat img, vector<Rect>&found);
//时间：2016.8.25
//功能：用统计结果对目标进行筛选
//参数：结果向量，分类阈值
//说明：专门针对第低空的处理
extern Rect Rect_Merge(Rect r1, Rect r2);
//时间：2016.8.13
//功能：将两个矩形合并
//参数：两个矩形
//说明：返回新的矩形

extern void dianbianhuan(CvMat* R,CvMat* P,double h,int u,int v);
//时间：2016.8.13
//功能：坐标系转换
//参数：旋转矩阵，输出，高度，图像坐标
//说明：暂无
CvPoint DistortionPoint(int x,int y,CvMat* distortion,CvMat* intrinsic);
//时间：2016.8.13
//功能：畸变和矫正之间的坐标转换
//参数：畸变图中的坐标，畸变参数，内参矩阵
//说明：返回对应矫正图的坐标

extern Rect Dec_Tra_Connect(Rect rec, float kain);
//时间：2016.8.17
//功能：检测与跟踪的衔接
//参数：检测结果矩形，缩小比例
//说明：返回跟踪矩形
extern int Tra_Filter(RotatedRect res, float kainthres, float height, int Min_Fit_Met, int minoff, int Max_Fit_Met, int maTra_Filterxoff);
//时间：2016.8.17
//功能：检测结果矩形是否满足要求
//参数：结果矩形，长宽比阈值，高度
//说明：满足返回1，不满足返回0
extern double point2Line(Point2f pt, Point2f lp1, Point2f lp2);
//time: 2016.8.20
//func: 计算点到直线距离
//para: point outside this line, two points on this line
//tips: none
extern void PointInArea(vector<Point>&area, Rect rec, Mat img);
//找到区域内部满足条件的点集
extern int Ell_Inside_Cal(Point center, float a, float b, Point test);
//time: 2016.8.21
//func: 判断一个点是否在椭圆内
//para: 椭圆中心，长轴，短轴，待测点
//tips: 返回1表示在，0表示不在
extern float RoaRec_Len_Scale(RotatedRect res);
//时间：2016.8.31
//功能：计算旋转矩形长短边比例
//参数：旋转矩形
//tips: 暂无
extern int OTSU_Th(Mat img);
//时间：2016.9.12
//功能：计算单通道图的大津阈值
//参数：单通道图
//tips: 暂无
#endif
