 #ifndef _TRACK_H_
#define _TRACK_H_

#include "iarc007/hy_target/parameter.h"
#include "iarc007/hy_target/common.h"

void R_Watch_Init(void);
//time: 2016.8.14
//func：initialization of structure "R_Watch"
//para: none
//tips: As for variables of Bool, when we initializ them, 
//		we can't give all of them the same initial value 0 or 1, it must has some useful purpose.
 void CamShift_Init(void);
//time: 2016.8.14
//func：initialize some parameters
//para: none
//tips: none
 void R_Sta_Reset(Robot_Sta_Single *robot);
//time: 2016.8.14
//func: reset the status of a robot
//para: the struct of one robot
//tips: This function can only clean up the last member of our array, we can't use it when our robots disappear.
 void Hist_Up(Mat& hist1, Mat& hist2, float weight);
//time: 2016.8.14
//func: Update the histogram of the target
//para: first histogram, second histogram, weight
//tips: the result will be stored in the first paramater
 void Hist_Get(Hist_Store *hist, Mat img);
//time: 2016.8.14
//func: Get a standard model to the file
//para: histogram(Double pointer), the iamge(not ROI), rectangle
//tips: the first input parametaer must be "&(hist_store.hist_mould)"
 void Num_Update(void);
//time: 2016.8.14
//func: Update the number of the target
//para: none
//tips: none
 void R_Mes_Up(Robot_Sta_Single *robot);
//time: 2016.8.14
//func: update of our struct "Robot_Sta_Single"
//para: struct "Robot_Sta_Single"
//tips:none
 void Robot_Sta_Copy(Robot_Sta_Single *robot1, Robot_Sta_Single *robot2);
//time: 2016.8.14
//func: Copy the message of the target
//para: all the message of one target
//tips: copy 1 to 2
 void R_Watch_Refresh(void);
//time: 2016.8.14
//func: update of struct "R_Watch"
//para: none
//tips: none
 void Refresh(float height);
//time: 2016.8.15
//func: refresh all of the messages
//para: none
//tips: none
 void RoaRec_Long_Cal(RotatedRect res, vector<Point>&longcenter);
//time: 2016.8.20
//func: 找到旋转矩形长边对应的中心点
//para: none
//tips: none
 void RoaRec_Draw(Mat& img, RotatedRect res);
//time: 2016.8.20
//func: 画出旋转矩形的相关信息
//para: none
//tips: none
 void Sig_Obj_Tra(Mat src, Robot_Sta_Single* rssig);
//时间：2016.8.28
//功能：对单个目标的跟踪
//参数：图像，单个机器人结构体指针
//tips: 暂无
extern void Sig_Match(Robot_Sta_Single* rssig, Mat src, float height);
//时间：2016.9.15
//功能：判断单个目标是否匹配
//参数：单个机器人结构体指针
//tips: 暂无
extern void All_Obj_Tra(Mat img, float height);
//时间：2016.8.29
//功能：所有目标的跟踪
//参数：暂无
//tips: 暂无
void Green_Tra_Back(Mat src, Mat& backproject, Rect rect);
//时间：2016.9.12
//功能：绿色目标的二值化
//参数：多通道绿色目标图
//tips: 暂无
extern void Dec_Tra_Match(Robot_Sta_All* r, vector<Rect>Red_Out, vector<Rect>Green_Out);
//时间：2016.9.16
//功能：检测和跟踪的目标的匹配
//参数：所有机器人结构体指针，检测结果红色向量，检测结果绿色向量
//tips: 第一版的函数只适用于在检测-跟踪的连接上
extern void Tra_Pos_Predict(Robot_Sta_Single* rssig, float scale);
//时间：2016.9.15
//功能：跟踪目标的预测
//参数：单个机器人结构体，方框缩放尺度
//tips: 第一版的函数只适用于在检测-跟踪的连接上
#endif
