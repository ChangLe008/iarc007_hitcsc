#ifndef _DECHOG_H_
#define _DECHOG_H_

#include "iarc007/hy_target/parameter.h"
#include "iarc007/hy_target/common.h"

extern void Hog_Init(void);
//时间：2016.8.2
//功能：初始HOG类模板
//参数：暂无，均由函数内部设定
//说明：暂无
extern void Multi_Scale_Obj_Dec(int height, Mat src, HOGDescriptor myHOG, Point start, Size winsize, Size step, float scale, int S, int hflag, int fflag);
//时间：2016.8.2
//功能：多尺度目标检测
//参数：原图，HOG类，SVM类，检测开始点，检测窗口的大小，每一次移动的大小，图片的压缩尺度，图片缩放尺度，高度限制标志位，结果滤波标志位
//说明：暂无
extern void Full_Search_Img(Mat src, Point start, Size winsize, Size stepsize, HOGDescriptor myHOG, vector<Rect>&found, int time, int S, float scale);
//时间：2016.8.8
//功能：对一幅图进行一次全搜索
//参数：图像，起始点，窗口大小，步长，HOG，结果向量，已经循环的次数，尺度因子，缩放尺度
//说明：暂无

#endif
