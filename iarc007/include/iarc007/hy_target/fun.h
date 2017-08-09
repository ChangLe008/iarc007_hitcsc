/*
 * fun.h
 *
 *  Created on: 2016年8月24日
 *      Author: hou
 */

#ifndef INCLUDE_IARC007_HY_TARGET_FUN_H_
#define INCLUDE_IARC007_HY_TARGET_FUN_H_

#include "iarc007/hy_target/parameter.h"


int panduan(IplImage* img,IplImage* dst,CvPoint* p);
int liantongyu(IplImage* src,float cx[],float cy[],double h);
void creat(int x,int y);
void pushback(int x,int y);
int empty();
void clear();
int RegionGrow(const IplImage* src,IplImage* dst, int u,int v,int threshold,CvPoint* p,double h);
void imageprocessing(IplImage* img,PicPoint *picp,double h);
int featuredetector(IplImage* img,CvPoint2D32f* corners,double h,int u,int v);

#endif /* INCLUDE_IARC007_HY_TARGET_FUN_H_ */
