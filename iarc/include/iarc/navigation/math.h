#ifndef _MATH_H_
#define _MATH_H_

#include "iarc/navigation/common.h"

void Quaternion2Euler(Mat Quaternion, Mat &Eurler);
void Euler2Matrix(Mat Eurler, Mat &R);

float FindSign(float a);
void Matrix2Quernion(Mat RM, Mat &Quaternion);

void  Quaternion2Matrix(Mat Quaternion, Mat &R);

void  PointPic2CZ(const Point2f &PointPic, Point2f& PointCZ, const Mat &R_cz_pic, const float &Height);

void  PointCZ2Pic(const Point2f &PointCZ, Point2f& PointPic, const Mat &R_cz_pic, const float &Height);

float DistanceP2L(const Vec4f &Line, const Point2f &P);

bool Comp0(const vector<Vec4f> &vector1, const vector<Vec4f> &vector2);

bool Comp90(const vector<Vec4f> &vector1, const vector<Vec4f> &vector2);

void LineFit90(const vector<Point2f>& P, Vec4f& Line);
void LineFit0(const vector<Point2f>& P, Vec4f& Line);

#endif