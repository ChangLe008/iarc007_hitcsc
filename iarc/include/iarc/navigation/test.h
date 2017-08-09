#ifndef _TEST_H_
#define _TEST_H_

#include "iarc/navigation/common.h"

void AllLineTest(const vector<Vec4i> &Lines, const Mat& Image, const Mat &R_cz_pic, const float &Height);
void AngleClassifiedTest(vector<Vec4f> &Line0, const vector<Vec4f> &Line90, const Mat& R_cz_pic, const float &Height, const Mat& Image);
void DistanceClassifiedTest(const vector<vector<Vec4f>> &Lines, const Mat & R_cz_pic, const float &Height, const Mat& Image, int Flag);

//LineFLag——边界编号，FunctionFlag——判断方式编号：1——平行线，2——垂直线， 3——位置， 0——颜色
void BoundaryTest(const Vec4f &Line, const Mat &R_cz_pic, const float &Height,const Mat &Image, int LineFLag, int FunctionFlag);
void DistanceTest(const vector<Vec4f> &Line, const Mat& R_cz_pic, const float &Height, const Mat &Image, int Flag);

void LineInSideCorrectTest(const vector<Vec4f> &Lines, const int & FirstLineNum, const float &Error, const int &LineFlag, const Mat &R_cz_pic, const float &Height, const Mat &Image, int Corrected = 1);
void AllLineInsideTest(const vector<Vec4f> &Lines, const int &Flag, const Mat &Image, const Mat &R_cz_pic, const float &Height);

void ResultTest(const Mat &Image, const Mat& Position);

#endif