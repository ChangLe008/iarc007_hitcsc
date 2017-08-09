#ifndef _LINE_INSIDE_H_
#define _LINE_INSIDE_H_

#include "iarc/navigation/common.h"

void LineInsideFilter(const vector<vector<Vec4f>> &Lines, vector<vector<Vec4f>> &Lines1);

//LineNum——匹配的第一条直线属于那一条，仅仅用于调试
//返回值——0：没检测出直线，1——检测出直线并校正，2——检测出直线但不满足条件未校正
int LineInside0(const float& Position0, const vector<Vec4f> &Line0AfterFit, float &Error0, int &FirstLineNum);
int LineInside90(const float& Position90, const vector<Vec4f> &Line90AfterFit, float &Error90, int &FirstLineNum);

//返回值： -1——没有找到符合条件的直线
int FindLine0Num(const float &PositionY, const float &MiddleY);
int FindLine90Num(const float &Position90, const float &Middle90);
#endif