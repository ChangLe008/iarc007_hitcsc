#ifndef _LINE_CLASSIFIED_H_
#define _LINE_CLASSIFIED_H_

#include "iarc/navigation/common.h"

void ClassifiedByAngle(const vector<Vec4i> &RawLine, vector<Vec4f> &Line0, vector<Vec4f> &Line90, const Mat& R_cz_pic, const float &Height);

void ClassifiedByDistance(const vector<Vec4f> &RawLine, vector<vector<Vec4f>> &LineClassified);

//红绿挑选
void ClassifiedByAngle(const vector<Vec4i> &RawLine,  vector<Vec4f> &LineClassified, const Mat& R_cz_pic, const float& Height);

#endif