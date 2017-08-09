#ifndef _MJH_MATH_H_
#define _MJH_MATH_H_

#include "iarc007/mjh_pos/common.h"

void Quaternion2Euler(Mat Quaternion, Mat &Eurler);
void Euler2Matrix(Mat Eurler, Mat &R);
#endif