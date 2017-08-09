#ifndef _THRESHOLD_IMAGE_H_
#define _THRESHOLD_IMAGE_H_
#include "iarc/navigation/common.h"

void ThresholdWhite(const Mat &_ImageSource, Mat & ImageLine);
void ThresholdRed(const Mat &_ImageSource, Mat & ImageLine);
void ThresholdGreen(const Mat &_ImageSource, Mat & ImageLine);
Mat thinImage(const Mat & src, const int maxIterations);
#endif