#ifndef _THRESHOLD_IMAGE_H_
#define _THRESHOLD_IMAGE_H_

#include "iarc007/mjh_pos/common.h"

cv::Mat thinImage(const cv::Mat & src, const int maxIterations);
void ThresholdRedLine(const Mat &ImageSource, Mat &ImageBinary);
void ThresholdGreenLine(const Mat &ImageSource, Mat &ImageBinary);
void  ThresholdWhiteLine(const Mat &ImageSource, Mat &ImageBinary, int ThinTime);

#endif