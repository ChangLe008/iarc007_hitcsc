#ifndef _COMMON_H_
#define _COMMON_H_

#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <vector>
#include <algorithm>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

extern float WhiteLineWidth;

extern float CompensateAngle;
extern float AngleThreshold;
extern float DistanceThreshold;
extern float DistanceThresholdTwoLines;

extern float LengthThreshold;

extern float Line90RealCoordinate[23];

extern float Line0RealCoordinate[23];

extern int Line90Max;

extern int Line0Max;

extern float InsideLineErrorThreshold;
extern float BoundaryErrorThreshold;

#endif
