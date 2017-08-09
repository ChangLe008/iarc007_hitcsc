#include "iarc/navigation/common.h"

float CompensateAngle = 0;

float AngleThreshold = 30.0 / 180 * CV_PI;
float DistanceThreshold = 0.1; //小于该值即为同一条直线
float DistanceThresholdTwoLines = 0.3; //内部直线之间距离阈值

float LengthThreshold = 0.5; //直线长度小于该值则认为检测错误

float WhiteLineWidth = 0.20; //大白线宽度
 
float InsideLineErrorThreshold = 0.1; //内部直线的校正值大于该值则不校正
float BoundaryErrorThreshold = 6;
 
int Line90Max = 5;
int Line0Max = 7;

float Line90RealCoordinate[23] = {0, 0.92, 1.91, 2.9, 3.89, 4.71};
float Line0RealCoordinate[23] = { 0, 1.01, 2.00, 3.02, 4.00, 5.03, 5.99, 6.8};