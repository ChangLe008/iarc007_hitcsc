#ifndef _FIND_LINE_H
#define _FIND_LINE_H

#include "iarc007/mjh_pos/common.h"


bool FindLine(Mat ImageSource,  vector<Vec4i> &_OutFitLine, int Flag, int PixelThreshold);

void DrawLine(Mat &Image, vector<Vec4i> Line);

void PutClass(Mat& Image, vector<Vec4i> Line, int * ClassOfLine);

#endif