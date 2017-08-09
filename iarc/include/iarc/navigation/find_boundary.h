#ifndef _FIND_BOUNDARY_H_
#define _FIND_BOUNDARY_H_

#include "iarc/navigation/common.h"

//返回值：0——没有边界，5——检测出边界，但无法判断是那一条， 2——2号边界， 4——4号边界 //FunctionFlag：判断方式，仅仅用于调试
int FindWhiteBoundary90(const vector<vector<Vec4f>> &LineD90, const vector<vector<Vec4f>> &LineD0, Vec4f &Boundary90, int &FunctionFlag);
//返回值：0——没有边界，5——检测出边界，但无法判断是那一条， 1——1号边界， 3——3号边界
int FindWhiteBoundary0(const vector<vector<Vec4f>> &LineD0, const vector<vector<Vec4f>> &LineD90, Vec4f &Boundary0, int &FunctionFlag);

int FindRGBoundary0(const vector<vector<Vec4f>> &LineRG0, Vec4f &BoundaryRG0, int Flag);

int FindRed1Boundary(const vector<vector<Vec4f>> &LineR, Vec4f &BoundaryR);

#endif