#include "iarc/navigation/find_boundary.h"
#include "iarc/navigation/math.h"
#include "iarc/navigation/test.h"

float Line0MiddleX(const vector<vector<Vec4f>> &LineD0)
{
    float MiddleX = 0;
    int LineNum = 0;
    for(int i = 0; i < LineD0.size(); i++)
    {
	for(int j = 0; j < LineD0[i].size(); j++)
	{
	    MiddleX += (LineD0[i][j][0] + LineD0[i][j][2]) / 2;
	    LineNum ++;
	}
    }
    MiddleX = MiddleX / LineNum;
    return MiddleX;
}
float Line90MiddleY(const vector<vector<Vec4f>> &LineD90)
{
    float MiddleY = 0;
    int LineNum = 0;
    for(int i = 0; i < LineD90.size(); i++)
    {
	for(int j = 0; j < LineD90[i].size(); j++)
	{
	    MiddleY += (LineD90[i][j][1] + LineD90[i][j][3]) / 2;
	    LineNum ++;
	}
    }
    MiddleY = MiddleY / LineNum;
    return MiddleY;
}
// int FindWhiteBoundary90(const vector<vector<Vec4f>> &LineD90, const vector<vector<Vec4f>> &LineD0, Vec4f &Boundary90, int &FunctionFlag)
// {
//     for(int j = 1; j <LineD90.size(); j++ ) // 第j类
//     {
// 	float Distance = 0;
// 	int DistanceNumber = 0;
// 	vector<Point2f> PointInBoundary;
// 	for(int jj = 0; jj < LineD90[j].size(); jj++) // 第j类第jj条
// 	{
// 	    float XMiddle;
// 	    float YMiddle;
// 	    XMiddle = (LineD90[j][jj][0] + LineD90[j][jj][2]) / 2;
// 	    YMiddle = (LineD90[j][jj][1] + LineD90[j][jj][3]) / 2;
// 	    for(int i = 0; i < LineD90[0].size(); i++)
// 	    {
// 		Distance += DistanceP2L(LineD90[0][i], Point2f(XMiddle, YMiddle));
// 		DistanceNumber ++;
// 	    }
// 	    PointInBoundary.push_back(Point2f(LineD90[j][jj][0], LineD90[j][jj][1]));
// 	    PointInBoundary.push_back(Point2f(LineD90[j][jj][2], LineD90[j][jj][3]));
// 	}
// 	Distance = Distance / DistanceNumber;
// 	if(Distance > 0.8)
// 	{
// 	    break;
// 	}
// 	
// 	bool BoundaryFound = true;
// 	BoundaryFound = BoundaryFound && (Distance > WhiteLineWidth * 0.5);
// 	BoundaryFound = BoundaryFound && (Distance < WhiteLineWidth * 1.5);
// 	
// 	if(BoundaryFound)
// 	{
// 	    LineFit90(PointInBoundary, Boundary90);
// 	    float BoundaryMiddlePointX = (Boundary90[0] + Boundary90[2]) / 2;
// 	    if(LineD90.size() >= j + 2) //TODO
// 	    {
// 		FunctionFlag = 1;
// 		return 4;
// 	    }
// 	    else if(LineD0.size() >= 1)
// 	    {
// 		float MiddleXOFLine0 = Line0MiddleX(LineD0);
// 		if(BoundaryMiddlePointX < MiddleXOFLine0)
// 		{
// 		    FunctionFlag = 2;
// 		    return 4;
// 		}
// 		else
// 		{
// 		    FunctionFlag = 2;
// 		    return 2;
// 		}
// 	    }
// 	    else
// 	    {
// 		return 5;
// 	    }
// 	}
//     }
//     
//     for(int j = LineD90.size() - 2; j >= 0; j -- ) // 第j类
//     {
// 	float Distance = 0;
// 	int DistanceNumber = 0;
// 	vector<Point2f> PointInBoundary;
// 	for(int jj = 0; jj < LineD90[j].size(); jj++) // 第j类第jj条
// 	{
// 	    float XMiddle;
// 	    float YMiddle;
// 	    XMiddle = (LineD90[j][jj][0] + LineD90[j][jj][2]) / 2;
// 	    YMiddle = (LineD90[j][jj][1] + LineD90[j][jj][3]) / 2;
// 	    for(int i = 0; i < LineD90[LineD90.size() - 1].size(); i++)
// 	    {
// 		Distance += DistanceP2L(LineD90[LineD90.size() - 1][i], Point2f(XMiddle, YMiddle));
// 		DistanceNumber ++;
// 	    }
// 	    PointInBoundary.push_back(Point2f(LineD90[j][jj][0], LineD90[j][jj][1]));
// 	    PointInBoundary.push_back(Point2f(LineD90[j][jj][2], LineD90[j][jj][3]));
// 	}
// 	Distance = Distance / DistanceNumber;
// 	if(Distance > 0.8)
// 	{
// 	    break;
// 	}
// 	
// 	bool BoundaryFound = true;
// 	BoundaryFound = BoundaryFound && (Distance > WhiteLineWidth * 0.5);
// 	BoundaryFound = BoundaryFound && (Distance < WhiteLineWidth * 1.5);
// 	
// 	if(BoundaryFound)
// 	{
// 	    LineFit90(PointInBoundary, Boundary90);
// 	    float BoundaryMiddlePointX = (Boundary90[0] + Boundary90[2]) / 2;
// 	    if(j > 0) //TODO
// 	    {
// 		FunctionFlag = 1;
// 		return 2;
// 	    }
// 	    else if(LineD0.size() >= 1)
// 	    {
// 		float MiddleXOFLine0 = Line0MiddleX(LineD0);
// 		if(BoundaryMiddlePointX < MiddleXOFLine0)
// 		{
// 		    FunctionFlag = 2;
// 		    return 4;
// 		}
// 		else
// 		{
// 		    FunctionFlag = 2;
// 		    return 2;
// 		}
// 	    }
// 	    else
// 	    {
// 		return 5;
// 	    }
// 	}
//     }
//     return 0;
// }
// int FindWhiteBoundary0(const vector<vector<Vec4f>> &LineD0, const vector<vector<Vec4f>> &LineD90, Vec4f &Boundary0, int &FunctionFlag)
// {
//     for(int j = 1; j <LineD0.size(); j++ ) // 第j类
//     {
// 	float Distance = 0;
// 	int DistanceNumber = 0;
// 	vector<Point2f> PointInBoundary;
// 	for(int jj = 0; jj < LineD0[j].size(); jj++) // 第j类第jj条
// 	{
// 	    float XMiddle;
// 	    float YMiddle;
// 	    XMiddle = (LineD0[j][jj][0] + LineD0[j][jj][2]) / 2;
// 	    YMiddle = (LineD0[j][jj][1] + LineD0[j][jj][3]) / 2;
// 	    for(int i = 0; i < LineD0[0].size(); i++)
// 	    {
// 		Distance += DistanceP2L(LineD0[0][i], Point2f(XMiddle, YMiddle));
// 		DistanceNumber ++;
// 	    }
// 	    PointInBoundary.push_back(Point2f(LineD0[j][jj][0], LineD0[j][jj][1]));
// 	    PointInBoundary.push_back(Point2f(LineD0[j][jj][2], LineD0[j][jj][3]));
// 	}
// 	Distance = Distance / DistanceNumber;
// 	if(Distance > 0.8)
// 	{
// 	    break;
// 	}
// 	
// 	bool BoundaryFound = true;
// 	BoundaryFound = BoundaryFound && (Distance > WhiteLineWidth * 0.5);
// 	BoundaryFound = BoundaryFound && (Distance < WhiteLineWidth * 1.5);
// 	
// 	if(BoundaryFound)
// 	{
// 	    LineFit0(PointInBoundary, Boundary0);
// 	    float BoundaryMiddlePointY = (Boundary0[1] + Boundary0[3]) / 2;
// 	    if(LineD0.size() >= j + 2) //TODO
// 	    {
// 		FunctionFlag = 1;
// 		return 1;
// 	    }
// 	    else if(LineD90.size() >= 1)
// 	    {
// 		float MiddleYOFLine90 = Line90MiddleY(LineD90);
// 		if(BoundaryMiddlePointY < MiddleYOFLine90)
// 		{
// 		    FunctionFlag = 2;
// 		    return 1;
// 		}
// 		else
// 		{
// 		    FunctionFlag = 2;
// 		    return 3;
// 		}
// 	    }
// 	    else
// 	    {
// 		return 5;
// 	    }
// 	}
//     }
//     
//     for(int j = LineD0.size() - 2; j >= 0; j -- ) // 第j类
//     {
// 	float Distance = 0;
// 	int DistanceNumber = 0;
// 	vector<Point2f> PointInBoundary;
// 	for(int jj = 0; jj < LineD0[j].size(); jj++) // 第j类第jj条
// 	{
// 	    float XMiddle;
// 	    float YMiddle;
// 	    XMiddle = (LineD0[j][jj][0] + LineD0[j][jj][2]) / 2;
// 	    YMiddle = (LineD0[j][jj][1] + LineD0[j][jj][3]) / 2;
// 	    for(int i = 0; i < LineD0[LineD0.size() - 1].size(); i++)
// 	    {
// 		Distance += DistanceP2L(LineD0[LineD0.size() - 1][i], Point2f(XMiddle, YMiddle));
// 		DistanceNumber ++;
// 	    }
// 	    PointInBoundary.push_back(Point2f(LineD0[j][jj][0], LineD0[j][jj][1]));
// 	    PointInBoundary.push_back(Point2f(LineD0[j][jj][2], LineD0[j][jj][3]));
// 	}
// 	Distance = Distance / DistanceNumber;
// 	if(Distance > 0.8)
// 	{
// 	    break;
// 	}
// 	
// 	bool BoundaryFound = true;
// 	BoundaryFound = BoundaryFound && (Distance > WhiteLineWidth * 0.5);
// 	BoundaryFound = BoundaryFound && (Distance < WhiteLineWidth * 1.5);
// 	
// 	if(BoundaryFound)
// 	{
// 	    LineFit0(PointInBoundary, Boundary0);
// 	    float BoundaryMiddlePointY = (Boundary0[1] + Boundary0[3]) / 2;
// 	    if(j > 0) //TODO
// 	    {
// 		FunctionFlag = 1;
// 		return 3;
// 	    }
// 	    else if(LineD90.size() >= 1)
// 	    {
// 		float MiddleYOFLine90 = Line90MiddleY(LineD90);
// 		if(BoundaryMiddlePointY< MiddleYOFLine90)
// 		{
// 		    FunctionFlag = 1;
// 		    return 1;
// 		}
// 		else
// 		{
// 		    FunctionFlag = 2;
// 		    return 3;
// 		}
// 	    }
// 	    else
// 	    {
// 		return 5;
// 	    }
// 	}
//     }
//     return 0;
// }
int FindWhiteBoundary90(const vector<vector<Vec4f>> &LineD90, const vector<vector<Vec4f>> &LineD0, Vec4f &Boundary90, int &FunctionFlag)
{
    bool FoundButNODecide = false;
    vector<Vec4f> Line90;
    Vec4f LineTemp;
    for(int i = 0; i < LineD90.size(); i++)
    {
	vector<Point2f> PointInLine;
	for(int ii = 0; ii < LineD90[i].size(); ii++)
	{
	    PointInLine.push_back(Point2f(LineD90[i][ii][0], LineD90[i][ii][1]));
	    PointInLine.push_back(Point2f(LineD90[i][ii][2], LineD90[i][ii][3]));
	}
	LineFit90(PointInLine, LineTemp);
	Point2f Point1;
	Point2f Point2;
	Point1.x = LineTemp[0];
	Point1.y = LineTemp[1];
	Point2.x = LineTemp[2];
	Point2.y = LineTemp[3];
	float Length;
	Length = (Point1.x - Point2.x) * (Point1.x - Point2.x) + (Point1.y - Point2.y) * (Point1.y - Point2.y);
	Length = sqrt(Length);
	if(Length > LengthThreshold)
	{
	    Line90.push_back(LineTemp);
	}
	else
	{
	    cout << "hahahahha" << endl;
	}
    }
    if(Line90.size() > 1)
    {

	for(int i = 1; i < Line90.size(); i++) 
	{
	    float XMiddle;
	    float YMiddle;
	    XMiddle = (Line90[i][0] + Line90[i][2]) / 2;
	    YMiddle = (Line90[i][1] + Line90[i][3]) / 2;

	    float Distance;
	    Distance = DistanceP2L(Line90[0],Point2f(XMiddle, YMiddle));
	    if(Distance > 0.8)
	    {
		break;
	    }
	    
	    bool BoundaryFound = true;
	    BoundaryFound = BoundaryFound && (Distance > WhiteLineWidth * 0.5);
	    BoundaryFound = BoundaryFound && (Distance < WhiteLineWidth * 1.5);
	    
	    if(BoundaryFound)
	    {
		for(int j = 0; j < 4; j++)
		{
		    Boundary90[j] = Line90[i][j];
		}
		if(Line90.size() >= i + 2)
		{
		    FunctionFlag = 1;
		    return 4;
		}
		else if(LineD0.size() > 0)
		{
		    float MiddleXOFLine0 = Line0MiddleX(LineD0);
		    float BoundaryMiddlePointX = (Line90[i][0] + Line90[i][2]) /2;
		    
		    if(BoundaryMiddlePointX < MiddleXOFLine0)
		    {
			FunctionFlag = 2;
			return 4;
		    }
		    else
		    {
			for(int j = 0; j < 4; j++)
			{
			    Boundary90[j] = Line90[0][j];
			}
			FunctionFlag = 2;
			return 2;
		    }
		}
		else 
		{
		    FoundButNODecide = true;
		}
	    }
	}
	
	for(int i = Line90.size() - 2; i >= 0; i --)
	{
	    float XMiddle;
	    float YMiddle;
	    XMiddle = (Line90[i][0] + Line90[i][2]) / 2;
	    YMiddle = (Line90[i][1] + Line90[i][3]) / 2;

	    float Distance;
	    Distance = DistanceP2L(Line90[Line90.size() - 1], Point2f(XMiddle, YMiddle));
	    if(Distance > 0.8)
	    {
		return 0;
	    }
	    
	    bool BoundaryFound = true;
	    BoundaryFound = BoundaryFound && (Distance > WhiteLineWidth * 0.5);
	    BoundaryFound = BoundaryFound && (Distance < WhiteLineWidth * 1.5);
	    
	    if(BoundaryFound)
	    {
		for(int j = 0; j < 4; j++)
		{
		    Boundary90[j] = Line90[i][j];
		}
		if(i > 0)
		{
		    FunctionFlag = 1;
		    return 2;
		}
		else if(LineD0.size() > 0)
		{
		    float MiddleXOFLine0 = Line0MiddleX(LineD0);
		    float BoundaryMiddlePointX = (Line90[i][0] + Line90[i][2]) /2;
		    
		    if(BoundaryMiddlePointX < MiddleXOFLine0)
		    {
			for(int j = 0; j < 4; j++)
			{
			    Boundary90[j] = Line90[Line90.size() - 1][j];
			}
			FunctionFlag = 2;
			return 4;
		    }
		    else
		    {
			FunctionFlag = 2;
			return 2;
		    }
		}
		else 
		{
		    FoundButNODecide = true;
		}
	    }   
	}
	if(FoundButNODecide)
	{
	    return 5;
	}
    }
    return 0;
}
int FindWhiteBoundary0(const vector<vector<Vec4f>> &LineD0, const vector<vector<Vec4f>> &LineD90, Vec4f &Boundary0, int &FunctionFlag)
{
    bool FoundButNODecide = false;
    vector<Vec4f> Line0;
    Vec4f LineTemp;
    for(int i = 0; i < LineD0.size(); i++)
    {
	vector<Point2f> PointInLine;
	for(int ii = 0; ii < LineD0[i].size(); ii++)
	{
	    PointInLine.push_back(Point2f(LineD0[i][ii][0], LineD0[i][ii][1]));
	    PointInLine.push_back(Point2f(LineD0[i][ii][2], LineD0[i][ii][3]));
	}
	LineFit0(PointInLine, LineTemp);
	Point2f Point1;
	Point2f Point2;
	Point1.x = LineTemp[0];
	Point1.y = LineTemp[1];
	Point2.x = LineTemp[2];
	Point2.y = LineTemp[3];
	float Length;
	Length = (Point1.x - Point2.x) * (Point1.x - Point2.x) + (Point1.y - Point2.y) * (Point1.y - Point2.y);
	Length = sqrt(Length);
	if(Length > LengthThreshold)
	{
	    Line0.push_back(LineTemp);
	}
    }
    if(Line0.size() > 1)
    {

	for(int i = 1; i < Line0.size(); i++) 
	{
	    float XMiddle;
	    float YMiddle;
	    XMiddle = (Line0[i][0] + Line0[i][2]) / 2;
	    YMiddle = (Line0[i][1] + Line0[i][3]) / 2;

	    float Distance;
	    Distance = DistanceP2L(Line0[0],Point2f(XMiddle, YMiddle));
	    if(Distance > 0.8)
	    {
		break;
	    }
	    
	    bool BoundaryFound = true;
	    BoundaryFound = BoundaryFound && (Distance > WhiteLineWidth * 0.5);
	    BoundaryFound = BoundaryFound && (Distance < WhiteLineWidth * 1.5);
	    
	    if(BoundaryFound)
	    {
		for(int j = 0; j < 4; j++)
		{
		    Boundary0[j] = Line0[i][j];
		}
		if(Line0.size() >= i + 2)
		{
		    FunctionFlag = 1;
		    return 1;
		}
		else if(LineD90.size() > 0)
		{
		    float MiddleYOFLine90 = Line90MiddleY(LineD90);
		    float BoundaryMiddlePointY = (Line0[i][1] + Line0[i][3]) /2;
		    
		    if(BoundaryMiddlePointY < MiddleYOFLine90)
		    {
			FunctionFlag = 2;
			return 1;
		    }
		    else
		    {
			for(int j = 0; j < 4; j++)
			{
			    Boundary0[j] = Line0[0][j];
			}
			FunctionFlag = 2;
			return 3;
		    }
		}
		else 
		{
		    FoundButNODecide = true;
		}
	    }
	}
	
	for(int i = Line0.size() - 2; i >= 0; i --)
	{
	    float XMiddle;
	    float YMiddle;
	    XMiddle = (Line0[i][0] + Line0[i][2]) / 2;
	    YMiddle = (Line0[i][1] + Line0[i][3]) / 2;

	    float Distance;
	    Distance = DistanceP2L(Line0[Line0.size() - 1], Point2f(XMiddle, YMiddle));
	    if(Distance > 0.8)
	    {
		break;
	    }
	    
	    bool BoundaryFound = true;
	    BoundaryFound = BoundaryFound && (Distance > WhiteLineWidth * 0.5);
	    BoundaryFound = BoundaryFound && (Distance < WhiteLineWidth * 1.5);
	    
	    if(BoundaryFound)
	    {
		for(int j = 0; j < 4; j++)
		{
		    Boundary0[j] = Line0[i][j];
		}
		if(i > 0)
		{
		    FunctionFlag = 1;
		    return 3;
		}
		else if(LineD90.size() > 0)
		{
		    float MiddleYOFLine90 = Line90MiddleY(LineD90);
		    float BoundaryMiddlePointY = (Line0[i][1] + Line0[i][3]) /2;
		    
		    if(BoundaryMiddlePointY < MiddleYOFLine90)
		    {
			for(int j = 0; j < 4; j++)
			{
			    Boundary0[j] = Line0[Line0.size() -1][j];
			}
			FunctionFlag = 2;
			return 1;
		    }
		    else
		    {
			FunctionFlag = 2;
			return 3;
		    }
		}
		else 
		{
		    FoundButNODecide = true;
		}
	    }   
	}
	if(FoundButNODecide)
	{
	    return 5;
	}
    }
    return 0;
}
int FindRGBoundary0(const vector<vector<Vec4f>> &LineRG0, Vec4f &BoundaryRG0, int Flag)
{
    vector<Vec4f> LineRGLengthSelect;
    for(int i = 0; i < LineRG0.size(); i++)
    {
	vector<Point2f> PointInRGLine;
	Vec4f LineRGTemp;
	for(int j = 0; j < LineRG0[i].size(); j++)
	{
	    PointInRGLine.push_back(Point2f(LineRG0[i][j][0], LineRG0[i][j][1]));
	    PointInRGLine.push_back(Point2f(LineRG0[i][j][2], LineRG0[i][j][3]));
	}
	LineFit0(PointInRGLine, LineRGTemp);
	
	float Length;
	Length = (LineRGTemp[0] - LineRGTemp[2]) * (LineRGTemp[0] - LineRGTemp[2]) + (LineRGTemp[1] - LineRGTemp[3]) * (LineRGTemp[1] - LineRGTemp[3]);
	Length = sqrt(Length);
	
	if(Length > LengthThreshold)
	{
	    LineRGLengthSelect.push_back(LineRGTemp);
	}
    }
    
    if(LineRGLengthSelect.size() > 0)
    {
	if(Flag == 3)
	{
	    for(int i = 0; i < 4; i++)
	    {
		BoundaryRG0[i] = LineRGLengthSelect[0][i];
	    }
	    return 3;
	}
	else if(Flag == 1)
	{
	    for(int i = 0; i < 4; i++)
	    {
		BoundaryRG0[i] = LineRGLengthSelect[LineRGLengthSelect.size() - 1][i];
	    }
	    return 1;
	}
	else
	{
	    cout << "Flag 值传入错误" << endl;
	    exit(-1);
	}
    }
    return 0;
}


int FindRed1Boundary(const vector<vector<Vec4f>> &LineR, Vec4f &BoundaryR)
{
    vector<Vec4f> LineRGLengthSelect;
    for(int i = 0; i < LineR.size(); i++)
    {
	vector<Point2f> PointInRGLine;
	Vec4f LineRGTemp;
	for(int j = 0; j < LineR[i].size(); j++)
	{
	    PointInRGLine.push_back(Point2f(LineR[i][j][0], LineR[i][j][1]));
	    PointInRGLine.push_back(Point2f(LineR[i][j][2], LineR[i][j][3]));
	}
	LineFit0(PointInRGLine, LineRGTemp);
	
	float Length;
	Length = (LineRGTemp[0] - LineRGTemp[2]) * (LineRGTemp[0] - LineRGTemp[2]) + (LineRGTemp[1] - LineRGTemp[3]) * (LineRGTemp[1] - LineRGTemp[3]);
	
	Length = sqrt(Length);
	
	if(Length > LengthThreshold)
	{
	    LineRGLengthSelect.push_back(LineRGTemp);
	}
    }
    
    if(LineRGLengthSelect.size() > 0)
    {
	for(int i = 0; i < 4; i++)
	{
	    BoundaryR[i] = LineRGLengthSelect[0][i];
	}
	return 2;
    }
    return 0;
}