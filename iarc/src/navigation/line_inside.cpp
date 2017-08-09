#include "iarc/navigation/line_inside.h"
#include "iarc/navigation/math.h"

void LineInsideFilter(const vector<vector<Vec4f>> &Lines, vector<vector<Vec4f>> &Lines1)
{
    Lines1.clear();
    
    for(int i = 0; i < Lines.size(); i++)
    {
	if(Lines[i].size() >= 2)
	{
	    Lines1.push_back(Lines[i]);
	}
    }
}

int LineInside0(const float& Position0, const vector<Vec4f> &Line0AfterFit, float &Error0, int &FirstLineNum)
{    
    if(Line0AfterFit.size() >= 2)
    {
	vector<float> AllLineMiddleY;
	float MiddleY1;
	float MiddleY2;
	for(int i = 0; i + 1 < Line0AfterFit.size(); i++)
	{
	    int j = i + 1;
	    
	    MiddleY1 = (Line0AfterFit[i][1] + Line0AfterFit[i][3])/2;
	    AllLineMiddleY.push_back(MiddleY1);
	    
	    MiddleY2 = (Line0AfterFit[j][1] + Line0AfterFit[j][3])/2;
	    
	    bool TwoLineDistanceCheck = true;
	    TwoLineDistanceCheck = TwoLineDistanceCheck && (MiddleY2 - MiddleY1 > 1.0 - DistanceThresholdTwoLines);
	    TwoLineDistanceCheck = TwoLineDistanceCheck && (MiddleY2 - MiddleY1 < 1.0 +DistanceThresholdTwoLines);
	    
	    if(!TwoLineDistanceCheck)
	    {
		return 2;
	    }
	    if(j == Line0AfterFit.size() -1)
	    {
		AllLineMiddleY.push_back(MiddleY2);
	    }
	}
	
	float Error = 0;
	
	FirstLineNum = FindLine0Num(Position0, AllLineMiddleY[0]);
	
	if(FirstLineNum == -1)
	{
	    return 2;
	}
	else
	{
	    float LineMeasure0 = Position0 + AllLineMiddleY[0];
	    Error += LineMeasure0 - Line0RealCoordinate[FirstLineNum];
	    for(int i = 1; i < AllLineMiddleY.size(); i++)
	    {
		LineMeasure0 = Position0 + AllLineMiddleY[i];
		
		if(FirstLineNum + i > Line0Max)
		{
		    return 2;
		}
		
		bool Middle0Check = true;
		
		Middle0Check = Middle0Check && (LineMeasure0 > Line0RealCoordinate[FirstLineNum + i] - DistanceThresholdTwoLines);
		Middle0Check = Middle0Check && (LineMeasure0 < Line0RealCoordinate[FirstLineNum + i] + DistanceThresholdTwoLines);
		
		if(Middle0Check)
		{
		    Error += LineMeasure0 - Line0RealCoordinate[FirstLineNum + i];
		}
		else
		{
		    return 2;
		}
	    }
	}
	
	Error0 = Error / AllLineMiddleY.size();
	return 1;	
    }
    else if(Line0AfterFit.size() == 1)
    {
	float MiddleY = 0;
	MiddleY = (Line0AfterFit[0][1] + Line0AfterFit[0][3])/2;
	
	FirstLineNum = FindLine0Num(Position0, MiddleY);
	if(FirstLineNum ==  -1)
	{
	    return 2;
	}
	else
	{
	    float LineMeasure0 = Position0 + MiddleY;
	    Error0 = LineMeasure0 - Line0RealCoordinate[FirstLineNum];
	    return 1;
	}
    }
    else
    {
	return 0;
    }

}
int LineInside90(const float& Position90, const vector<Vec4f> &Line90AfterFit, float &Error90, int &FirstLineNum)
{   
    if(Line90AfterFit.size() >= 2)
    {
	vector<float> AllLineMiddleX;
	float MiddleX1;
	float MiddleX2;
	for(int i = 0; i + 1 < Line90AfterFit.size(); i++)
	{
	    int j = i + 1;
	    
	    MiddleX1 = (Line90AfterFit[i][0] + Line90AfterFit[i][2])/2;
	    AllLineMiddleX.push_back(MiddleX1);
	    
	    MiddleX2 = (Line90AfterFit[j][0] + Line90AfterFit[j][2])/2;
	    
	    bool TwoLineDistanceCheck = true;
	    TwoLineDistanceCheck = TwoLineDistanceCheck && (MiddleX2 - MiddleX1 > 1.0 - DistanceThresholdTwoLines);
	    TwoLineDistanceCheck = TwoLineDistanceCheck && (MiddleX2 - MiddleX1 < 1.0 +DistanceThresholdTwoLines);
	    
	    if(!TwoLineDistanceCheck)
	    {
		return 2;
	    }
	    if(j == Line90AfterFit.size() -1)
	    {
		AllLineMiddleX.push_back(MiddleX2);
	    }
	}
	
	float Error = 0;
	
	FirstLineNum = FindLine90Num(Position90, AllLineMiddleX[0]);
	
	if(FirstLineNum == -1)
	{
	    return 2;
	}
	else
	{
	    float LineMeasure90 = Position90 + AllLineMiddleX[0];
	    Error += LineMeasure90 - Line90RealCoordinate[FirstLineNum];
	    for(int i = 1; i < AllLineMiddleX.size(); i++)
	    {
		LineMeasure90 = Position90 + AllLineMiddleX[i];
		
		if(FirstLineNum + i > Line0Max)
		{
		    return 2;
		}
		
		bool Middle90Check = true;
		
		Middle90Check = Middle90Check && (LineMeasure90 > Line90RealCoordinate[FirstLineNum + i] - DistanceThresholdTwoLines);
		Middle90Check = Middle90Check && (LineMeasure90 < Line90RealCoordinate[FirstLineNum + i] + DistanceThresholdTwoLines);
		
		if(Middle90Check)
		{
		    Error += LineMeasure90 - Line0RealCoordinate[FirstLineNum + i];
		}
		else
		{
		    return 2;
		}
	    }
	}
	
	Error90 = Error / AllLineMiddleX.size();
	return 1;
    }
    else if(Line90AfterFit.size() == 1)
    {
	float MiddleX;
	MiddleX = (Line90AfterFit[0][0] + Line90AfterFit[0][2]) / 2; 
	FirstLineNum = FindLine90Num(Position90, MiddleX);
	
	if(FirstLineNum == -1)
	{
	    return 2;
	}
	else
	{
	    float LineMeasure90 = Position90 + MiddleX;
	    Error90 = LineMeasure90 - Line90RealCoordinate[FirstLineNum];
	    return 1;
	}
    }
    else
    {
	return 0;
    }
}
int FindLine0Num(const float &Position0, const float &Middle0)
{
    for(int i = 0; i < Line0Max; i++)
    {
	float Middle0RealCoordinate = Position0 + Middle0;
	
	bool Middle0Check = true;
	
	Middle0Check = Middle0Check && (Middle0RealCoordinate > Line0RealCoordinate[i] - DistanceThresholdTwoLines);
	Middle0Check = Middle0Check && (Middle0RealCoordinate < Line0RealCoordinate[i] + DistanceThresholdTwoLines);
	
	if(Middle0Check)
	{
	    return i;
	}
    }
    return -1;
}
int FindLine90Num(const float &Position90, const float &Middle90)
{
    for(int i = 0; i < Line90Max; i++)
    {
	float Middle90RealCoordinate = Position90 + Middle90;
	
	bool Middle90Check = true;
	
	Middle90Check = Middle90Check && (Middle90RealCoordinate > Line90RealCoordinate[i] - DistanceThresholdTwoLines);
	Middle90Check = Middle90Check && (Middle90RealCoordinate < Line0RealCoordinate[i] + DistanceThresholdTwoLines);
	
	if(Middle90Check)
	{
	    return i;
	}
    }
    return -1;
}