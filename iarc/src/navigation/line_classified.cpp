#include "iarc/navigation/line_classified.h"
#include "iarc/navigation/math.h"

void ClassifiedByAngle(const vector<Vec4i> &RawLine, vector<Vec4f> &Line0, vector<Vec4f> &Line90, const Mat& R_cz_pic, const float &Height)
{
    Line0.clear();
    Line90.clear();
    
    Point2f PointInPic1;
    Point2f PointInPic2;
    Point2f PointInWorld1;
    Point2f PointInWorld2;
    
    float AngleOffsetAll = 0;
    
    Vec4f LineTemp;
    for(int i = 0; i < RawLine.size(); i++)
    {
	PointInPic1.x = RawLine[i][0];
	PointInPic1.y = RawLine[i][1];
	PointInPic2.x = RawLine[i][2];
	PointInPic2.y = RawLine[i][3];
	
	PointPic2CZ(PointInPic1, PointInWorld1, R_cz_pic, Height);
	PointPic2CZ(PointInPic2, PointInWorld2, R_cz_pic, Height);
	
	float Angle;
	
	Angle = atan2(PointInWorld1.y - PointInWorld2.y, PointInWorld1.x - PointInWorld2.x);
	
	if(Angle < 0)
	{
	    Angle = Angle + CV_PI;
	}
	
	LineTemp[0] = PointInWorld1.x;
	LineTemp[1] = PointInWorld1.y;
	LineTemp[2] = PointInWorld2.x;
	LineTemp[3] = PointInWorld2.y;
	
	if(fabs(Angle - CV_PI / 2) < AngleThreshold)
	{
	    Line90.push_back(LineTemp);
	    AngleOffsetAll += CV_PI / 2 - Angle;
	}
	else if(Angle < AngleThreshold || Angle > CV_PI - AngleThreshold)
	{
	    Line0.push_back(LineTemp);
	    if(Angle < AngleThreshold)
	    {
		AngleOffsetAll += 0 - Angle; 
	    }
	    else
	    {
		AngleOffsetAll += CV_PI - Angle;
	    }
	}
    }
    int LineNumber = Line0.size() + Line90.size();
    if(LineNumber > 2)
    {
	CompensateAngle += AngleOffsetAll / LineNumber;
    }
}
void ClassifiedByAngle(const vector<Vec4i> &RawLine, vector<Vec4f> &LineClassified, const Mat&R_cz_pic, const float &Height)
{
    LineClassified.clear();
    Point2f PointInPic1;
    Point2f PointInPic2;
    Point2f PointInWorld1;
    Point2f PointInWorld2;
     Vec4f LineTemp;
    for(int i = 0; i < RawLine.size(); i++)
    {
	PointInPic1.x = RawLine[i][0];
	PointInPic1.y = RawLine[i][1];
	PointInPic2.x = RawLine[i][2];
	PointInPic2.y = RawLine[i][3];
	
	PointPic2CZ(PointInPic1, PointInWorld1, R_cz_pic, Height);
	PointPic2CZ(PointInPic2, PointInWorld2, R_cz_pic, Height);
	
	float Angle;
	
	Angle = atan2(PointInWorld1.y - PointInWorld2.y, PointInWorld1.x - PointInWorld2.x);
	
	if(Angle < 0)
	{
	    Angle = Angle + CV_PI;
	}
	
	LineTemp[0] = PointInWorld1.x;
	LineTemp[1] = PointInWorld1.y;
	LineTemp[2] = PointInWorld2.x;
	LineTemp[3] = PointInWorld2.y;
	
	 if(Angle < AngleThreshold || Angle > CV_PI - AngleThreshold)
	{
	    LineClassified.push_back(LineTemp);
	}
    }
}
void ClassifiedByDistance(const vector<Vec4f> &RawLine, vector<vector<Vec4f>> &LineClassified)
{
    LineClassified.clear();
    vector<bool> LineFlag(RawLine.size(), false);
    
    for(int i = 0; i < RawLine.size(); i++)
    {
	if(LineFlag[i])
	{
	    continue;
	}
	vector<Vec4f> LinesC;
	LineFlag[i] = true;
	LinesC.push_back(RawLine[i]);
	
	for(int j = i + 1; j < RawLine.size(); j++)
	{
	    if(LineFlag[j])
	    {
		continue;
	    }
	    float XMiddle = (RawLine[j][0] + RawLine[j][2]) / 2;
	    float YMiddle = (RawLine[j][1] + RawLine[j][3]) / 2;
	    float Distance = DistanceP2L(RawLine[i], Point2f(XMiddle, YMiddle));
	    
	    if(Distance < DistanceThreshold)
	    {
		LinesC.push_back(RawLine[j]);
		LineFlag[j] = true;
	    }
	}
	LineClassified.push_back(LinesC);
    }
}