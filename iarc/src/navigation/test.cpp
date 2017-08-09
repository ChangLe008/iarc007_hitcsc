#include "iarc/navigation/test.h"
#include "iarc/navigation/math.h"
void AllLineTest(const vector<Vec4i> &Lines, const Mat& Image, const Mat& R_cz_pic, const float &Height)
{
    Mat ImageSource;
    Image.copyTo(ImageSource);
    for(int i = 0; i < Lines.size(); i++)
    {
	Point2f Point1InPic, Point2InPic, Point1InCZ, Point2InCZ, MiddlePoint;
	float Angle;
	Point1InPic.x = Lines[i][0];
	Point1InPic.y = Lines[i][1];
	PointPic2CZ(Point1InPic, Point1InCZ, R_cz_pic, Height);
	Point2InPic.x = Lines[i][2];
	Point2InPic.y = Lines[i][3];
	PointPic2CZ(Point2InPic, Point2InCZ, R_cz_pic, Height);
	
	Angle = atan2(Point2InCZ.y - Point1InCZ.y, Point2InCZ.x - Point1InCZ.x);
	Angle = Angle / CV_PI * 180;
	
	line(ImageSource, Point1InPic, Point2InPic, Scalar(0, 0, 255));
	MiddlePoint.x = (Point1InPic.x + Point2InPic.x) / 2;
	MiddlePoint.y = (Point1InPic.y + Point2InPic.y) / 2;
	
	string Words;
	stringstream os;
	os << Angle;
	Words = os.str();
	putText(ImageSource, Words, MiddlePoint, CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 0));
	
	stringstream os1;
	os1 << CompensateAngle / CV_PI * 180;
	Words = os1.str();
	putText(ImageSource, Words, Point(320, 240), CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 255, 0));
    }
    imshow("所有直线", ImageSource);
}

void AngleClassifiedTest(vector<Vec4f> &Line0, const vector<Vec4f> &Line90, const Mat& R_cz_pic, const float &Height, const Mat& Image)
{
    Mat ImageSource0;
    Mat ImageSource90;
    
    Image.copyTo(ImageSource0);
    Image.copyTo(ImageSource90);
    
    Point2f PointInPic1;
    Point2f PointInPic2;
    Point2f PointInWorld1;
    Point2f PointInWorld2;
    Point2f MiddlePoint;
    
    float Angle; 
    string Words;
    for(int i = 0; i < Line0.size(); i++)
    {
	PointInWorld1.x = Line0[i][0];
	PointInWorld1.y = Line0[i][1];
	PointInWorld2.x = Line0[i][2];
	PointInWorld2.y = Line0[i][3];
	
	PointCZ2Pic(PointInWorld1, PointInPic1, R_cz_pic, Height);
	PointCZ2Pic(PointInWorld2, PointInPic2, R_cz_pic, Height);
	line(ImageSource0, PointInPic1, PointInPic2, Scalar(0, 0, 255));
	
	MiddlePoint.x = (PointInPic1.x + PointInPic2.x) / 2;
	MiddlePoint.y = (PointInPic1.y + PointInPic2.y) / 2;
	
	Angle = atan2(PointInWorld1.y - PointInWorld2.y, PointInWorld1.x - PointInWorld2.x);
	Angle = Angle / CV_PI * 180;
	stringstream os;
	os  << Angle;
	Words = os.str();
	putText(ImageSource0, Words, MiddlePoint, CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 0));
    }
    for(int i = 0; i < Line90.size(); i++)
    {
	PointInWorld1.x = Line90[i][0];
	PointInWorld1.y = Line90[i][1];
	PointInWorld2.x = Line90[i][2];
	PointInWorld2.y = Line90[i][3];
	
	PointCZ2Pic(PointInWorld1, PointInPic1, R_cz_pic, Height);
	PointCZ2Pic(PointInWorld2, PointInPic2, R_cz_pic, Height);
	MiddlePoint.x = (PointInPic1.x + PointInPic2.x) / 2;
	MiddlePoint.y = (PointInPic1.y + PointInPic2.y) / 2;
	line(ImageSource90, PointInPic1, PointInPic2, Scalar(0, 0, 255));
	
	Angle = atan2(PointInWorld1.y - PointInWorld2.y, PointInWorld1.x - PointInWorld2.x);
	Angle = Angle / CV_PI * 180;
	stringstream os1;
	os1 << Angle;
	Words = os1.str();
	putText(ImageSource90, Words, MiddlePoint, CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 0));
    }
    imshow("0度直线", ImageSource0);
    imshow("90度直线", ImageSource90);
}

void DistanceClassifiedTest(const vector<vector<Vec4f>> &Lines, const Mat & R_cz_pic, const float &Height, const Mat& Image, int Flag)
{
    Mat ImageSource;
    Image.copyTo(ImageSource);
    
    Point2f PointInPic1;
    Point2f PointInPic2;
    Point2f PointInWorld1;
    Point2f PointInWorld2;
    Point2f MiddlePoint;
    
    string Words;
    
    for(int i = 0; i < Lines.size(); i++)
    {
	for(int j = 0; j < Lines[i].size(); j++)
	{
	    PointInWorld1.x = Lines[i][j][0];
	    PointInWorld1.y = Lines[i][j][1];
	    PointInWorld2.x = Lines[i][j][2];
	    PointInWorld2.y = Lines[i][j][3];
	    
	    PointCZ2Pic(PointInWorld1, PointInPic1, R_cz_pic, Height);
	    PointCZ2Pic(PointInWorld2, PointInPic2, R_cz_pic, Height);
	    
	    line(ImageSource, PointInPic1, PointInPic2, Scalar(0, 0, 255));
	    
	    MiddlePoint.x = (PointInPic1.x + PointInPic2.x) / 2;
	    MiddlePoint.y = (PointInPic1.y + PointInPic2.y) / 2;
	    
	    stringstream os;
	    os << i;
	    Words = os.str();
	    putText(ImageSource, Words, MiddlePoint, CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 0));
	}
    }
    if(Flag == 0)
    {
	imshow("0度距离分类测试", ImageSource);
    }
    else if(Flag == 90)
    {
	imshow("90度距离分类测试", ImageSource);
    }
}
void BoundaryTest(const Vec4f &Line, const Mat& R_cz_pic, const float &Height, const Mat &Image, int LineFlag, int FunctionFlag)
{
    Mat ImageSource;
    Image.copyTo(ImageSource);
    Point2f PointInCZ1, PointInCZ2, PointInPic1, PointInPic2, MiddlePoint;
    PointInCZ1.x = Line[0];
    PointInCZ1.y = Line[1];
    PointInCZ2.x = Line[2];
    PointInCZ2.y = Line[3];
    
    PointCZ2Pic(PointInCZ1, PointInPic1, R_cz_pic, Height);
    PointCZ2Pic(PointInCZ2, PointInPic2, R_cz_pic, Height);
    
    MiddlePoint.x = (PointInPic1.x + PointInPic2.x) / 2;
    MiddlePoint.y = (PointInPic1.y + PointInPic2.y) / 2;
    
    line(ImageSource, PointInPic1, PointInPic2, Scalar(0, 0, 255));
    
    float Angle;
    Angle = atan2(PointInCZ1.y - PointInCZ2.y, PointInCZ1.x - PointInCZ2.x);
    if(Angle < 0)
    {
	Angle += CV_PI;
    }
    Angle = Angle / CV_PI * 180;
    
    string Words;
    stringstream os;
    os << Angle;
    Words = os.str();
    putText(ImageSource, Words, MiddlePoint, CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 0));
    
    string Words1;
    stringstream os1;
    os1 << FunctionFlag;
    Words1 = os1.str();
    putText(ImageSource, Words1, Point(320, 240), CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 255));
    
    if(FunctionFlag != 0)
    {
	if(LineFlag == 1)
	{
	    imshow("边界1", ImageSource);
	}
	if(LineFlag == 2)
	{
	    imshow("边界2", ImageSource);
	}
	if(LineFlag == 3)
	{
	    imshow("边界3", ImageSource);
	}
	if(LineFlag == 4)
	{
	    imshow("边界4", ImageSource);
	}
    }
    else
    {
	if(LineFlag == 2)
	{
	    imshow("红1", ImageSource);
	}
	if(LineFlag == 1)
	{
	    imshow("绿", ImageSource);
	}
	if(LineFlag == 3)
	{
	    imshow("红", ImageSource);
	}
    }
}
void DistanceTest(const vector<Vec4f> &Lines, const Mat& R_cz_pic, const float &Height, const Mat &Image, int Flag)
{
    Mat ImageSource;
    Image.copyTo(ImageSource);  
    
    Point2f PointInCZ1, PointInCZ2, PointInPic1, PointInPic2, MiddlePoint;
    for(int i = 0; i < Lines.size(); i++)
    {
	PointInCZ1.x = Lines[i][0];
	PointInCZ1.y = Lines[i][1];
	PointInCZ2.x = Lines[i][2];
	PointInCZ2.y = Lines[i][3];
	
	PointCZ2Pic(PointInCZ1, PointInPic1, R_cz_pic, Height);
	PointCZ2Pic(PointInCZ2, PointInPic2, R_cz_pic, Height);
	
	line(ImageSource, PointInPic1, PointInPic2, Scalar(0, 0, 255));
	
	MiddlePoint.x = (PointInPic1.x + PointInPic2.x) / 2;
	MiddlePoint.y = (PointInPic1.y + PointInPic2.y) / 2;
	for(int j = i + 1; j < Lines.size(); j++ )
	{
	    float XMiddle = (Lines[j][0] + Lines[j][2]) / 2;
	    float YMiddle = (Lines[j][1] + Lines[j][3]) / 2;
	    float Distance = DistanceP2L(Lines[i], Point2f(XMiddle, YMiddle));
	    if(Flag == 0)
	    {
		cout << "Distance between Line0_"<<i << " and Line0_" << j << " = " << Distance << endl;
	    }
	    if(Flag == 90)
	    {
		cout << "Distance between Line90_"<<i << " and Line90_" << j << " = " << Distance << endl;
	    }
	}
	stringstream os;
	string Words;
	os << i;
	Words = os.str();
	putText(ImageSource, Words, MiddlePoint, CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 0));
    }
    if(Flag == 0)
    {
	imshow("0度直线距离", ImageSource);
    }
    if(Flag == 90)
    {
	imshow("90度直线距离", ImageSource);
    }
}
void LineInSideCorrectTest(const vector<Vec4f> &Lines, const int & FirstLineNum, const float &Error, const int &LineFlag, const Mat &R_cz_pic, const float &Height, const Mat &Image, int Corrected)
{
    Mat ImageSource;
    Image.copyTo(ImageSource);
    
    Point2f PointInCZ1, PointInCZ2, PointInPic1, PointInPic2, MiddlePoint;
    for(int i = 0; i < Lines.size(); i++)
    {
	
	PointInCZ1.x = Lines[i][0];
	PointInCZ1.y = Lines[i][1];
	PointInCZ2.x = Lines[i][2];
	PointInCZ2.y = Lines[i][3];
	
	PointCZ2Pic(PointInCZ1, PointInPic1, R_cz_pic, Height);
	PointCZ2Pic(PointInCZ2, PointInPic2, R_cz_pic, Height);
	
	MiddlePoint.x = (PointInPic1.x + PointInPic2.x) / 2;
	MiddlePoint.y = (PointInPic1.y + PointInPic2.y) / 2;
	
	line(ImageSource, PointInPic1, PointInPic2, Scalar(0, 0, 255));
	stringstream os;
	string Words;
	os << i +FirstLineNum;
	Words = os.str();
	putText(ImageSource, Words, MiddlePoint, CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 255));
    }
    stringstream os;
    string Words;
    os << Error;
    Words = os.str();
    putText(ImageSource, Words, Point(320, 240), CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 255));
    
    if(Corrected == 0)
    {
	
	string Words1 = "NO";

	putText(ImageSource, Words1, Point(160, 120), CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 255, 255));
    }
    
    if(LineFlag == 0)
    {
	imshow("0度内部直线校正", ImageSource);
    }
    if(LineFlag == 90)
    {
	imshow("90度内部直线校正", ImageSource);
    }
}
void AllLineInsideTest(const vector<Vec4f> &Lines, const int &Flag, const Mat &Image, const Mat &R_cz_pic, const float &Height)
{
    Mat ImageSource;
    Image.copyTo(ImageSource);
    
    Point2f PointInCZ1, PointInCZ2, PointInPic1, PointInPic2, MiddlePoint;
    for(int i = 0; i < Lines.size(); i++)
    {
	PointInCZ1.x = Lines[i][0];
	PointInCZ1.y = Lines[i][1];
	PointInCZ2.x = Lines[i][2];
	PointInCZ2.y = Lines[i][3];
	
	PointCZ2Pic(PointInCZ1, PointInPic1, R_cz_pic, Height);
	PointCZ2Pic(PointInCZ2, PointInPic2, R_cz_pic, Height);
	
	MiddlePoint.x = (PointInPic1.x + PointInPic2.x) / 2;
	MiddlePoint.y = (PointInPic1.y + PointInPic2.y) / 2;
	
	line(ImageSource, PointInPic1, PointInPic2, Scalar(0, 0, 255));
	stringstream os;
	string Words;
	os << i;
	Words = os.str();
	putText(ImageSource, Words, MiddlePoint, CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 255));
    }
    if(Flag == 0)
    {
	imshow("全部0度内部直线", ImageSource);
    }
    if(Flag == 90)
    {
	imshow("全部90度内部直线", ImageSource);
    }
}

void ResultTest(const Mat &Image, const Mat& Position)
{
    Mat ImageSource;
    Image.copyTo(ImageSource);
    stringstream os;
    string Words;
    os << "(" << Position.at<float>(0, 0) << ", " << Position.at<float>(1, 0) << ")";
    Words = os.str();
    putText(ImageSource, Words, Point(320, 240), CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 255));
    imshow("结果测试", ImageSource);
}