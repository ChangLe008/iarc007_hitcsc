#include "iarc/navigation/math.h"

void Quaternion2Euler(Mat Quaternion, Mat &Eurler)
{
	float R11,R12,R21,R31,R32,R1,R2,R3;
	float Q[4];
	Q[0] = Quaternion.at<float>(0, 0);
	Q[1] = Quaternion.at<float>(1, 0);
	Q[2] = Quaternion.at<float>(2, 0);
	Q[3] = Quaternion.at<float>(3, 0);
    	R11 = 2.0f *(Q[1] * Q[2] + Q[0] * Q[3]);
    	R12 = Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2]  - Q[3] * Q[3] ;
    	R21 = -2.0f * (Q[1] * Q[3] - Q[0] * Q[2]);
    	R31 = 2.0f *( Q[2] * Q[3] + Q[0]  * Q[1]);
    	R32 = Q[0] * Q[0] - Q[1] * Q[1]  - Q[2] * Q[2] + Q[3] * Q[3] ;
    	float Yaw= atan2( R11, R12 );
    	float Pitch = asin( R21 );
    	float Roll = atan2( R31, R32 );
	Eurler.at<float>(0, 0) = Roll;
	Eurler.at<float>(1, 0) = Pitch;
	Eurler.at<float>(2, 0) = Yaw;
}

void Euler2Matrix(Mat Eurler, Mat &R)
{
	float Roll = Eurler.at<float>(0, 0);
	float Pitch = Eurler.at<float>(1, 0);
	float Yaw = Eurler.at<float>(2, 0);
	
	float CP = cosf(Pitch);
	float SP = sinf(Pitch);
	float SR = sinf(Roll);
	float CR= cosf(Roll);
	float SY = sinf(Yaw);
	float CY = cosf(Yaw);
	
	R.at<float>(0, 0) = CP * CY;
	R.at<float>(0, 1) = SR * SP * CY - CR* SY;
	R.at<float>(0, 2) = CR* SP * CY + SR * SY;
	R.at<float>(1, 0) = CP * SY;
	R.at<float>(1, 1) = SR * SP * SY + CR * CY;
	R.at<float>(1, 2) =  CR* SP * SY - SR * CY;
	R.at<float>(2, 0) =  -SP;
	R.at<float>(2, 1) =  SR * CP;
	R.at<float>(2, 2) =  CR* CP;
}

void  Quaternion2Matrix(Mat Quaternion, Mat &R)
{
    Mat Eurler = Mat(3, 1, CV_32FC1, Scalar(0));
    Quaternion2Euler(Quaternion, Eurler);
    Euler2Matrix(Eurler, R);
}
void Matrix2Quernion(Mat RM, Mat &Quaternion)
{
    float R[3][3];
    
    for(int i = 0; i < 3; i++)
    {
	for(int j = 0; j < 3; j++)
	{
	    R[i][j] = RM.at<float>(i, j);
	}
    }
    
    float Q0Absolute = 0.5 * sqrt(1 + R[0][0] + R[1][1] + R[2][2]);
    float Q1Absolute = 0.5 * sqrt(1 + R[0][0] - R[1][1] - R[2][2]);
    float Q2Absolute = 0.5 * sqrt(1 - R[0][0] + R[1][1] - R[2][2]);
    float Q3Absolute = 0.5 * sqrt(1 - R[0][0] - R[1][1] + R[2][2]);
    
    float Q0 = Q0Absolute;
    float Q1 = Q1Absolute * FindSign(R[1][2] - R[2][1]);
    float Q2 = Q2Absolute * FindSign(R[2][0] - R[0][2]);
    float Q3 = Q3Absolute * FindSign(R[0][1] - R[1][0]);
    
    Quaternion.at<float>(0, 0) = Q0;
    Quaternion.at<float>(1, 0) = -Q1;
    Quaternion.at<float>(2, 0) = -Q2;
    Quaternion.at<float>(3, 0) = -Q3; 
}
float FindSign(float a)
{
    if(a < 0)
    {
	return -1.0;
    }
    else if(a >= 0)
    {
	return 1.0;
    }
}

void  PointPic2CZ(const Point2f &PointPic, Point2f& PointCZ, const Mat &R_cz_pic, const float &Height)
{
    Mat PointInPic(3, 1, CV_32FC1);
    PointInPic.at<float>(0, 0) = PointPic.x;
    PointInPic.at<float>(1, 0) = PointPic.y;
    PointInPic.at<float>(2, 0) = 1;
    
    Mat PointInCZ = Mat(3, 1, CV_32FC1);
    PointInCZ = R_cz_pic * PointInPic;
    
    float Scale = PointInCZ.at<float>(2, 0) / Height;
    PointCZ.x = PointInCZ.at<float>(0, 0) / Scale;
    PointCZ.y = PointInCZ.at<float>(1, 0) / Scale;
}
void  PointCZ2Pic(const Point2f &PointCZ, Point2f& PointPic, const Mat &R_cz_pic, const float &Height)
{
    Mat PointInCZ(3, 1, CV_32FC1);
    PointInCZ.at<float>(0, 0) = PointCZ.x;
    PointInCZ.at<float>(1, 0) = PointCZ.y;
    PointInCZ.at<float>(2, 0) = Height;
    
    Mat PointInPic(3, 1, CV_32FC1);
    Mat R_pic_cz = R_cz_pic.inv();
    
    PointInPic = R_pic_cz * PointInCZ;
    
    float Scale = PointInPic.at<float>(2, 0);
    PointPic.x = PointInPic.at<float>(0, 0) / Scale;
    PointPic.y = PointInPic.at<float>(1, 0) / Scale;
}
float DistanceP2L(const Vec4f &Line, const Point2f &P)
{
    float X0 = Line[0];
    float Y0 = Line[1];
    float X1 = Line[2];
    float Y1 = Line[3];
    
    float A = Y1 - Y0;
    float B = -(X1 - X0);
    float C = (X1 - X0) * Y0 - X0 * (Y1 - Y0);
    float Den;
    
    Den = A * A + B * B;
    Den = sqrt(Den);
    float Distance;
    Distance = fabs(A * P.x + B * P.y + C);
    Distance = Distance / Den;
    
    return Distance;
}
bool Comp0(const vector<Vec4f> &vector1, const vector<Vec4f> &vector2)
{
    float Y1 = 0;
    for(int i = 0; i < vector1.size(); i++)
    {
	Y1 += (vector1[i][1] + vector1[i][3]) / 2;
    }
    
    Y1 = Y1 / vector1.size();
    
    float Y2 = 0;
    for(int i = 0; i < vector2.size(); i++)
    {
	Y2 += (vector2[i][1] + vector2[i][3]) / 2;
    }
    Y2 = Y2 / vector2.size();
    return Y1 < Y2;
}
bool Comp90(const vector<Vec4f> &vector1, const vector<Vec4f> &vector2)
{
    float X1 = 0;
    for(int i = 0; i < vector1.size(); i++)
    {
	X1 += (vector1[i][0] + vector1[i][2]) / 2;
    }
    X1 = X1 / vector1.size();
    
    float X2 = 0;
    for(int i = 0; i < vector2.size(); i++)
    {
	X2 += (vector2[i][0] + vector2[i][2]) / 2;
    }
    
    X2 = X2 / vector2.size();
    
    return X1 < X2;
}

void LineFit90(const vector<Point2f>& P, Vec4f& Line)
{
    Vec4f L;
    fitLine(P, L, CV_DIST_L12, 0, 0.01, 0.01);
    float YMin = 1000;
    float YMax = -1000;
    for(int i = 0; i < P.size(); i++)
    {
	YMin = YMin > P[i].y ? P[i].y : YMin;
	YMax = YMax < P[i].y ? P[i].y : YMax;
    }
    float Vx = L[0];
    float Vy = L[1];
    float X0 = L[2];
    float Y0 = L[3];
    float XMin;
    float XMax;
    
    XMin = X0 + (YMax - Y0) * Vx / Vy;
    XMax = X0 + (YMin - Y0) * Vx / Vy;
    
    Line[0] = XMin;
    Line[1] = YMin;
    Line[2] = XMax;
    Line[3] = YMax;
}
void LineFit0(const vector<Point2f>& P, Vec4f& Line)
{
    Vec4f L;
    fitLine(P, L, CV_DIST_L12, 0, 0.01, 0.01);
    float XMin = 1000;
    float XMax = -1000;
    for(int i = 0; i < P.size(); i++)
    {
	XMin = XMin > P[i].x ? P[i].x : XMin;
	XMax = XMax < P[i].x ? P[i].x : XMax;
    }
    float Vx = L[0];
    float Vy = L[1];
    float X0 = L[2];
    float Y0 = L[3];
    float YMin;
    float YMax;
    
    YMin = Y0 + (XMin - X0) * Vy / Vx;
    YMax = Y0 + (XMax - X0) * Vy / Vx;
    
    Line[0] = XMin;
    Line[1] = YMin;
    Line[2] = XMax;
    Line[3] = YMax;
}
