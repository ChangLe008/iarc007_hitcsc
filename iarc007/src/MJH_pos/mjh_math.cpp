#include "iarc007/mjh_pos/mjh_math.h"

//Q=w+x*i+y*j+z*k
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