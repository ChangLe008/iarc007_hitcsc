#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <sys/time.h>

#ifndef PI 
#define PI 3.1416
#endif

#ifndef pid_struct
#define pid_struct
typedef struct
{
	float kp;
	float ki;
	float kd;
	float history;                  //last error
	float out;                      // output
	float integral;                 // Integral 
    float integral_upper_limit;     // Integral limit
    float integral_lower_limit;     // 
    float output_upper_limit;       // output limit 
    float output_lower_limit;       //
	float acc_limit;			// acc limit 
}pid_control;

typedef struct
{
	float x;
	float y;
	float vx;
	float vy;
	float dir;
	int num;
}target_info;

typedef struct
{
	float x1;
	float y1;
	float x2;
	float y2;
	float x3;
	float y3;
	float x4;
	float y4;
	int num;
}obstacle_info;


#endif

void Quaternion_To_Euler(CvMat *q_att, CvMat *att);

void Euler_To_Matrix(float roll, float pitch, float yaw, CvMat *R);

double tic();

float limiter(float data,float upperlimit,float lowerlimit);

float Low_Pass(float h_sonar);


