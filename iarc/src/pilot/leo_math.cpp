#include "iarc/pilot/leo_math.h"



void Quaternion_To_Euler(CvMat *q_att, CvMat *att)
{
		double r11,r12,r21,r31,r32,r1,r2,r3;
		double q[4] = { cvmGet(q_att,0,0),  cvmGet(q_att,1,0), cvmGet(q_att,2,0), cvmGet(q_att,3,0)};
    	r11 = 2.0f *(q[1] * q[2] + q[0] * q[3]);
    	r12 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2]  - q[3] * q[3] ;
    	r21 = -2.0f * (q[1] * q[3] - q[0] * q[2]);
    	r31 = 2.0f *( q[2] * q[3] + q[0]  * q[1]);
    	r32 = q[0] * q[0] - q[1] * q[1]  - q[2] * q[2] + q[3] * q[3] ;
    	double yaw= atan2( r11, r12 );
    	double pitch = asin( r21 );
    	double roll = atan2( r31, r32 );
    	cvmSet(att, 0, 0, roll);
    	cvmSet(att, 1, 0, pitch);
    	cvmSet(att, 2, 0, yaw);
}

double tic()
{
	struct timeval t;
	gettimeofday(&t,NULL);
	return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

float limiter(float data,float upperlimit,float lowerlimit)
{

	data = data>upperlimit?upperlimit:data;
	data = data<lowerlimit?lowerlimit:data;
	return data;
}

void Euler_To_Matrix(float roll, float pitch, float yaw, CvMat *R)
{
	float cp = cosf(pitch);
	float sp = sinf(pitch);
	float sr = sinf(roll);
	float cr = cosf(roll);
	float sy = sinf(yaw);
	float cy = cosf(yaw);
	
	cvmSet(R,0, 0,cp * cy);
	cvmSet(R,0, 1, ((sr * sp * cy) - (cr * sy)));
	cvmSet(R,0, 2, ((cr * sp * cy) + (sr * sy)));
	cvmSet(R,1, 0, (cp * sy));
	cvmSet(R,1,1, ((sr * sp * sy) + (cr * cy)));
	cvmSet(R,1, 2, ( (cr * sp * sy) - (sr * cy)));
	cvmSet(R,2, 0,  -sp);
	cvmSet(R,2, 1,  sr * cp);
	cvmSet(R,2, 2,  cr * cp);
//printf("%f	%f	%f\n",cvmGet(R,0,0),cvmGet(R,0,1),cvmGet(R,0,2));
}

extern float last_height_fusion[3];
extern float last_height_original[4];
extern int height_count_original;
extern int height_count_fusion;
extern bool low_pass_flag1;
extern bool low_pass_flag2;
extern float  h_sonar;

float last_height_fusion[3]={0.0f};
float last_height_original[4]={0.0f};
int height_count_original=0;
int height_count_fusion=0;
bool low_pass_flag1=true;
bool low_pass_flag2=true;

float Low_Pass(float h_sonar)
{	

	float a1=1,a2= -2.4803,a3=2.0872,a4=-0.5930;
	float b1=0.0017,b2= 0.0052,b3= 0.0052,b4=0.0017;

	if(height_count_fusion==3)
	{
		low_pass_flag2=false;
	}

	//////////////////////////////////////////////////////////////////////////////实现归零的功能
	if(height_count_original==4)
	{
		height_count_original=0;
	}
	if(height_count_fusion==3)
	{
		height_count_fusion=0;
	}
	//////////////////////////////////////////////////////////////////////////////用来判断是不是前两次为了第一次滤波准备

	 //printf("%f %f %f %f %f %f %f %f\n",a1,a2,a3,a4,b1,b2,b3,b4);
	last_height_original[height_count_original]=h_sonar;

	if(height_count_original==2)
	{
		low_pass_flag1=false;
	}
	//////////////////////////////////////////////////////////////////////////////当标志位为可以滤波的标志的时候开始初步第一次滤波

	if((low_pass_flag1==0))
	{
		if(height_count_original==0)
		{
			if(last_height_original[0]<0.01)
			{
				last_height_original[0]=last_height_original[3];
			}
		}
		else if(height_count_original==1)
		{
			if(last_height_original[1]<0.01)
			{
				last_height_original[1]=last_height_original[0];
			}
		}
		else if(height_count_original==2)
		{
			if(last_height_original[2]<0.01)
			{
				last_height_original[2]=last_height_original[1];
			}
		}
		else if(height_count_original==3)
		{
			if(last_height_original[3]<0.01)
			{
				last_height_original[3]=last_height_original[2];
			}
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////利用标志位判断是不是第一次进入第二次的滤波

	if(low_pass_flag2)
	{
		//printf("bbb---if-----bb\n");
		if(height_count_fusion==0)
		{
			last_height_fusion[0]=b1*last_height_original[0];
		}
		if(height_count_fusion==1)
		{
			last_height_fusion[1]=b1*last_height_original[1]+b2*last_height_original[0]-a2*last_height_fusion[0];
		}
		if(height_count_fusion==2)
		{
			last_height_fusion[2]=b1*last_height_original[2]+b2*last_height_original[1]+b3*last_height_original[0]-a2*last_height_fusion[1]-a3*last_height_fusion[0];
		}
	}
	else
	{
		 //printf("aaaa---else ---\n");
		if(height_count_fusion==0&&height_count_original==0)
		{
			last_height_fusion[0]=b1*last_height_original[0]+b2*last_height_original[3]+b3*last_height_original[2]+b4*last_height_original[1]-a2*last_height_fusion[2]-a3*last_height_fusion[1]-a4*last_height_fusion[0];
		}
		if(height_count_fusion==0&&height_count_original==1)
		{
			last_height_fusion[0]=b1*last_height_original[1]+b2*last_height_original[0]+b3*last_height_original[3]+b4*last_height_original[2]-a2*last_height_fusion[2]-a3*last_height_fusion[1]-a4*last_height_fusion[0];
		}
		if(height_count_fusion==0&&height_count_original==2)
		{
			last_height_fusion[0]=b1*last_height_original[2]+b2*last_height_original[1]+b3*last_height_original[0]+b4*last_height_original[3]-a2*last_height_fusion[2]-a3*last_height_fusion[1]-a4*last_height_fusion[0];
		}
		if(height_count_fusion==0&&height_count_original==3)
		{
			last_height_fusion[0]=b1*last_height_original[3]+b2*last_height_original[2]+b3*last_height_original[1]+b4*last_height_original[0]-a2*last_height_fusion[2]-a3*last_height_fusion[1]-a4*last_height_fusion[0];
		}

		if(height_count_fusion==1&&height_count_original==0)
		{
			last_height_fusion[1]=b1*last_height_original[0]+b2*last_height_original[3]+b3*last_height_original[2]+b4*last_height_original[1]-a2*last_height_fusion[0]-a3*last_height_fusion[2]-a4*last_height_fusion[1];
		}
		if(height_count_fusion==1&&height_count_original==1)
		{
			last_height_fusion[1]=b1*last_height_original[1]+b2*last_height_original[0]+b3*last_height_original[3]+b4*last_height_original[2]-a2*last_height_fusion[0]-a3*last_height_fusion[2]-a4*last_height_fusion[1];
		}
		if(height_count_fusion==1&&height_count_original==2)
		{
			last_height_fusion[1]=b1*last_height_original[2]+b2*last_height_original[1]+b3*last_height_original[0]+b4*last_height_original[3]-a2*last_height_fusion[0]-a3*last_height_fusion[2]-a4*last_height_fusion[1];
		}
		if(height_count_fusion==1&&height_count_original==3)
		{
			last_height_fusion[1]=b1*last_height_original[3]+b2*last_height_original[2]+b3*last_height_original[1]+b4*last_height_original[0]-a2*last_height_fusion[0]-a3*last_height_fusion[2]-a4*last_height_fusion[1];
		}


		if(height_count_fusion==2&&height_count_original==0)
		{
			last_height_fusion[2]=b1*last_height_original[0]+b2*last_height_original[3]+b3*last_height_original[2]+b4*last_height_original[1]-a2*last_height_fusion[1]-a3*last_height_fusion[0]-a4*last_height_fusion[2];
		}
		if(height_count_fusion==2&&height_count_original==1)
		{
			last_height_fusion[2]=b1*last_height_original[1]+b2*last_height_original[0]+b3*last_height_original[3]+b4*last_height_original[2]-a2*last_height_fusion[1]-a3*last_height_fusion[0]-a4*last_height_fusion[2];
		}
		if(height_count_fusion==2&&height_count_original==2)
		{
			last_height_fusion[2]=b1*last_height_original[2]+b2*last_height_original[1]+b3*last_height_original[0]+b4*last_height_original[3]-a2*last_height_fusion[1]-a3*last_height_fusion[0]-a4*last_height_fusion[2];
		}
		if(height_count_fusion==2&&height_count_original==3)
		{
			last_height_fusion[2]=b1*last_height_original[3]+b2*last_height_original[2]+b3*last_height_original[1]+b4*last_height_original[0]-a2*last_height_fusion[1]-a3*last_height_fusion[0]-a4*last_height_fusion[2];
		}
	}
	//printf("original3    %d     fusion2    %d    height    %f\n",height_count_original,height_count_fusion,last_height_fusion[height_count_fusion]);
	float y=last_height_fusion[height_count_fusion];
	height_count_original=height_count_original+1;
	height_count_fusion=height_count_fusion+1;

	h_sonar = y;

	return y;
}


