#include <iarc/pilot/pid_controller.h>


pid_ctrl::pid_ctrl()
{
	;
}
void pid_ctrl::open(char* para)
{
    FILE *param=fopen(para,"r"); //need to be selected again
	if(NULL==param)
	{
		printf("not find pid_para!\n");
	}
	fscanf(param,"%f",&this->pid.kp);		//x
	fscanf(param,"%f",&this->pid.ki);
	fscanf(param,"%f",&this->pid.kd);
	fscanf(param,"%f",&this->pid.integral_upper_limit);
	fscanf(param,"%f",&this->pid.integral_lower_limit);
	fscanf(param,"%f",&this->pid.output_upper_limit);
	fscanf(param,"%f",&this->pid.output_lower_limit);
	fscanf(param,"%f",&this->pid.acc_limit);

	fclose(param);
}

float pid_ctrl::saturation(float data,float top_limit,float lower_limit)
{
  return data>top_limit?top_limit:data<lower_limit?lower_limit:data;
}
float pid_ctrl::acc_satu(float data,float acc)
{
	return data>(pid.history+acc)?(pid.history+acc):data<(pid.history-acc)?(pid.history-acc):data;
}

void pid_ctrl::setparam(pid_control new_pid )
{
	this->pid.kp = new_pid.kp;
	this->pid.ki = new_pid.ki;
	this->pid.kd = new_pid.kd;
	this->pid.integral_upper_limit = new_pid.integral_upper_limit;
	this->pid.integral_lower_limit = new_pid.integral_lower_limit;
	this->pid.output_upper_limit = new_pid.output_upper_limit;
	this->pid.output_lower_limit = new_pid.output_lower_limit;
	this->pid.acc_limit = new_pid.acc_limit;
}


float pid_ctrl::out(float ideal,float now,float acc_flag)
{
	float temp		=	ideal-now;
	pid.integral	+=	ideal-now;
	pid.integral	=	saturation(pid.integral,pid.integral_upper_limit,pid.integral_lower_limit);
	pid.out		=	pid.kp*temp+pid.ki*pid.integral+pid.kd*(temp-pid.history);
	pid.history	=	temp;
	if(acc_flag)
	{
		pid.out 	=	acc_satu(pid.out,pid.acc_limit);
	}		
	pid.out     =   saturation(pid.out,pid.output_upper_limit,pid.output_lower_limit);
	return pid.out;
}

void pid_ctrl::clean()
{
	pid.integral	= 0;
	pid.history		= 0;
}

/*
velocity_command pid_ctrl::out(float dx,float dy,float dz,float dyaw)
{
	output.vx=pid_x.kp*dx+pid_x.ki;
	return output;
}*/