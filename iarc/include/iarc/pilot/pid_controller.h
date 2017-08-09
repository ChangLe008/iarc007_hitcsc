#include <stdlib.h>
#include <stdio.h>
#include <iarc/pilot/leo_math.h>
#include <iarc/pilot/control.h>
#include <string>


class pid_ctrl
{
    private: 
        //FILE *para;
        pid_control pid={0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.}; 
        float saturation(float data,float upper_limit,float lower_limit);
        float acc_satu(float data,float acc);
        _velocity_command output;
        
    public:     
        pid_ctrl();
        void open(char* para);
        void _init_();
        void setparam(pid_control new_pid);
        void clean();
        //velocity_command out(float dx,float dy,float dz,float dyaw);
        float out(float ideal,float now,float acc_flag);
};