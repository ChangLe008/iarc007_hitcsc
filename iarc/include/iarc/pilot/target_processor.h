#include "iarc/pilot/leo_math.h"
class target_processor
{
    private: 
        target_info info = {.x=0., .y=0., .vx=0., .vy=0., .dir=0.,.num=0};
        target_info info_last = {.x=0., .y=0., .vx=0., .vy=0., .dir=0.,.num=0};
        float impulse = 1.0;
        float saturation = 0.5;
    public:     
        target_processor();
        void _init_();
        void setparam(pid_control new_pid);
        void clean();
        float dir_filter(float dir_now);
        float vx_filter(float vx_now);
        float vy_filter(float vy_now);
        target_info out(target_info ori);
};