#include "iarc/pilot/target_processor.h"

target_info target_processor::out(target_info ori)
{
    info_last = info;
    info = ori;
    info.vx = (info.vx-info_last.vx>impulse)?info_last.vx:(info.vx-info_last.vx<-1*impulse)?info_last.vx:info.vx;
    info.vy = (info.vy-info_last.vy>impulse)?info_last.vy:(info.vy-info_last.vy<-1*impulse)?info_last.vy:info.vy;
    info.vx = (info.vx-info_last.vx>saturation)?info_last.vx+0.5:(info.vx-info_last.vx<-1*saturation)?info_last.vx-0.5:info.vx;
    info.vy = (info.vy-info_last.vy>saturation)?info_last.vy+0.5:(info.vy-info_last.vy<-1*saturation)?info_last.vy-0.5:info.vy;
    info.vx = limiter(info.vx,0.75,-0.75);
    info.vy = limiter(info.vy,0.75,-0.75);
    info.x  = -1*info.x;
    info.y  = -1*info.y;
    return info;
}
target_processor::target_processor()
{
    ;
}
void target_processor::clean()
{
    info = {.x=0., .y=0., .vx=0., .vy=0., .dir=0.,.num=0};
    info_last = {.x=0., .y=0., .vx=0., .vy=0., .dir=0.,.num=0};
}
