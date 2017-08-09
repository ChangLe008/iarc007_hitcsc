#include <iostream>
#include <iarc/pilot/leo_math.h>
#include "Eigen/Dense"
#include "Eigen/Eigen"
#include <cmath>
#include <random>
#include <iarc/path.h>
using namespace Eigen;


class rrt
{
    private: 
        int map[30][40];
        int cost[30][40];
        int parent_x[30][40];
        int parent_y[30][40];
        int is_map[30][40];
        int level[30][40];

        Vector2f start_point;
        Vector2f end_point;
        Vector2f sample_point;
        float radius = 1.1;
        int obstacle_num = 0;
        float min_dis = 2.5; // valid distance between two points
        int seeds = 800;




        float dist(float x1,float y1,float x2,float y2);
        float dist(Vector2f p1,Vector2f p2);
        bool collision(Vector2f p1,Vector2f p2,Vector2f obstacle);
        Vector2f resample(Vector2f st,Vector2f end);
        Vector2f resample();
        bool steering(int x1,int x2,int x3,int y1,int y2,int y3);
    public:     
        rrt();
        void open(char* para);
        void _init_();
        void setparam(pid_control new_pid);
        void clean();
        //velocity_command out(float dx,float dy,float dz,float dyaw);
        float out(float ideal,float now,float acc_flag);
        iarc::path output(float target_x,float target_y,float obs_x,float obs_y);
        iarc::path output(float target_x,float target_y,float obs_x[],float obs_y[],int num);
        void fill(float obs_x,float obs_y,int r);
};