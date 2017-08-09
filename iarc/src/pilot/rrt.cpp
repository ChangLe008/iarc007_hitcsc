#include "iarc/pilot/rrt.h"
#include <iarc/obstacle.h>
#include <random>
#include <cmath>
using namespace std;
#define random(x) (rand()%x)


default_random_engine grow((unsigned int)(time(0)*100.));  
default_random_engine gcol((unsigned int)(time(0)*1000.));  
uniform_int_distribution<int> row(0,30);
uniform_int_distribution<int> col(0,40);
auto r_row= bind(row,grow);
auto r_col= bind(col,gcol);

iarc::path result;


rrt::rrt()
{
    start_point <<  15,
                    20;
    MatrixXf cost=MatrixXf::Zero(30,40);
}

float rrt::dist(Vector2f p1,Vector2f p2)
{
    return pow((p1-p2).dot(p1-p2),0.5);
}

float rrt::dist(float x1,float y1,float x2,float y2)
{
    return pow((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2),0.5);

}

bool rrt::collision(Vector2f p1,Vector2f p2,Vector2f o)
{
    if( dist(p1,o)<=radius || dist(p2,o)<=radius )
        return 1;
    else
    {
        float temp = abs(pow(1-radius*radius/pow(dist(p1,o),2),0.5));
        if(temp<((o-p1).dot(p2-p1)/((o-p1).norm()*(p2-p1).norm())))
            return 1;
        else
            return 0;
    }
}


Vector2f rrt::resample(Vector2f st,Vector2f end)
{
    Vector2f tempr;
    // temp << r_col(),
    //         r_row();
    
    tempr << (int)(29.0*rand()/(RAND_MAX+1.0)),
            (int)(39.0*rand()/(RAND_MAX+1.0));
    int xx = tempr(0);
    int yy = tempr(1);
    while(tempr.dot(end)<0. || is_map[xx][yy]==2 || is_map[xx][yy]==1)
    {
        tempr << (int)(29.0*rand()/(RAND_MAX+1.0)),
            (int)(39.0*rand()/(RAND_MAX+1.0));
        xx = tempr(0);
        yy = tempr(1);
    }
    //cout << tempr(0)<<"\t" << tempr(1)<< endl;
    // cout << "jkl";
    return tempr;

}

Vector2f rrt::resample()
{
    Vector2f tempr;
    tempr << (int)(29.0*rand()/(RAND_MAX+1.0)),
            (int)(39.0*rand()/(RAND_MAX+1.0));
    int xx = tempr(0);
    int yy = tempr(1);
    while(/*tempr.dot(end)<0. ||*/ is_map[xx][yy]==2 || is_map[xx][yy]==1)
    {
        tempr << (int)(29.0*rand()/(RAND_MAX+1.0)),
            (int)(39.0*rand()/(RAND_MAX+1.0));
    }
    //cout << tempr(0)<<"\t" << tempr(1)<< endl;
    // cout << "jkl";
    return tempr;

}

bool rrt::steering(int x1,int x2,int x3,int y1,int y2,int y3)
{
    return 1;
}

void rrt::fill(float obs_x,float obs_y,int r)
{
    int ox = (int)(obs_x/0.1)+20;
    int oy = (int)(obs_y/0.1)+15;
    for(int i=0;i<30;i++)
        {
            for(int j=0;j<40;j++)
            {
                is_map[i][j]=0;
                level[i][j]=0;
                map[i][j]=0;
                cost[i][j]=0;
                parent_x[i][j]=0;
                parent_y[i][j]=0;
        
                if(dist(i,j,oy,ox)<=r)
                    {is_map[i][j]=2;}
                
                // if(is_map(i,j)==1)
                
            }
        }
        is_map[15][20] = 1;

}

iarc::path rrt::output(float target_x,float target_y,float obs_x,float obs_y)
{
    // obs = obstacle;
    // int obs_num = obs.num;
    end_point << (int)(target_y/0.1)+15,
                (int)(target_x/0.1)+20;

    // 1 route
    // 2 obstacle 
    // 0 not
    //srand((int)(time(0)*1000.)); 
    //if(end_point(0)>=40 )
    is_map[15][20] = 1;
    level[15][20] = 1;
    fill(obs_x,obs_y,7);
    //cout << "ec"<<endl;
    for(int seed=0;seed<=seeds;seed++)
    {
        // sample_point = resample(start_point,end_point);
        // sample_point << (int)(29.0*rand()/(RAND_MAX+1.0)),
            // (int)(39.0*rand()/(RAND_MAX+1.0));
        sample_point = resample(start_point,end_point);
        // cout <<"start" << end_point(0)<< "\t"<<end_point(1)<<"\t";
        int spx =(int)sample_point(0);
        int spy =(int)sample_point(1);
        // cout << sample_point(0) << "\t "<< sample_point(1)<<"sp"<<endl;
        bool sample_valid = 0;
        if(is_map[spx][spy]==2)
        {
            continue;
        }
        for(int i=0;i<29;i++)
        {
            for(int j=0;j<39;j++)
            {
                if(is_map[i][j]==1)
                {
                    Vector2f temp;
                    temp<< i,
                        j;
                        // cout <<"i"<< i<<"j"<<j<<endl;
                    
                    if( dist(temp,sample_point)<=min_dis)
                    {
                        switch(1)
                        {
                            case 0:
                            {
                                sample_valid = 0;
                                break;
                            }
                            case 1:
                            {
                                sample_valid = 1;
                                break;
                            }
                            case 2:
                            {
                                sample_valid = 1;
                                break;
                            }
                            case 3:
                            {
                                sample_valid = 1;
                                break;
                            }
                            case 4:
                            {
                                sample_valid = 1;
                                break;
                            }
                        }
                        
                    }
                    
                    if(sample_valid == 1 && (spx != i || spy!=j))
                    {
                        
                        parent_x[spx][spy]  = i;
                        parent_y[spx][spy]  = j;
                        level[spx][spy]     = level[i][j]+1;
                        is_map[spx][spy]    = 1;
                        //if()
                    }
                    /*else
                    {
                        is_map[spx][spy]    = 0;
                        cout << "f "<< spx << " " << spy;
                    }*/
                    
                }
                else
                {
                    ;//cout << "w ";
                }
            }
        }
        if(dist(end_point,sample_point)<=min_dis && sample_valid==1)
        {
            int levels=level[spx][spy];
            result.len = levels;
            result.x.resize(levels);
            result.y.resize(levels);
            int tempi = sample_point(0);
            int tempj = sample_point(1);
            for(int cc=0;cc<levels;cc++)
            {
                result.x[levels-cc-1] = tempj;
                result.y[levels-cc-1] = tempi;
                int a = tempi;
                int b = tempj;
                tempi = parent_x[a][b];
                tempj = parent_y[a][b];
                result.found=1;
            }
            return result;
        }
    }
    /*for(int i=0;i<30;i++)
        {
            for(int j=0;j<40;j++)
            {
                if(is_map[i][j]==1 && dist(i,j,))
                
            }
        }*/
    result.found=0;
    return result;
}


iarc::path rrt::output(float target_x,float target_y,float obs_x[],float obs_y[],int num)
{
    // obs = obstacle;
    // int obs_num = obs.num;
    end_point << (int)(target_x/0.1)+20,
                (int)(target_y/0.1)+15;

    // 1 route
    // 2 obstacle 
    // 0 not
    //srand((int)(time(0)*1000.)); 
    is_map[15][20] = 1;
    level[15][20] = 1;
    for(int i=0;i<num;i++)
    {
        fill(obs_x[i],obs_y[i],7);
    }
    
    //cout << "ec"<<endl;
    for(int seed;seed<=seeds;seed++)
    {
        // sample_point = resample(start_point,end_point);
        sample_point << (int)(39.0*rand()/(RAND_MAX+1.0)),
            (int)(29.0*rand()/(RAND_MAX+1.0));
        sample_point = resample();
        // cout <<"start" << end_point(0)<< "\t"<<end_point(1)<<"\t";
        int spx =(int)sample_point(0);
        int spy =(int)sample_point(1);
        // cout << sample_point(0) << "\t "<< sample_point(1)<<"sp"<<endl;
        bool sample_valid = 0;
        for(int i=0;i<29;i++)
        {
            for(int j=0;j<39;j++)
            {
                if(is_map[i][j]==1)
                {
                    Vector2f temp;
                    temp<< i,
                        j;
                        // cout <<"i"<< i<<"j"<<j<<endl;
                    /*if( dist(temp,sample_point)<=min_dis)
                    {
                        switch(1)
                        {
                            case 0:
                            {
                                sample_valid = 0;
                                break;
                            }
                            case 1:
                            {
                                sample_valid = 1;
                                break;
                            }
                            case 2:
                            {
                                sample_valid = 1;
                                break;
                            }
                            case 3:
                            {
                                sample_valid = 1;
                                break;
                            }
                            case 4:
                            {
                                sample_valid = 1;
                                break;
                            }
                        }
                        
                    }*/
                    
                    if(sample_valid == 1)
                    {
                        
                        parent_x[spx][spy]  = i;
                        parent_y[spx][spy]  = j;
                        level[spx][spy]     = level[i][j]+1;
                        is_map[spx][spy]    = 1;
                        //if()
                    }
                    else
                    {
                        is_map[spx][spy]    = 0;
                    }
                    
                }
                else
                {
                    ;//cout << "w ";
                }
            }
        }
        if(dist(end_point,sample_point)<=min_dis && sample_valid==1)
        {
            int levels=level[spx][spy];
            result.len = levels;
            result.x.resize(levels);
            result.y.resize(levels);
            result.rx.resize(levels);
            result.ry.resize(levels);
            int tempi = sample_point(0);
            int tempj = sample_point(1);
            for(int cc=0;cc<levels;cc++)
            {
                result.x[levels-cc-1] = tempi;
                result.y[levels-cc-1] = tempj;
                int a = tempi;
                int b = tempj;
                tempi = parent_x[a][b];
                tempj = parent_y[a][b];
                result.found=1;
            }
            int steer_counter=0;
            int rrt_delta=1;
            int x1_counter=0;
            for(int cc=0;cc<levels-1;)
            {
                int x1,x2,x3,y1,y2,y3;
                x1 = result.x[cc];
                x2 = result.x[cc+rrt_delta];
                x3 = result.x[cc+rrt_delta+1];
                y1 = result.y[cc];
                y2 = result.y[cc+rrt_delta];
                y3 = result.y[cc+rrt_delta+1];
                
                bool steer = steering( x1, x2, x3, y1, y2, y3);
                if(steer)
                {
                    result.rx[steer_counter]=x3;
                    result.ry[steer_counter]=y3;
                    rrt_delta++;
                }
                else
                {
                    result.rx[steer_counter]=x2;
                    result.ry[steer_counter]=y2;
                    steer_counter++;
                    cc = cc + rrt_delta;
                }
                x1_counter = cc+1;
            }
            result.len = x1_counter;
            result.rx.resize(x1_counter);
            result.ry.resize(x1_counter);
            return result;
        }
    }
    /*for(int i=0;i<30;i++)
        {
            for(int j=0;j<40;j++)
            {
                if(is_map[i][j]==1 && dist(i,j,))
                
            }
        }*/
    result.found=0;
    return result;
}