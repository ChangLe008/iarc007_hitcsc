#include <ros/ros.h>

#include <sys/time.h>
#include <unistd.h>
#include <time.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <iostream>
#include <iomanip>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <dji_sdk/Acceleration.h>
#include <iarc007/vehicle_pos.h>
#include "iarc007/jrh_pilot/JRH_math.h"
#include "iarc007/mjh_pos/estimater.h"

using namespace cv;
using namespace std;

int WIDTH=640;
int HEIGHT=480;

#define PI 3.1415926
#define IMAGE_SIZE (WIDTH*HEIGHT)

estimater location;//

Mat img_src;

/*	R_g_cz coming from initial calibration indicates tramsformation from ground to competition zone(All observe left-multiply principle)*/
CvMat* R_g_cz; //3*3
/*	R_cz_b indicates transformation from competition zone to body(All observe left-multiply principle)	*/                
CvMat* R_cz_b; //3×3
/*	R_g_b coming from aerial pilot indicates transformation from ground to body (All observe left-multiply principle)	*/
CvMat* R_g_b; //3*3
/*	R_cz_g coming from initial calibration indicates tramsformation from competition zone to ground(All observe left-multiply principle)*/
CvMat* R_cz_g;
/*	R_b_ca coming from initial calibration indicates tramsformation from body to camera(All observe left-multiply principle)*/
CvMat* R_b_ca;
/*	R_cz_ca indicates transformation from competion zone to camera(ALl observe left-multiply principle)	*/
CvMat* R_cz_ca;

CvMat* R_cz_b0;
CvMat* R_cz_b1;

CvMat* q_att; //4×1
CvMat* att; //3×1

CvMat* velocity; //3×1
CvMat* accelerometer; //3×1
CvMat* pos_mat; //3*1
CvMat* sonar_ca;
CvMat* sonar_cz;

CvMat* tf_1;
CvMat* tf_0;

CvMat* longth_zy;// ca to flow

ros::Subscriber mono_video_sub;
ros::Subscriber imu_sub;
ros::Subscriber velocity_sub;
ros::Subscriber position_sub;
ros::Subscriber ultrasonic_sub;

/*	Sonar's position in camera frame attained by measuring	*/
float sonar_in_ca[3] = {0.0,-0.29,-0.02};
/*	position	*/
float air_pos[3] = {0.};
float p_x = 0., p_y = 0., p_z = 0.;

float v_x,v_y,v_z;

float pilot_pos[3] = {0.};
bool first_flag = true;
bool first_angle_flag=true;
bool get_the_angle=false;
/*	attitude measured by imu	*/
float roll = 0., pitch = 0., yaw = 0.;
/*	update	period */
float t = 0.1;
/* error of yaw angle */
float e_xy = 0.;
float zy_cha=PI/9;
/* TIME */
double t_img = 0., t_gui = 0.;
struct Array2D
{
  int x;
  int y;
};
bool zy_tiaoshi=false;
bool zy_bundary=true;
//boundary  zuo biao xi guiding
const float x_0=-0.275;
const float x_20=21.25;
const float y_0=-0.29;
const float y_20=21.35;
struct boundary{
    bool meet_flag;
    vector<Vec2f> boundary_lines;
};
/* Mono Video */
FILE* fp = fopen("/home/hitcsc/catkin_ws/src/iarc007/log/vision_position.txt","w");
FILE* fp1 = fopen("/home/hitcsc/catkin_ws/src/iarc007/log/mjh_time_delay.txt","w");
FILE* fp2 = fopen("/home/hitcsc/catkin_ws/src/iarc007/log/globalwatch_position.txt","w");
FILE* fp3 = fopen("/home/hitcsc/catkin_ws/src/iarc007/log/velocity.txt","w");
FILE* fp4 = fopen("/home/hitcsc/catkin_ws/src/iarc007/log/sonar.txt","w");
FILE* fp5 = fopen("/home/hitcsc/catkin_ws/src/iarc007/log/acc.txt","w");
FILE* fp6 = fopen("/home/hitcsc/catkin_ws/src/iarc007/log/boundary.txt","w");
void mono_sub_callback(sensor_msgs::Image img_msg)
{ 
	cv_bridge::CvImagePtr img_bridge_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::RGB8);       
	img_bridge_ptr->image.copyTo(img_src);   
	
	t_img = img_msg.header.stamp.toSec();
   // cout << "time of img" << setprecision(9) << setiosflags(ios::fixed) << img_msg.header.stamp.toSec() << endl;
}

/* imu */
void imu_callback(const geometry_msgs::TransformStamped& g_imu)
{ 
/*	printf("mjh-frame_id: %s stamp: %d\n", g_imu.header.frame_id.c_str(), g_imu.header.stamp.sec );
	printf("mjh-imu: [%f %f %f %f %f %f %f]\n", g_imu.transform.translation.x,g_imu.transform.translation.y,\
	g_imu.transform.translation.z,g_imu.transform.rotation.x, g_imu.transform.rotation.y, g_imu.transform.rotation.z,\
	g_imu.transform.rotation.w );
*/	
	cvmSet(q_att,0,0,g_imu.transform.rotation.w);
	cvmSet(q_att,1,0,g_imu.transform.rotation.x);
	cvmSet(q_att,2,0,g_imu.transform.rotation.y);
	cvmSet(q_att,3,0,g_imu.transform.rotation.z);
	//printf("aerial pilot : %f %f %f %f\n",attitude_quaternion.q0,attitude_quaternion.q1,attitude_quaternion.q2,attitude_quaternion.q3);
	//cout << g_imu.transform.rotation.w << ' ' << g_imu.transform.rotation.x << ' ' << g_imu.transform.rotation.y \
        //     << ' ' << g_imu.transform.rotation.z << endl;
	Quaternion_To_Euler(q_att, att);

	roll = cvmGet(att,0,0);
	pitch = cvmGet(att,1,0);
	yaw = cvmGet(att,2,0);
	
	//cout << "yaw = " << yaw*180/PI << " pitch = " << pitch*180/PI << " roll = " << roll*180/PI << endl;
	/*	R_g_b coming from aerial pilot indicates transformation from ground to body (All observe left-multiply principle)	*/
	Euler_To_Matrix(roll,pitch,yaw,R_g_b);
/*
	cout << "---R_g_b---" << endl; 	
	for(int i = 0;i < 9;i++){
		cout << R_g_b->data.fl[i] << ' ';
		if(!((i+1)%3))
			cout << endl;
	}
*/
	/*	R_g_cz coming from initial calibration indicates tramsformation from ground to competition zone(All observe left-multiply principle)*/
	//cvTranspose(R_g_cz,R_cz_g);
        /*	R_cz_b indicates transformation from competition zone to body(All observe left-multiply principle)	*/                                   
	cvGEMM(R_cz_g,R_g_b,1,NULL,0,R_cz_b);
    //roll,pitch,yawt
    if (first_angle_flag){

        R_cz_b1=cvCloneMat(R_cz_b);
        cvGEMM(R_cz_b1,longth_zy,1,NULL,0,tf_1);
        first_angle_flag=false;
    }
    else
    {
          R_cz_b0=cvCloneMat(R_cz_b1);
          R_cz_b1=cvCloneMat(R_cz_b);

           tf_0=cvCloneMat(tf_1);

          cvGEMM(R_cz_b1,longth_zy,1,NULL,0,tf_1);


          get_the_angle=true;
    }

}
//acc
void acc_callback(const dji_sdk::Acceleration& g_acc)
{
    cvmSet(accelerometer, 0, 0, g_acc.ax);
    cvmSet(accelerometer, 1, 0, g_acc.ay);
    cvmSet(accelerometer, 2, 0, g_acc.az);

    cvGEMM(R_cz_g,accelerometer,1,NULL,0,accelerometer);

    float a_x = cvmGet(accelerometer, 0, 0);
    float a_y = cvmGet(accelerometer, 1, 0);
    float a_z = cvmGet(accelerometer, 2, 0);

    a_y = a_y*cos(e_xy) + a_x*sin(e_xy);
    a_x = -a_y*sin(e_xy) + a_x*cos(e_xy);

    if(location.start){

        location.a_x = a_x;
        location.a_y = a_y;

        location.vo_x = v_x;
        location.vo_y = v_y;

        if(air_pos[2] > 0.5){
            location.vo_update(1,0.8);
            location.estimate();
            //cout << "accelerometer: >0.3" << endl;
        }
        else{
            location.vo_update(2,0.5);
            location.estimate();
            //cout << "accelerometer: <0.3" << endl;
        }

        if(air_pos[2] <0.5){
            if (get_the_angle)
            {
           // cout<<"X:"<<p_x<<"  Y: "<<p_y<<" Z: "<<p_z<<endl;
            location.dp_x+= ((cvmGet(tf_1, 0, 0) - cvmGet(tf_0, 0, 0))*cos(e_xy)-(cvmGet(tf_1, 1, 0) - cvmGet(tf_0, 1, 0))*sin(e_xy));
            location.dp_y+=(cvmGet(tf_1, 1, 0) - cvmGet(tf_0, 1, 0))*cos(e_xy)+(cvmGet(tf_1, 0, 0) - cvmGet(tf_0, 0, 0))*sin(e_xy);
//            p_z+= (cvmGet(tf_1, 2,0) - cvmGet(tf_0,2,0));
               // float xxx=(cvmGet(tf_1, 0, 0) - cvmGet(tf_0, 0, 0));
            // cout<<"X: "<<xxx<<endl;
            }
        air_pos[0] += location.dp_x;
        air_pos[1] += location.dp_y;
        }
        //cout << "accelerometer" << location.start << endl;
    }
    else{
        location.initialize();
    }
    fprintf(fp5,"time= %f\tax= %f\tay= %f\taz= %f\n",g_acc.header.stamp.toSec(),a_x,a_y,a_z);
}


/* velocity */
void velocity_callback(const geometry_msgs::Vector3Stamped& g_vo)
{
/*
    printf("mjh--frame_id: %s stamp: %d\n", g_vo.header.frame_id.c_str(), g_vo.header.stamp.sec );
    printf("mjh--velocity: [%f %f %f]\n", g_vo.vector.x, g_vo.vector.y, g_vo.vector.z );
*/

    v_x = g_vo.vector.x;
    v_y = g_vo.vector.y;
    v_z = g_vo.vector.z;


    cvmSet(velocity, 0, 0, g_vo.vector.x);
    cvmSet(velocity, 1, 0, g_vo.vector.y);
    cvmSet(velocity, 2, 0, g_vo.vector.z);

    cvGEMM(R_cz_g,velocity,1,NULL,0,velocity);

    v_x = cvmGet(velocity, 0, 0);
    v_y = cvmGet(velocity, 1, 0);
    v_z = cvmGet(velocity, 2, 0);

    fprintf(fp3,"time= %f\tvx= %f\tvy= %f\t vz= %f\n",g_vo.header.stamp.toSec(),v_x,v_y,v_z);

/*	float tmp_vx,tmp_vy;
    tmp_vx = v_x;
    tmp_vy = v_y;
        v_x = tmp_vx*cos(e_xy) - tmp_vy*sin(e_xy);
    v_y = tmp_vx*sin(e_xy) + tmp_vy*cos(e_xy);
*/
}
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{
/*	printf("mjh-frame_id: %s stamp: %d\n", g_ul.header.frame_id.c_str(), g_ul.header.stamp.sec );

    for (int i = 0; i < 5; i++){
        printf("mjh--ultrasonic distance: [%f]  reliability: [%d]\n", g_ul.ranges[i], (int)g_ul.intensities[i] );
    }
*/

    cvGEMM(R_cz_ca,sonar_ca,1,NULL,0,sonar_cz);
    //air_pos[2] = g_ul.ranges[0] - 0.03;
    //cout << "height = " << g_ul.ranges[0] << endl;
/*	cout << "----sonar_cz-----" << endl;
    for(int i = 0;i < 3;i++){
        cout << cvmGet(sonar_cz,i,0) << ' ';
    }
    cout << endl;
*/
    if(g_ul.ranges[0] > 0){
        //air_pos[2] = g_ul.ranges[0]-0.02;// + cvmGet(sonar_cz,2,0);(cvmGet(tf_1, 2,0) - cvmGet(tf_0,2,0))

        if (get_the_angle){
        air_pos[2] = g_ul.ranges[0]+(cvmGet(tf_1, 2,0) - cvmGet(tf_0,2,0));
        }
        else{
            air_pos[2] = g_ul.ranges[0]-0.02;//
        }
    }
    else{
        //air_pos[2] = air_pos[2];
    }

    fprintf(fp4,"time= %f\theight= %f\n",g_ul.header.stamp.toSec(),air_pos[2]);
    //cout << "height = " << air_pos[2] << endl;
}
/* motion */
void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
//	printf("mjh--frame_id:%s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
//	for (int i = 0; i < 3; i++){
//		printf("mjh--global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
//	}
//	fprintf(fp,"%s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
//	for (int i = 0; i < 3; i++){
		//fprintf(fp2,"x=%f\t y=%f \tz=%f\ttime=%f\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z,);
//	}
//	fprintf(fp,"\n");
	if(first_flag)
	{
		cvmSet(pos_mat, 0, 0, g_pos.vector.x);
		cvmSet(pos_mat, 1, 0, g_pos.vector.y);
		cvmSet(pos_mat, 2, 0, g_pos.vector.z);
		t_gui = g_pos.header.stamp.toSec();
		cvGEMM(R_cz_g,pos_mat,1,NULL,0,pos_mat);
		fprintf(fp2,"x=%f\t y=%f \tz=%f\ttime=%f\n",  g_pos.vector.x ,  g_pos.vector.y, g_pos.vector.z,t_gui);
		p_x= cvmGet(pos_mat, 0, 0) - pilot_pos[0];
		p_y = cvmGet(pos_mat, 1, 0) - pilot_pos[1];
		p_z = cvmGet(pos_mat, 2, 0) - pilot_pos[2];

		pilot_pos[0] = cvmGet(pos_mat, 0, 0);
		pilot_pos[1] = cvmGet(pos_mat, 1, 0);
		pilot_pos[2] = cvmGet(pos_mat, 2, 0);

		first_flag = false;
	}
	else
	{
		cvmSet(pos_mat, 0, 0, g_pos.vector.x);
		cvmSet(pos_mat, 1, 0, g_pos.vector.y);
		cvmSet(pos_mat, 2, 0, g_pos.vector.z);
		t_gui = g_pos.header.stamp.toSec();
		cvGEMM(R_cz_g,pos_mat,1,NULL,0,pos_mat);
		fprintf(fp2,"x=%f\t y=%f \tz=%f\ttime=%f\n",  g_pos.vector.x ,  g_pos.vector.y, g_pos.vector.z,t_gui);
		p_x= cvmGet(pos_mat, 0, 0) - pilot_pos[0];
		p_y = cvmGet(pos_mat, 1, 0) - pilot_pos[1];
		p_z = cvmGet(pos_mat, 2, 0) - pilot_pos[2];

		pilot_pos[0] = cvmGet(pos_mat, 0, 0);
		pilot_pos[1] = cvmGet(pos_mat, 1, 0);
		pilot_pos[2] = cvmGet(pos_mat, 2, 0);
				
		//air_pos[0] += p_x;
		//air_pos[1] += p_y;
        if (get_the_angle)
        {
       // cout<<"X:"<<p_x<<"  Y: "<<p_y<<" Z: "<<p_z<<endl;
        p_x+= (cvmGet(tf_1, 0, 0) - cvmGet(tf_0, 0, 0));
        p_y+= (cvmGet(tf_1, 1, 0) - cvmGet(tf_0, 1, 0));
        p_z+= (cvmGet(tf_1, 2,0) - cvmGet(tf_0,2,0));
           // float xxx=(cvmGet(tf_1, 0, 0) - cvmGet(tf_0, 0, 0));
        // cout<<"X: "<<xxx<<endl;
        }
        if (air_pos[2] > 0.5){
        air_pos[1] += p_y*cos(e_xy) + p_x*sin(e_xy);
		air_pos[0] += -p_y*sin(e_xy) + p_x*cos(e_xy);
        }
		//air_pos[2] += p_z;
	}
	//t_gui = g_pos.header.stamp.toSec();
	//cout << "time of position of guidance" << setprecision(9) << setiosflags(ios::fixed) << g_pos.header.stamp.toSec() << endl;
	fprintf(fp,"x = %f\t y = %f\t z = %f\t  time=%f\n",air_pos[0],air_pos[1],air_pos[2],t_gui);
}

/** 
 * @brief 对输入图像进行细化 
 * @param src为输入图像,用cvThreshold函数处理过的8位灰度图像格式，元素中只有0与1,1代表有元素，0代表为空白 
 * @param maxIterations限制迭代次数，如果不进行限制，默认为-1，代表不限制迭代次数，直到获得最终结果 
 * @return 为对src细化后的输出图像,格式与src格式相同，元素中只有0与1,1代表有元素，0代表为空白 
 */  
cv::Mat thinImage(const cv::Mat & src, const int maxIterations = -1)  
{  
    assert(src.type() == CV_8UC1);  
    cv::Mat dst;  
    int width  = src.cols;  
    int height = src.rows;  
    src.copyTo(dst);  
    int count = 0;  //记录迭代次数  
    while (true)  
    {  
        count++;  
        if (maxIterations != -1 && count > maxIterations) //限制次数并且迭代次数到达  
            break;  
        std::vector<uchar *> mFlag; //用于标记需要删除的点  
        //对点标记  
        for (int i = 0; i < height ;++i)  
        {  
            uchar * p = dst.ptr<uchar>(i);  
            for (int j = 0; j < width; ++j)  
            {  
                //如果满足四个条件，进行标记  
                //  p9 p2 p3  
                //  p8 p1 p4  
                //  p7 p6 p5  
                uchar p1 = p[j];  
                if (p1 != 1) continue;  
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);  
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);  
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);  
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);  
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);  
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);  
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);  
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);  
                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)  
                {  
                    int ap = 0;  
                    if (p2 == 0 && p3 == 1) ++ap;  
                    if (p3 == 0 && p4 == 1) ++ap;  
                    if (p4 == 0 && p5 == 1) ++ap;  
                    if (p5 == 0 && p6 == 1) ++ap;  
                    if (p6 == 0 && p7 == 1) ++ap;  
                    if (p7 == 0 && p8 == 1) ++ap;  
                    if (p8 == 0 && p9 == 1) ++ap;  
                    if (p9 == 0 && p2 == 1) ++ap;  
  
                    if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)  
                    {  
                        //标记  
                        mFlag.push_back(p+j);  
                    }  
                }  
            }  
        }  
  
        //将标记的点删除  
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)  
        {  
            **i = 0;  
        }  
  
        //直到没有点满足，算法结束  
        if (mFlag.empty())  
        {  
            break;  
        }  
        else  
        {  
            mFlag.clear();//将mFlag清空  
        }  
  
        //对点标记  
        for (int i = 0; i < height; ++i)  
        {  
            uchar * p = dst.ptr<uchar>(i);  
            for (int j = 0; j < width; ++j)  
            {  
                //如果满足四个条件，进行标记  
                //  p9 p2 p3  
                //  p8 p1 p4  
                //  p7 p6 p5  
                uchar p1 = p[j];  
                if (p1 != 1) continue;  
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);  
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);  
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);  
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);  
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);  
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);  
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);  
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);  
  
                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)  
                {  
                    int ap = 0;  
                    if (p2 == 0 && p3 == 1) ++ap;  
                    if (p3 == 0 && p4 == 1) ++ap;  
                    if (p4 == 0 && p5 == 1) ++ap;  
                    if (p5 == 0 && p6 == 1) ++ap;  
                    if (p6 == 0 && p7 == 1) ++ap;  
                    if (p7 == 0 && p8 == 1) ++ap;  
                    if (p8 == 0 && p9 == 1) ++ap;  
                    if (p9 == 0 && p2 == 1) ++ap;  
  
                    if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)  
                    {  
                        //标记  
                        mFlag.push_back(p+j);  
                    }  
                }  
            }  
        }  
  
        //将标记的点删除  
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)  
        {  
            **i = 0;  
        }  
  
        //直到没有点满足，算法结束  
        if (mFlag.empty())  
        {  
            break;  
        }  
        else  
        {  
            mFlag.clear();//将mFlag清空  
        }  
    }  
    return dst;  
} 

boundary boun_det(const cv::Mat & canny_src)
{
    cv::Mat canny_result;
    struct boundary zy_boundary;
    Canny(canny_src,canny_result,50,150);
    canny_result=canny_result*255;
     HoughLines( canny_result, zy_boundary.boundary_lines, 1, CV_PI/180, 120 );
    
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"mjh_vision_pos");
	ros::NodeHandle my_node;

    FILE *jrh_q=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/Q_INIT.txt","r");
	if(!jrh_q)
	{
		std::cout << "Error In File mjh_vision_pos: file Q_INIT open failed !" << std::endl;
		return 0;
	}	

	float q0,q1,q2,q3;
	CvMat *q_init=cvCreateMat(4,1,CV_32FC1);
	CvMat *att_init=cvCreateMat(3,1,CV_32FC1);
	R_g_cz=cvCreateMat(3,3,CV_32FC1);
    velocity=cvCreateMat(3,1,CV_32FC1);
    accelerometer=cvCreateMat(3,1,CV_32FC1);

	fscanf(jrh_q,"%f%f%f%f\n",&q0,&q1,&q2,&q3);
	fclose(jrh_q);

    	cvmSet(q_init, 0, 0, q0);
	cvmSet(q_init, 1, 0, q1);
	cvmSet(q_init, 2, 0, q2);
	cvmSet(q_init, 3, 0, q3);
	
	Quaternion_To_Euler(q_init,att_init);

/*	roll = cvmGet(att_init,0,0);
	pitch = cvmGet(att_init,1,0);
	yaw = cvmGet(att_init,2,0);
	
	cout << "yaw = " << yaw*180/PI << " pitch = " << pitch*180/PI << " roll = " << roll*180/PI << endl;
*/
	Euler_To_Matrix(cvmGet(att_init,0,0),cvmGet(att_init,1,0),cvmGet(att_init,2,0),R_g_cz);
	//Euler_To_Matrix(0.,0.,-95./180*PI,R_g_cz);
/*	cout << "---R_g_cz---" << endl; 	
	for(int i = 0;i < 9;i++){
		cout << R_g_cz->data.fl[i] << ' ';
		if(!((i+1)%3))
			cout << endl;
	}
*/

	//mono_video_sub = my_node.subscribe<sensor_msgs::Image>("/iarc007/mono_video",2,mono_sub_callback);
	mono_video_sub = my_node.subscribe<sensor_msgs::Image>("/iarc007/mono_video1",2,mono_sub_callback);
	velocity_sub = my_node.subscribe("/guidance/velocity",10, velocity_callback);
	position_sub = my_node.subscribe("/guidance/position",10, position_callback);
	imu_sub = my_node.subscribe("/guidance/imu", 10, imu_callback);
	ultrasonic_sub = my_node.subscribe("/guidance/ultrasonic", 10, ultrasonic_callback);

        ros::Subscriber acc_sub = my_node.subscribe("/dji_sdk/acceleration",50,acc_callback);
	/* Loop for mono_sub_callback */
	ros::Publisher position_pub = my_node.advertise<iarc007::vehicle_pos>("/iarc007/vehicle_pos",10);

	iarc007::vehicle_pos position;
	position.header.seq = 0;
	position.header.frame_id = "iarc007";

    ros::Rate rate(5);

	R_cz_g=cvCreateMat(3,3,CV_32FC1);
	cvTranspose(R_g_cz,R_cz_g);
	//velocity=cvCreateMat(3,1,CV_32FC1);
	pos_mat = cvCreateMat(3,1,CV_32FC1);

    tf_0=cvCreateMat(3,1,CV_32FC1);
    tf_1=cvCreateMat(3,1,CV_32FC1);

     longth_zy=cvCreateMat(3,1,CV_32FC1);

     cvmSet(longth_zy, 0, 0, 0.27);
     cvmSet(longth_zy, 1, 0, 0);
     cvmSet(longth_zy, 2, 0, 0);

	/*	extern parameter of cam(indicates transformation from camera to cheessboard(All observe left-multiply principle))	*/
    CvMat* R_ca_chess=(CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/Rotation.xml");
	/*	R_ca_b indicates transformation from camera to body(All observe left-multiply principle) 	*/
	CvMat *R_ca_b=cvCreateMat(3,3,CV_32FC1);
	/*	R_chess_b indicates transformation from chessboard to body(All observe left-multiply principle) 	*/
	CvMat *R_chess_b=cvCreateMat(3,3,CV_32FC1);
	
	/*cvZero(R_chess_b);
	cvmSet(R_chess_b,0,0,1.0);
	cvmSet(R_chess_b,1,1,-1.0);
	cvmSet(R_chess_b,2,2,-1.0);
	*/
    R_chess_b=(CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/R_chess_b.xml");

	cvGEMM(R_ca_chess,R_chess_b,1,NULL,0,R_ca_b);
	/*	R_cz_ca indicates transformation from competion zone to camera(ALl observe left-multiply principle)	*/
	R_cz_ca=cvCreateMat(3,3,CV_32FC1);
	CvMat* R_ca_cz=cvCreateMat(3,3,CV_32FC1);
	CvMat* R_b_ca=cvCreateMat(3,3,CV_32FC1);
	cvTranspose(R_ca_b,R_b_ca);

	R_cz_b = cvCreateMat(3,3,CV_32FC1);
    R_cz_b0 = cvCreateMat(3,3,CV_32FC1);
    R_cz_b1 = cvCreateMat(3,3,CV_32FC1);
	R_g_b = cvCreateMat(3,3,CV_32FC1);

	q_att = cvCreateMat(4,1,CV_32FC1);
	att = cvCreateMat(3,1,CV_32FC1);
//    velocity=cvCreateMat(3,1,CV_32FC1);
//    accelerometer=cvCreateMat(3,1,CV_32FC1);

	sonar_ca=cvCreateMat(3,1,CV_32FC1);
	
	for(int i = 0;i < 3;i++){
		sonar_ca->data.fl[i] = sonar_in_ca[i];		
	}

	sonar_cz=cvCreateMat(3,1,CV_32FC1);

	IplImage* img = cvCreateImage(cvSize(WIDTH,HEIGHT),8,3);
	IplImage* pImg = cvCreateImage(cvSize(WIDTH,HEIGHT),8,1);
    IplImage* HSV_pImg = cvCreateImage(cvSize(WIDTH,HEIGHT),8,3);
	//IplImage* dst = cvCreateImage (cvSize(WIDTH,HEIGHT), IPL_DEPTH_32F, 1); 

    CvMat* mono_intrinsic = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/Intrinsics.xml");
	//CvMat* T_ca_cz = cvCreateMat(3,4,CV_32FC1);
	CvMat* Q = cvCreateMat(3,3,CV_32FC1);
	//CvMat* Q_inter = cvCreateMat(4,4,CV_32FC1);
	CvMat* T_inv = cvCreateMat(3,3,CV_32FC1);

	CvMat* Vec_pixel_A = cvCreateMat(3,1,CV_32FC1);
	CvMat* Vec_pixel_B = cvCreateMat(3,1,CV_32FC1);

	CvMat* Vec_physic_A = cvCreateMat(3,1,CV_32FC1);
	CvMat* Vec_physic_B = cvCreateMat(3,1,CV_32FC1);

	//CvSeq* lines;
	//CvMemStorage* storage = cvCreateMemStorage(0);
	
	bool flag = false;
	vector<Vec2f> lines;

    vector<float> x_lines;
    vector<float> y_lines;
	//cout << setw(16) << setiosflags(ios::left);



	float drho_x = 0.,drho_y = 0.;
    float boundary_x=0.,boundary_y=0.;
	//CvVideoWriter* writer = cvCreateVideoWriter("grid.avi",CV_FOURCC('X','V','I','D'),5,cvSize(640,480));

	while(ros::ok())
	{
		double time = ros::Time::now().toSec();
		//cout << "--------------"<< time <<"-------------------" << endl;
		//cout << "registration error" << ' ' << setprecision(9) << setiosflags(ios::fixed) << t_gui - t_img << endl;
		//cout << "img_delay" << ' ' << setprecision(9) << setiosflags(ios::fixed) << time - t_img << endl;
		//cout << "gui_delay" << ' ' << setprecision(9) << setiosflags(ios::fixed) << time - t_gui << endl;

		fprintf(fp1,"registration error = %.6f\timg_delay = %.6f\tgui_delay = %.6f\n",t_gui - t_img,time - t_img,time - t_gui);
		//fprintf(fp2,"x=%f\t y=%f \tz=%f\ttime=%f\n", air_pos[0] , air_pos[1], air_pos[2],t_gui);
		if(!img_src.data)
		{
			cout << "--------------error: img is null-------------------" << endl;

		}

//		cout << "-------air_pos------" << endl;
//		for(int i = 0;i < 3;i++){
//			cout << air_pos[i] << ' ';
//			if(!((i+1)%3))
//				cout << endl;
//		}

		if(img_src.data)
		{
/*
			air_pos[0] += v_x*t;
			air_pos[1] += v_y*t;
			cout << "v_x = " << v_x << " v_y = " << v_y << endl;
			pilot_pos[0] += v_x*t;
			pilot_pos[1] += v_y*t;
*/
			//imshow("mono_video_sub",img_src);
			*img = IplImage(img_src);

			//cvWriteFrame(writer,img);

			/* Extract position of aircraft */
			{
//********************************
                cvCvtColor (img,pImg,CV_BGR2GRAY);
                cv::Mat src = Mat(pImg,false);
                //将原图像转换为二值图像
                cv::threshold(src, src, 140, 1, cv::THRESH_BINARY);

				int morph_elem = 0;
				int morph_size = 2;

				Mat element = getStructuringElement( morph_elem, Size( 2*morph_size \
						+ 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );		
				/// 运行指定形态学操作
				int operation = 2;
				morphologyEx( src, src, operation, element );
				morphologyEx( src, src, 3, element );

				cv::Mat dst = thinImage(src);  
                //显示图像
                dst = dst * 255;
                src = src*255;

                imshow("dst",dst);
                imshow("src",src);
//**************************************************8
                //***************************************************/////////////////////////////
                         vector<Mat> MV;
                         cv::Mat H_image=Mat(HEIGHT,WIDTH,CV_8UC1);
                         cv::Mat S_image=Mat(HEIGHT,WIDTH,CV_8UC1);
                         cv::Mat V_image=Mat(HEIGHT,WIDTH,CV_8UC1);

                          cvCvtColor (img,HSV_pImg,CV_BGR2HSV);
                          cv::Mat HSV_src=Mat(HSV_pImg,false);
                          cv::split(HSV_src,MV);

                    MV[0].convertTo(H_image,CV_8U);
                    MV[1].convertTo(S_image,CV_8U);
                    MV[2].convertTo(V_image,CV_8U);

                    cv::threshold(S_image, S_image, 30, 1, cv::THRESH_BINARY_INV);
                    cv::threshold(V_image, V_image, 200, 1, cv::THRESH_BINARY);
                     cv::Mat canny_src=S_image.mul(V_image);
                     morphologyEx( canny_src, canny_src, operation, element );
                     morphologyEx( canny_src, canny_src, 3, element );

                     cv::Mat canny_result;
                     //
                     // canny:::
                      canny_src=canny_src*255;
                      cv::Mat canny_a;
                      canny_src.convertTo(canny_a,CV_8U);
                     Canny(canny_a,canny_result,50,150);                                        //zy

                     //                   canny_result = thinImage(canny_src);
                       
                      
                      
                      //tell me weather  I meet the boundary 
//                        struct boundary zy_boundary;
//                        zy_boundary=boun_det(canny_src);
                        
                      //
                     // canny_src=canny_src*255;
                      canny_result=canny_result*255;
                      imshow("canny_src",canny_src);
                     imshow("canny_result",canny_result);

                     HoughLines( canny_result, lines, 1, CV_PI/180, 80 );
                    cv::Mat image_src=img_src;
                       for( size_t i = 0; i < lines.size(); i++ )
                       {
                           float rho = lines[i][0];
                           float theta = lines[i][1];

                           double a = cos(theta), b = sin(theta);
                           double x0 = a*rho, y0 = b*rho;
                           Point pt1(cvRound(x0 + 1000*(-b)),
                                 cvRound(y0 + 1000*(a)));
                           Point pt2(cvRound(x0 - 1000*(-b)),
                                 cvRound(y0 - 1000*(a)));
                           line( image_src, pt1, pt2, Scalar(0,0,255), 3, 8 );
                           //cout << "rho" << rho << "theta" << theta << endl;

                       }
            imshow("image_src",image_src);
                  //***************************************************/////////////////////////////



//                HoughLines( dst, lines, 1, CV_PI/180, 120 );//   MJH

				//cout << "Start" << lines.size() << endl;		
				for( size_t i = 0; i < lines.size(); i++ )
				{
					float rho = lines[i][0];
					float theta = lines[i][1];					
				
					double a = cos(theta), b = sin(theta);
					double x0 = a*rho, y0 = b*rho;
					Point pt1(cvRound(x0 + 1000*(-b)),
						  cvRound(y0 + 1000*(a)));
					Point pt2(cvRound(x0 - 1000*(-b)),
						  cvRound(y0 - 1000*(a)));
					line( img_src, pt1, pt2, Scalar(0,0,255), 3, 8 );
					//cout << "rho" << rho << "theta" << theta << endl; 
				}
				//cout << "End" << endl;				

				imshow("image",img_src);

				char keycode=cvWaitKey(10);             
				if(keycode == 27){
					break;
				}

				lines.size() > 0 ? flag = true : flag = false;
                               	
			}
		
			/* Calculate position of aircraft */
			/* R_cz_b indicates transformation from camera to body(All observe left-multiply principle) */
			/* R_cz_ca indicates transformation from competion zone to camera(ALl observe left-multiply principle) */
			cvGEMM(R_cz_b,R_b_ca,1,NULL,0,R_cz_ca);

			cvTranspose(R_cz_ca,R_ca_cz);

/*			cout << "---R_ca_cz---" << endl; 	
			for(int i = 0;i < 9;i++){
				cout << R_ca_cz->data.fl[i] << ' ';
				if(!((i+1)%3))
					cout << endl;
			}
*/
			//flag = false;
            if(air_pos[2] < 0.3){
                flag = false;
            }
            else{
                flag = true;
            }
			if(flag)
			{
                cout<<"zy_shuai"<<endl;
                float error_xy[200];
              int  j_zy=0;
				//TODO Localization according to Grid Points
				cvGEMM(mono_intrinsic,R_ca_cz,1,NULL,0,Q);
			
				cvInvert(Q,T_inv);
				
				int* lines_flag = new int[lines.size()];

				for( size_t i = 0; i < lines.size(); i++ )
				{
					float rho = lines[i][0];
					float theta = lines[i][1];					
				
					double a = cos(theta), b = sin(theta);
					double x0 = a*rho, y0 = b*rho;
					
					cvmSet(Vec_pixel_A,0,0,x0-1000*(-b));
					cvmSet(Vec_pixel_A,1,0,y0-1000*(a));
					cvmSet(Vec_pixel_A,2,0,1);
	
					cvmSet(Vec_pixel_B,0,0,x0+1000*(-b));
					cvmSet(Vec_pixel_B,1,0,y0+1000*(a));
					cvmSet(Vec_pixel_B,2,0,1);

					cvGEMM(T_inv,Vec_pixel_A,1,NULL,0,Vec_physic_A);
					cvGEMM(T_inv,Vec_pixel_B,1,NULL,0,Vec_physic_B);

					float s_A,s_B;

					s_A = Vec_physic_A->data.fl[2]/air_pos[2];
					s_B = Vec_physic_B->data.fl[2]/air_pos[2];

					Vec_physic_A->data.fl[2] = air_pos[2];
					Vec_physic_B->data.fl[2] = air_pos[2];

					Vec_physic_A->data.fl[0] /= s_A; 
					Vec_physic_A->data.fl[1] /= s_A;

					float ax,ay;
					
					ax = Vec_physic_A->data.fl[0];
					ay = Vec_physic_A->data.fl[1];

					Vec_physic_A->data.fl[0] = -ay*sin(e_xy) + ax*cos(e_xy);					
					Vec_physic_A->data.fl[1] = ay*cos(e_xy) + ax*sin(e_xy);					

					Vec_physic_A->data.fl[0] += air_pos[0];
					Vec_physic_A->data.fl[1] += air_pos[1];

					Vec_physic_B->data.fl[0] /= s_B;
					Vec_physic_B->data.fl[1] /= s_B;

					float bx,by;
					
					bx = Vec_physic_B->data.fl[0];
					by = Vec_physic_B->data.fl[1];

					Vec_physic_B->data.fl[0] = -by*sin(e_xy) + bx*cos(e_xy);					
					Vec_physic_B->data.fl[1] = by*cos(e_xy) + bx*sin(e_xy);

					Vec_physic_B->data.fl[0] += air_pos[0];
					Vec_physic_B->data.fl[1] += air_pos[1];
					
	
					/*cout << "--------physic_A-------" << endl;
					for(int i = 0;i < 3;i++){
						cout << cvmGet(Vec_physic_A,i,0) << ' ';
						if(!((i+1)%3))
							cout << endl;
					}

					cout << "--------physic_B-------" << endl;
					for(int i = 0;i < 3;i++){
						cout << cvmGet(Vec_physic_B,i,0) << ' ';
						if(!((i+1)%3))
							cout << endl;
					}*/

					/*if(fabs(Vec_physic_B->data.fl[1]-Vec_physic_A->data.fl[1]) < ems){

						lines[i][0] = fabs(Vec_physic_B->data.fl[1]+Vec_physic_A->data.fl[1])/2.0;
						if((Vec_physic_B->data.fl[1]+Vec_physic_A->data.fl[1])/2.0 > 0.){
							lines[i][1] = PI/2.;
						}
						else{
							lines[i][1] = -PI/2.;
						}
						lines_flag[i] = 1;
					}
					
					else if(fabs(Vec_physic_B->data.fl[0]-Vec_physic_A->data.fl[0]) < ems)
					{
						lines[i][0] = fabs(Vec_physic_B->data.fl[0]+Vec_physic_A->data.fl[0])/2.0;
						if((Vec_physic_B->data.fl[0]+Vec_physic_A->data.fl[0])/2.0 > 0.){
							lines[i][1] = 0.;
						}
						else{
							lines[i][1] = PI;
						}
						lines_flag[i] = 1;
					}*/
					lines[i][1] = atan(-(Vec_physic_B->data.fl[0] - Vec_physic_A->data.fl[0])/(Vec_physic_B->data.fl[1] - Vec_physic_A->data.fl[1]));

					lines[i][0] = Vec_physic_B->data.fl[0] * cos(lines[i][1]) + Vec_physic_B->data.fl[1] * sin(lines[i][1]);
                    float zy_xx=lines[i][1]*180/PI;
                       cout<<"zhi xian "<<zy_xx<<endl;
					
					//cout << "B_X = " << Vec_physic_B->data.fl[0] << "  B_Y = " << Vec_physic_B->data.fl[1] << endl;
					//lines[i][1] = lines[i][1] + e_xy;
					//cout << "lines[i][1]" << lines[i][1] << endl;
					//cout << "lines[i][0]" << lines[i][0] << endl;

                       if (lines[i][0] >= 0.) {

                           if (fabs(lines[i][1] - PI / 2.) < PI / 18.) {
                               error_xy[j_zy]=PI / 2.-lines[i][1] ;
                                 j_zy++;
                                 y_lines.push_back(lines[i][0]);

                                 lines[i][1] = PI / 2.;
                               lines_flag[i] = 1;
                           }
                           else if (fabs(lines[i][1] + PI / 2.) < PI / 18.) {
                               error_xy[j_zy]=-PI / 2.-lines[i][1] ;
                                 j_zy++;
                                 y_lines.push_back(lines[i][0]);

                               lines[i][1] = -PI / 2.;
                               lines_flag[i] = 1;
                           }
                           else if (fabs(lines[i][1]) < PI / 18.) {

                               error_xy[j_zy]=0.-lines[i][1] ;
                                 j_zy++;
                                x_lines.push_back(lines[i][0]);
                               lines[i][1] = 0;
                               lines_flag[i] = 1;
                           }

                           else
                           {
                               lines[i][0] = 0.;
                               lines[i][1] = 0.;
                               lines_flag[i] = 0;
                           }
                       }

                       else{

                                                   if(lines[i][1] >= 0.){
                               if (fabs(lines[i][1] - PI + PI/2.) < PI / 18.) {

                                   error_xy[j_zy]=PI/2.-lines[i][1] ;
                                     j_zy++;
                                    y_lines.push_back(lines[i][0]);

                                   lines[i][1] = -PI/2.;
                                   lines_flag[i] = 1;
                               }
                               else if (fabs(lines[i][1] - PI + PI) < PI / 18.) {
                                   error_xy[j_zy]=-lines[i][1] ;
                                     j_zy++;
                                     x_lines.push_back(lines[i][0]);

                                   lines[i][1] = -PI;
                                   lines_flag[i] = 1;
                               }
                               else
                               {
                                   lines[i][0] = 0.;
                                   lines[i][1] = 0.;
                                   lines_flag[i] = 0;
                               }
                                                   }
                                                   else{
                               if (fabs(lines[i][1] + PI -PI/2.) < PI / 18.) {
                                   error_xy[j_zy]=-PI/2.-lines[i][1] ;
                                     j_zy++;
                                    y_lines.push_back(lines[i][0]);

                                   lines[i][1] = PI/2.;
                                   lines_flag[i] = 1;
                               }
                               else if (fabs(lines[i][1] + PI -PI) < PI / 18.) {
                                   error_xy[j_zy]=-lines[i][1] ;
                                     j_zy++;
                                     x_lines.push_back(lines[i][0]);

                                   lines[i][1] = PI;
                                   lines_flag[i] = 1;
                               }
                               else
                               {
                                   lines[i][0] = 0.;
                                   lines[i][1] = 0.;
                                   lines_flag[i] = 0;
                               }
                           }
                           lines[i][0] = fabs(lines[i][0]);
                           //TODO
                           //lines[i][0] /= cos(e_xy);
                       }
                       //cout << "lines[i][1]--------------:" << lines[i][1] << endl;
                       //cout << "lines[i][0]--------------:" << lines[i][0] << endl;
                       /*else
                       {
                           lines[i][0] = 0.;
                           lines[i][1] = 0.;
                           lines_flag[i] = 0;
                       }*/
				}
                   float zy_ey=0;
                for (int num_zy=0;num_zy<j_zy;num_zy++){
                    zy_ey+=error_xy[num_zy];
                }
                if(j_zy){
                e_xy=e_xy+zy_ey/j_zy;
                }
                else{e_xy=0;}
                float mmm=e_xy*180/PI;
                cout<<j_zy<<endl;
                cout<<"zy_jiaozheng :  "<<mmm<<endl;
				int sum_x = 0,sum_y = 0;
                int sum_b_x=0,sum_b_y=0;

                float tmp_rho_x = 0.5,tmp_rho_y = 0.5;//limit rho
				sum_x = 0,sum_y = 0,drho_x = 0.,drho_y = 0.;
                boundary_x=0.,boundary_y=0.;
                //baundary hhhhhhhhh
                if(x_lines.size()||y_lines.size())
                {
                     if(x_lines.size()>1)//x_lines boundary find
                     {
                        sort(x_lines.begin(),x_lines.end());//sheng xu
                         float first;
                         float second;
                         bool x_line_flag=false;
                         float midl;
                        for(int i=0;i<x_lines.size()-1;i++)
                        {
                            first =x_lines[i];
                            for (int j=i+1;j<x_lines.size();j++)
                            {
                                second=x_lines[j];
                                if(fabs(fabs(first-second)-0.54)<0.15)
                                {
                                    midl=(first+second)/2;
                                    if(fabs(midl-x_0)<3)
                                    {
                                        boundary_x+=(x_0-midl);
                                        sum_b_x++;
                                        x_line_flag=true;
                                       
                                    }
                                   else if(fabs(midl-x_20)<3)
                                    {
                                        boundary_x+=(x_20-midl);
                                        sum_b_x++;
                                        x_line_flag=true;
                                    }
                                }
                            }
                        }
                        if(x_line_flag){     boundary_x/=sum_b_x;   cout<<boundary_x<<endl; fprintf(fp6,"midl = %.6f\t boundary_x = %f\t time=%.6f\n",midl,boundary_x,time); }
                     }
                     if(y_lines.size()>1)
                     {
                         sort(y_lines.begin(),y_lines.end());//sheng xu
                         float first;
                         float second;
                         float midl;
                         bool y_line_flag=false;
                        for(int i=0;i<y_lines.size()-1;i++)
                        {
                            first =y_lines[i];
                            for (int j=i+1;j<y_lines.size();j++)
                            {
                                second=y_lines[j];
                                if(fabs(fabs(first-second)-0.5)<0.15)
                                {
                                    midl=(first+second)/2;
                                    if(fabs(midl-y_0)<3)
                                    {
                                        boundary_y+=(y_0-midl);
                                        sum_b_y++;
                                        y_line_flag=true;
                                    }
                                   else if(fabs(midl-y_20)<3)
                                    {
                                        boundary_y+=(y_20-midl);
                                        sum_b_y++;
                                        y_line_flag=true;
                                    }
                                }
                            }
                        }
                              if(y_line_flag){     boundary_y/=sum_b_y;   cout<<boundary_y<<endl; fprintf(fp6,"midl = %.6f\t boundary_y = %.6f\t time=%f\n",midl,boundary_y,time);}
                     }
                }
                x_lines.clear();
                y_lines.clear();
                //hhhhhhhhhhhhhh
				for(int i = 0;i < lines.size();i++){  
					if(lines_flag[i] && fabs(cvRound(lines[i][0]) - lines[i][0]) < tmp_rho_y){
						if(fabs(lines[i][1]) < 0.1 || fabs(lines[i][1]-PI/2.) < 0.1){
							lines[i][0] -= cvRound(lines[i][0]);
							//cout << "lines[i][0]a " << lines[i][0] << endl;
								
							// && fabs(lines[i][0]) < tmp_rho_x
							if(fabs(lines[i][1]) < 0.1){
								drho_x += lines[i][0];
								sum_x += 1;
								//cout << "x_0 = " << lines[i][0] << endl;
							}
							else{
								drho_y += lines[i][0];
								sum_y += 1;
								//cout << "y_pi_2 = " << lines[i][1] << endl;
							}
							//cout << "sum_x = " << sum_x << endl;
						}
						else
						{
							lines[i][0] = cvRound(lines[i][0]) - lines[i][0];
							//lines[i][0] -= cvRound(lines[i][0]);// - lines[i][0];
							//cout << "lines[i][0]b " << lines[i][0] << endl;
							//&& fabs(lines[i][0]) < tmp_rho_x
							if(fabs(lines[i][1] - PI) < 0.1){
								drho_x += lines[i][0];
								sum_x += 1;
								//cout << "x_pi = " << lines[i][0] << endl;
							}
							else{ 
								drho_y += lines[i][0];
								sum_y += 1;
								//cout << "y_-pi_2 = " << lines[i][1] << endl;
							}
						}
					}
					//cout << "drho = " << lines[i][0] << endl;
				}
								
				if(sum_x){
					drho_x /= sum_x;
				}
				else{
					drho_x = 0.;
				}
				if(sum_y){
					drho_y /= sum_y;
				}
				else{
					drho_y = 0.;
				}
                                
				//cout << "drho_x = " << drho_x << "  drho_y = " << drho_y << endl;				
	
                                //cout << "sum_x " << sum_x << " sum_y " << sum_y << " drho_x " << drho_x << " drho_y " << drho_y << endl;
				//cout << "v_x*t" << v_x*t << "v_y*t" << v_y*t << endl;
		
	//v_x = tmp_vx*cos(e_xy) - tmp_vy*sin(e_xy);
	//v_y = tmp_vx*sin(e_xy) + tmp_vy*cos(e_xy);
						
				//cout << " drhox = " << drho_x*cos(e_xy) + drho_y*sin(e_xy) << endl;
				//cout << " drhoy = " << -drho_x*sin(e_xy) + drho_y*cos(e_xy) << endl;

				air_pos[0] -= drho_x;
				air_pos[1] -= drho_y;
                cout<<"jiao zheng hou x:"<<air_pos[0]<<"y:"<<air_pos[1]<<endl;
                if(zy_bundary){
                air_pos[0]+=boundary_x;
                air_pos[1]+=boundary_y;
                cout<<"boundary jiao zheng hou x:"<<air_pos[0]<<"y:"<<air_pos[1]<<endl;
                cout<<"x:"<<boundary_x<<"y:"<<boundary_y<<endl;
                }
                /*
				float dx = 0., dy = 0.;
	
				dx = dy = 0.;

				if((-drho_x) >= 0){
					dx += -drho_x*cos(e_xy);
					dy += -(-drho_x)*sin(e_xy);					
				}
				else{
					dx += (-drho_x)*cos(e_xy);
					dy += -(-drho_x)*sin(e_xy);
				}

				if((-drho_y) >= 0){
					dx += -drho_y*sin(e_xy);
					dy += -drho_y*cos(e_xy);
				}
				else{
					dx += (-drho_y)*sin(e_xy);
					dy += (-drho_y)*cos(e_xy);
				}
*/
				//air_pos[0] -= (drho_x*cos(e_xy) - drho_y*sin(e_xy));
				//air_pos[1] -= (drho_x*sin(e_xy) + drho_y*cos(e_xy));

				//air_pos[0] -= (drho_x*cos(e_xy) + drho_y*sin(e_xy));
				//air_pos[1] -= (-drho_x*sin(e_xy) + drho_y*cos(e_xy));

				//air_pos[0] -= drho_x;
				//air_pos[1] -= drho_y;

				//air_pos[0] += dx;
				//air_pos[1] += dy;

				delete [] lines_flag;

			}

		}
		else
		{
/*
			air_pos[0] += v_x*t;
			air_pos[1] += v_y*t;
				
			pilot_pos[0] += v_x*t;
			pilot_pos[1] += v_y*t;
*/
		}
		
		//temp---->e_xy
/*	
		float tmp_posx,tmp_posy,pos_x,pos_y;
		tmp_posx = air_pos[0];
		tmp_posy = air_pos[1];
		pos_x = tmp_posx*cos(e_xy) - tmp_posy*sin(e_xy);
		pos_y = tmp_posx*sin(e_xy) + tmp_posy*cos(e_xy);
		fprintf(fp,"x = %f\ty = %f\tz = %f\tx_v = %f\ty_v = %f\tdx = %f\tdy = %f \n",pos_x,pos_y,air_pos[3],pilot_pos[0],pilot_pos[1],drho_x,drho_y);
*/
		//fprintf(fp,"x = %f\ty = %f\tz = %f\tx_v = %f\ty_v = %f\tdx = %f\tdy = %f \n",air_pos[0],air_pos[1],air_pos[3],pilot_pos[0],pilot_pos[1],drho_x,drho_y);

		//fprintf(fp,"x = %f\t y = %f\t z = %f\t dx = %f\t dy = %f\t time=%f\n",air_pos[0],air_pos[1],air_pos[2],drho_x,drho_y,t_gui);
		
		position.header.stamp = ros::Time::now();
		position.header.seq += 1;
		position.pos.x = air_pos[0];
		position.pos.y = air_pos[1];
		position.pos.z = air_pos[2];
        cout<<"---------------------------------"<<endl;
        cout<<air_pos[0]<<"    "<<air_pos[1]<<"    "<<air_pos[2]<<endl;
		position_pub.publish(position);

		ros::spinOnce();
		rate.sleep();
	}

	fclose(fp);
	fclose(fp1);
    fclose(fp2);
    fclose(fp3);
    fclose(fp4);
    fclose(fp5);
    fclose(fp6);
	//cvReleaseMemStorage(&storage);

	//cvDestroyAllWindows();

	cvReleaseImage(&pImg);
	//cvReleaseVideoWriter(&writer);

	cvReleaseMat(&R_g_cz);
	cvReleaseMat(&R_cz_b);
	cvReleaseMat(&R_g_b);
	cvReleaseMat(&R_cz_g);
	cvReleaseMat(&R_b_ca);
	cvReleaseMat(&R_ca_b);
	cvReleaseMat(&R_chess_b);
	cvReleaseMat(&R_cz_ca);
    cvReleaseMat(&R_cz_b0);
    cvReleaseMat(&R_cz_b1);

	cvReleaseMat(&q_att);
	cvReleaseMat(&att);

	cvReleaseMat(&att_init);
	cvReleaseMat(&q_init);	


    cvReleaseMat(&velocity);
    cvReleaseMat(&accelerometer);

	cvReleaseMat(&sonar_ca);	
	cvReleaseMat(&sonar_cz);	
	cvReleaseMat(&pos_mat);
    cvReleaseMat(&tf_0);
    cvReleaseMat(&tf_1);

	cvReleaseMat(&T_inv);
	cvReleaseMat(&Q);
	//cvReleaseMat(&Q_inter);	
	cvReleaseMat(&Vec_pixel_A);
	cvReleaseMat(&Vec_pixel_B);
	cvReleaseMat(&Vec_physic_A);
	cvReleaseMat(&Vec_physic_B);

	return 0;
}
