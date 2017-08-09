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

#include <iarc007/vehicle_pos.h>
#include "iarc007/jrh_pilot/JRH_math.h"

using namespace cv;
using namespace std;

int WIDTH=640;
int HEIGHT=480;

#define PI 3.1415926
#define IMAGE_SIZE (WIDTH*HEIGHT)

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

CvMat* q_att; //4×1
CvMat* att; //3×1

//CvMat* velocity; //3×1
CvMat* pos_mat; //3*1
CvMat* sonar_ca;
CvMat* sonar_cz;

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
float pilot_pos[3] = {0.};
/*	attitude measured by imu	*/
float roll = 0., pitch = 0., yaw = 0.;
/*	update	period */
float t = 0.1;
/* error of yaw angle */
float e_xy = 0.;
/* TIME */  
double t_img = 0., t_gui = 0.;

/* Mono Video */
FILE* fp = fopen("/home/hitcsc/catkin_ws/src/iarc007/log/vision_position.txt","w");
FILE* fp1 = fopen("/home/hitcsc/catkin_ws/src/iarc007/log/mjh_time_delay.txt","w");
FILE* fp2 = fopen("/home/hitcsc/catkin_ws/src/iarc007/log/globalwatch_position.txt","w");

void mono_sub_callback(sensor_msgs::Image img_msg)
{  
	cv_bridge::CvImagePtr img_bridge_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);       
	img_bridge_ptr->image.copyTo(img_src);   
	
	t_img = img_msg.header.stamp.toSec();
	//cout << "time of img" << setprecision(9) << setiosflags(ios::fixed) << img_msg.header.stamp.toSec() << endl;    
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

	/*	R_g_cz coming from initial calibration indicates tramsformation from ground to competition zone(All observe left-multiply principle)*/
	//cvTranspose(R_g_cz,R_cz_g);
        /*	R_cz_b indicates transformation from competition zone to body(All observe left-multiply principle)	*/                                   
	cvGEMM(R_cz_g,R_g_b,1,NULL,0,R_cz_b);
	//roll,pitch,yaw
}

/* velocity */
void velocity_callback(const geometry_msgs::Vector3Stamped& g_vo)
{ 
/*
	printf("mjh--frame_id: %s stamp: %d\n", g_vo.header.frame_id.c_str(), g_vo.header.stamp.sec );
	printf("mjh--velocity: [%f %f %f]\n", g_vo.vector.x, g_vo.vector.y, g_vo.vector.z );
*/
	
/*	v_x = g_vo.vector.x;
	v_y = g_vo.vector.y;
	v_z = g_vo.vector.z;
*/
/*
	cvmSet(velocity, 0, 0, g_vo.vector.x);
	cvmSet(velocity, 1, 0, g_vo.vector.y);
	cvmSet(velocity, 2, 0, g_vo.vector.z);

	cvGEMM(R_cz_g,velocity,1,NULL,0,velocity);

	v_x = cvmGet(velocity, 0, 0);
	v_y = cvmGet(velocity, 1, 0);
	v_z = cvmGet(velocity, 2, 0);

*/	
/*	float tmp_vx,tmp_vy;
	tmp_vx = v_x;
	tmp_vy = v_y;
        v_x = tmp_vx*cos(e_xy) - tmp_vy*sin(e_xy);
	v_y = tmp_vx*sin(e_xy) + tmp_vy*cos(e_xy);
*/
}

/* ultrasonic */
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
	air_pos[2] = g_ul.ranges[0]-0.02;// + cvmGet(sonar_cz,2,0);
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

	air_pos[0] += p_x;
	air_pos[1] += p_y;
	air_pos[2] += p_z;

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

	/* Loop for mono_sub_callback */
	ros::Publisher position_pub = my_node.advertise<iarc007::vehicle_pos>("/iarc007/vehicle_pos",10);

	iarc007::vehicle_pos position;
	position.header.seq = 0;
	position.header.frame_id = "iarc007";

	ros::Rate rate(10);

	R_cz_g=cvCreateMat(3,3,CV_32FC1);
	cvTranspose(R_g_cz,R_cz_g);
	//velocity=cvCreateMat(3,1,CV_32FC1);
	pos_mat = cvCreateMat(3,1,CV_32FC1);

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
	R_g_b = cvCreateMat(3,3,CV_32FC1);

	q_att = cvCreateMat(4,1,CV_32FC1);
	att = cvCreateMat(3,1,CV_32FC1);

	sonar_ca=cvCreateMat(3,1,CV_32FC1);
	
	for(int i = 0;i < 3;i++){
		sonar_ca->data.fl[i] = sonar_in_ca[i];		
	}

	sonar_cz=cvCreateMat(3,1,CV_32FC1);

	IplImage* img = cvCreateImage(cvSize(WIDTH,HEIGHT),8,3);
	IplImage* pImg = cvCreateImage(cvSize(WIDTH,HEIGHT),8,1);
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

	//cout << setw(16) << setiosflags(ios::left);



	float drho_x = 0.,drho_y = 0.;

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
				cvCvtColor (img,pImg,CV_BGR2GRAY);
	
				cv::Mat src = Mat(pImg,false);

				//将原图像转换为二值图像  
				cv::threshold(src, src, 150, 1, cv::THRESH_BINARY);  

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
			
				HoughLines( dst, lines, 1, CV_PI/180, 120 );

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
			if(flag)
			{
				cout << "-------air_pos------" << endl;
				for(int i = 0;i < 3;i++){
					cout << air_pos[i] << ' ';
					if(!((i+1)%3))
						cout << endl;
				}

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
					Vec_physic_A->data.fl[0] += air_pos[0];
					Vec_physic_A->data.fl[1] /= s_A;
					Vec_physic_A->data.fl[1] += air_pos[1];

					Vec_physic_B->data.fl[0] /= s_B;
					Vec_physic_B->data.fl[0] += air_pos[0];
					Vec_physic_B->data.fl[1] /= s_B;
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

					lines[i][1] = lines[i][1] + e_xy;
					cout << "lines[i][1]" << lines[i][1] << endl;
					cout << "lines[i][0]" << lines[i][0] << endl;
 
					if (lines[i][0] >= 0.) {

						if (fabs(lines[i][1] - PI / 2.) < PI / 18.) {

							lines[i][1] = PI / 2.;
							lines_flag[i] = 1;
						}
						else if (fabs(lines[i][1] + PI / 2.) < PI / 18.) {

							lines[i][1] = -PI / 2.;
							lines_flag[i] = 1;
						}
						else if (fabs(lines[i][1]) < PI / 18.) {

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

								lines[i][1] = -PI/2.;
								lines_flag[i] = 1;
							}
							else if (fabs(lines[i][1] - PI + PI) < PI / 18.) {

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

								lines[i][1] = PI/2.;
								lines_flag[i] = 1;
							}
							else if (fabs(lines[i][1] + PI -PI) < PI / 18.) {

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
					//cout << "lines[i][1]--------------" << lines[i][1] << endl;
					//cout << "lines[i][0]--------------" << lines[i][0] << endl;
					/*else
					{
						lines[i][0] = 0.;
						lines[i][1] = 0.;
						lines_flag[i] = 0;
					}*/
				}

				int sum_x = 0,sum_y = 0;
				
				float tmp_rho_x = 0.1,tmp_rho_y = 0.15;//limit rho
				sum_x = 0,sum_y = 0,drho_x = 0.,drho_y = 0.;
				

				for(int i = 0;i < lines.size();i++){  
					if(lines_flag[i] && fabs(cvRound(lines[i][0]) - lines[i][0]) < tmp_rho_y){
						if(fabs(lines[i][1]) < 0.1 || fabs(lines[i][1]-PI/2.) < 0.1){
							lines[i][0] -= cvRound(lines[i][0]);
							//cout << "lines[i][0]a" << lines[i][0] << endl;
								
							
							if(fabs(lines[i][1]) < 0.1 && fabs(lines[i][0]) < tmp_rho_x){
								drho_x += lines[i][0];
								sum_x += 1;
							}
							else{
								drho_y += lines[i][0];
								sum_y += 1;
							}
							//cout << "sum_x = " << sum_x << endl;
						}
						else
						{
							lines[i][0] = cvRound(lines[i][0]) - lines[i][0];
							//cout << "lines[i][0]b" << lines[i][0] << endl;
							
							if(fabs((fabs(lines[i][1]) - PI) < 0.1) && fabs(lines[i][0]) < tmp_rho_x){
								drho_x += lines[i][0];
								sum_x += 1;
							}
							else{ 
								drho_y += lines[i][0];
								sum_y += 1;
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
                                
                                //cout << "sum_x " << sum_x << " sum_y " << sum_y << " drho_x " << drho_x << " drho_y " << drho_y << endl;
				//cout << "v_x*t" << v_x*t << "v_y*t" << v_y*t << endl;
		
	//v_x = tmp_vx*cos(e_xy) - tmp_vy*sin(e_xy);
	//v_y = tmp_vx*sin(e_xy) + tmp_vy*cos(e_xy);
						
				air_pos[0] -= drho_x*cos(e_xy) - drho_y*sin(e_xy);
				air_pos[1] -= drho_x*sin(e_xy) + drho_y*cos(e_xy);
				
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

		position_pub.publish(position);

		ros::spinOnce();
		rate.sleep();
	}

	fclose(fp);
	fclose(fp1);
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

	cvReleaseMat(&q_att);
	cvReleaseMat(&att);

	cvReleaseMat(&att_init);
	cvReleaseMat(&q_init);	

	cvReleaseMat(&sonar_ca);	
	cvReleaseMat(&sonar_cz);	
	cvReleaseMat(&pos_mat);

	cvReleaseMat(&T_inv);
	cvReleaseMat(&Q);
	//cvReleaseMat(&Q_inter);	
	cvReleaseMat(&Vec_pixel_A);
	cvReleaseMat(&Vec_pixel_B);
	cvReleaseMat(&Vec_physic_A);
	cvReleaseMat(&Vec_physic_B);

	return 0;
}
