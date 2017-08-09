/*
	FILE: 
	Author:hy
	Description:
*/
#include <iostream>
#include <iomanip>
#include <string>
 
#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "iarc007/hy_target/target.h"
#include "iarc007/jrh_pilot/JRH_math.h"
  
#include <iarc007/object.h>
#include <iarc007/jrh_model_choose.h>
#include <iarc007/vehicle_pos.h>
#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/dji_drone.h>
#include <iarc007/hy_target/angleID.h>

int WIDTH=640;
int HEIGHT=480;

int Frame_All;

#define IMAGE_SIZE (WIDTH*HEIGHT)

using namespace cv;
using namespace std;

Mat img_sub;

float air_pos[3]={0.};

float yaw,roll,pitch;
double h;
long target_serial;
int mode;

FILE* fp_global;

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

CvMat* q_att; //4×1
CvMat* att; //3×1


ros::Subscriber imu_sub;
ros::Subscriber ultrasonic_sub;
ros::Subscriber attitude_quaternion_subscriber;
ros::Subscriber mono_video_sub;
ros::Subscriber jrh_model_choose;
//dji_sdk::AttitudeQuaternion attitude_quaternion;

/**/
void MODEL_CHOOSE_callback(const iarc007::jrh_model_choose& jrh_model)
{ 
	     printf("void MODEL_CHOOSE_callback(const iarc007::jrh_model_choose& jrh_model)\n");
	     printf("jrh_model=%d\t%d\n",jrh_model.target_num_choose,jrh_model.model_choose);
		 if(jrh_model.model_choose==3)
			{
				mode=TARGET_SEARCH;
			}
			else if(jrh_model.model_choose==1||jrh_model.model_choose==2||jrh_model.model_choose==0)
			{
				mode=TARGET_TRACKING;
			}
			else
			{
				mode=TARGET_NULL;
			}
		target_serial=jrh_model.target_num_choose;
}
/* Mono Video */
void mono_sub_callback(sensor_msgs::Image img_msg)
{ 
	cv_bridge::CvImagePtr img_bridge_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);       
	img_bridge_ptr->image.copyTo(img_sub);       
}

/* imu from guidance */
void imu_callback(const geometry_msgs::TransformStamped& g_imu)
{ 
	//printf( "hy--frame_id: %s stamp: %d\n", g_imu.header.frame_id.c_str(), g_imu.header.stamp.sec );
/*	printf( "hy--imu: [%f %f %f %f %f %f %f]\n", g_imu.transform.translation.x,g_imu.transform.translation.y,\
	g_imu.transform.translation.z,g_imu.transform.rotation.x, g_imu.transform.rotation.y, g_imu.transform.rotation.z,\
 	g_imu.transform.rotation.w );
*/
}
/* imu from aerial pilot*/

FILE *yaws_txt=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/yaws.txt","w");

void attitude_quaternion_subscriber_callback(dji_sdk::AttitudeQuaternion attitude_quaternion)
{
	attitude_quaternion = attitude_quaternion;

	cvmSet(q_att,0,0,attitude_quaternion.q0);
	cvmSet(q_att,1,0,attitude_quaternion.q1);
	cvmSet(q_att,2,0,attitude_quaternion.q2);
	cvmSet(q_att,3,0,attitude_quaternion.q3);
	//printf("aerial pilot : %f %f %f %f\n",attitude_quaternion.q0,attitude_quaternion.q1,attitude_quaternion.q2,attitude_quaternion.q3);
	Quaternion_To_Euler(q_att, att);

	roll = cvmGet(att,0,0);
	pitch = cvmGet(att,1,0);
	yaw = cvmGet(att,2,0);
	printf(" %f  %f \n",ros::Time::now().sec+ros::Time::now().nsec*(1e-9),yaw*180/PI);

  fprintf(yaws_txt," %f  %f \n",ros::Time::now().sec+ros::Time::now().nsec*(1e-9),yaw*180/PI);

	/*	R_g_b coming from aerial pilot indicates transformation from ground to body (All observe left-multiply principle)	*/
	Euler_To_Matrix(roll,pitch,yaw,R_g_b);

	/*	R_g_cz coming from initial calibration indicates tramsformation from ground to competition zone(All observe left-multiply principle)*/
	cvTranspose(R_g_cz,R_cz_g);
        /*	R_cz_b indicates transformation from competition zone to body(All observe left-multiply principle)	*/                                   
	cvGEMM(R_cz_g,R_g_b,1,NULL,0,R_cz_b);
}

/* ultrasonic */
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{ 
	//printf("hy---frame_id: %s stamp: %d\n", g_ul.header.frame_id.c_str(), g_ul.header.stamp.sec );
	/*for (int i = 0; i < 5; i++){
		printf("hy---ultrasonic distance: [%f]  reliability: [%d]\n", g_ul.ranges[i], (int)g_ul.intensities[i] );
	}*/

	if(g_ul.ranges[0] > 0.)
	{
		h = g_ul.ranges[0];
	}
	else
	{
		//h=h;
	}
	//printf("--------%f\n",h-Low_Pass(h));
	//h=Low_Pass(h);
	//printf("----- h = %f\n",h);
}

void vehicle_pos_callback(const iarc007::vehicle_pos& wny)
{
	air_pos[0] = wny.pos.x*100.;
	air_pos[1] = wny.pos.y*100.;
	air_pos[2] = wny.pos.z*100.;

	//cout << "--------------------------------" << endl;

	//cout << setw(20) << setiosflags(ios::left) << "x" << setw(20) << "y" << endl;
	//cout << setw(20) << setiosflags(ios::left) << air_pos[0] << setw(20) << air_pos[1] << endl; 
}

int main(int argc, char** argv)
{

	ros::init(argc,argv,"mono_video_sub");
	ros::NodeHandle my_node;
	DJIDrone* drone = new DJIDrone(my_node);
	double prex=0,prey=0;
	FILE *jrh_q=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/JRH_pilot/data/Q_INIT.txt","r");
	if(!jrh_q)
	{
		std::cout << "File open failed !" << std::endl;
		return 0;
	}	

	fp_global = fopen("/home/hitcsc/catkin_ws/src/iarc007/log/irobot_position.txt","w");
	if(!fp_global)
	{
		cout << "ERROR: irobot_position.txt not open !" << endl;
	} 

	double time;
	float h_get;
	float q0,q1,q2,q3;
	double pre_angle=0.785;
	long target_serial_num=-1;
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
	Euler_To_Matrix(cvmGet(att_init,0,0),cvmGet(att_init,1,0),cvmGet(att_init,2,0),R_g_cz);
	
	/* Topic: mono_video -- Type: sensor_msgs::Image */
	mono_video_sub = my_node.subscribe<sensor_msgs::Image>("/iarc007/mono_video",2,mono_sub_callback);
	imu_sub = my_node.subscribe("/guidance/imu", 1, imu_callback);
	ultrasonic_sub = my_node.subscribe("/guidance/ultrasonic", 1, ultrasonic_callback);
	attitude_quaternion_subscriber = my_node.subscribe<dji_sdk::AttitudeQuaternion>("dji_sdk/attitude_quaternion", 1, attitude_quaternion_subscriber_callback);
	ros::Subscriber vehicle_pos_sub = my_node.subscribe("/iarc007/vehicle_pos", 1, vehicle_pos_callback);
	jrh_model_choose= my_node.subscribe("/jrh_model_choose", 10, MODEL_CHOOSE_callback);

	ros::Publisher object_pub = my_node.advertise<iarc007::object>("hy_object",10);
	ros::Publisher detect_pub = my_node.advertise<sensor_msgs::Image>("/iarc007/mono_video2",5);
	/* Loop for mono_sub_callback */    
    ros::Rate rate(30);

	IplImage* img = cvCreateImage(cvSize(WIDTH,HEIGHT),8,3);
	PosPoint point ={ 0 };

	iarc007::object obj;
	obj.target_x.resize(20);
	obj.serial_flag.resize(20);
	obj.target_y.resize(20);
	obj.target_dir.resize(20);
	obj.target_vx.resize(20);
	obj.target_vy.resize(20);
	
	R_cz_g=cvCreateMat(3,3,CV_32FC1);
	/*	extern parameter of cam(indicates transformation from camera to cheessboard(All observe left-multiply principle))	*/
	CvMat* R_ca_chess=(CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/Rotation.xml");
	/*	R_ca_b indicates transformation from camera to body(All observe left-multiply principle) 	*/
	CvMat *R_ca_b=cvCreateMat(3,3,CV_32FC1);
	/*	R_chess_b indicates transformation from chessboard to body(All observe left-multiply principle) 	*/
	CvMat *R_chess_b=cvCreateMat(3,3,CV_32FC1);
	
	/*
	cvZero(R_chess_b);
	cvmSet(R_chess_b,0,0,1.0);
	cvmSet(R_chess_b,1,1,-1.0);
	cvmSet(R_chess_b,2,2,-1.0);
	CvMat *R_chess_cz=cvCreateMat(3,3,CV_32FC1);
	cvZero(R_chess_cz);
	cvmSet(R_chess_cz,0,1,1.0);
	cvmSet(R_chess_cz,1,0,1.0);
	cvmSet(R_chess_cz,2,2,-1.0);
*/
	R_chess_b=(CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/R_chess_b.xml");

	cvGEMM(R_ca_chess,R_chess_b,1,NULL,0,R_ca_b);
	/*	R_cz_ca indicates transformation from competion zone to camera(ALl observe left-multiply principle)	*/
	CvMat* R_cz_ca=cvCreateMat(3,3,CV_32FC1);
	CvMat* R_ca_cz=cvCreateMat(3,3,CV_32FC1);
	CvMat* R_b_ca=cvCreateMat(3,3,CV_32FC1);

	R_cz_b = cvCreateMat(3,3,CV_32FC1);
	R_g_b = cvCreateMat(3,3,CV_32FC1);

	q_att = cvCreateMat(4,1,CV_32FC1);
	att = cvCreateMat(3,1,CV_32FC1);

   	cout << "hy_opencv_sub: start" << endl;
	FILE* fp=fopen("/home/hitcsc/catkin_ws/src/iarc007/src/HY_target/1.txt","w");

	CamShift_Init();

	//cvNamedWindow("camshift");
	//cvNamedWindow("img");
	//namedWindow("srcdet");
	//cvNamedWindow("dst");
	//cvNamedWindow("region");
	Frame_All=0;
	Vec_Init();
	Hog_Init();

	char str[200] = "/home/hitcsc/catkin_ws/src/iarc007/doc/SVMXML/SVM3/SVM_Data3.xml";
	SVM_Load(&mySVM, str);

	angleID angle;
	Point2f point2angle;
/*
	while(1)
	{
		printf("power1: %f\n",drone->attitude_quaternion.q0);
		printf("power2: %f\n",drone->attitude_quaternion.q1);
		printf("power3: %f\n",drone->attitude_quaternion.q2);
		printf("power4: %f\n",drone->attitude_quaternion.q3);
	}
*/
	VideoWriter writer("/home/hitcsc/catkin_ws/log/iarc/video/VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10.0, Size(640, 480));


	//*********************************
	long serial_pre[MAX_NUM]={0};
	double vx[MAX_NUM]={0.};
	double vy[MAX_NUM]={0.};
	double x_pre[MAX_NUM]={0.};
	double y_pre[MAX_NUM]={0.};
	int pre_num=0; 
	//*************************************
	double pre_time = ros::Time::now().sec+ros::Time::now().nsec*(1e-9);
	double now_time = ros::Time::now().sec+ros::Time::now().nsec*(1e-9);
	while(ros::ok())
	{
		cout << "*****Start! Frame_All = [ " << Frame_All << " ]*****" << endl;
		
		float t1 = ros::Time::now().nsec;
		if(img_sub.data)//确定有图像
		{
			imshow("mono_video_sub",img_sub);
			*img = IplImage(img_sub);

			char keycode=waitKey(1);
			if(keycode == 27)
			{
				break;
			} 
			h_get=h*100;
			h_get=h_get+1.0;
			if(h_get<10)
			{
				h_get=5;
			}


			cout << setw(10) << setiosflags(ios::left) << "height" << setw(20) << h_get << endl;

			cvTranspose(R_ca_b,R_b_ca);
			/*	R_cz_b indicates transformation from camera to body(All observe left-multiply principle) 	*/
			/*	R_cz_ca indicates transformation from competion zone to camera(ALl observe left-multiply principle)	*/
			cvGEMM(R_cz_b,R_b_ca,1,NULL,0,R_cz_ca);
			cvTranspose(R_cz_ca,R_ca_cz);
/*
			printf("R_cz_b = \n");
			for(int i=0;i<3;i++)
			{
				for(int j=0;j<3;j++)
				{
					printf("%f\t",cvmGet(R_cz_b,i,j));
				
					if(i==j)
					{
						//cvmSet(R_ca_cz,i,j,1);
					}
					else
					{
						//cvmSet(R_ca_cz,i,j,0);
					}

				}
				printf("\n");
			}
			*/
			//cvGEMM(R_chess_cz,R_cz_b,1,NULL,0,R_chess_b);
			//cvSave("/home/hitcsc/catkin_ws/src/iarc007/doc/R_chess_b.xml",R_chess_b);
			/* image processing (core function)*/
			//printf("11111111111111111111111111111111  %f \n",ros::Time::now().sec+ros::Time::now().nsec*(1e-9));
			time=ros::Time::now().sec+ros::Time::now().nsec*(1e-9);
			
			target_serial_num=target_serial;
			target(img,R_ca_cz,h_get,&point,mode,img,air_pos,target_serial_num);
			//printf("22222222222222222  %f \n",ros::Time::now().sec+ros::Time::now().nsec*(1e-9)-time);
		//printf("a\n");
			if(srcimgdraw.data)
			{
				writer << srcimgdraw;
			}
			//// LEO
			cv_bridge::CvImage leo_bridge;
			srcimgdraw.copyTo(leo_bridge.image);
			leo_bridge.header.frame_id = "makerd_tar";
			leo_bridge.header.stamp = ros::Time::now();
			leo_bridge.encoding = sensor_msgs::image_encodings::BGR8;
			detect_pub.publish(leo_bridge.toImageMsg());
			//// LEO
		}

		//printf("number=% d\n",point.num);
		for(int i=0;i<10;i++)
		{
			obj.target_vx[i]=0;
			obj.target_vy[i]=0;
		}
		fprintf(fp,"time %f\tnum=%d\tx %f\ty %f\tangle= %f\th=%f\t%d\n",ros::Time::now().sec+ros::Time::now().nsec*(1e-9),point.num,point.x[0],point.y[0],point.angle[0],h_get,mode);
		obj.target_num =point.num;
		
		if(obj.target_num > 0){			
			fprintf(fp_global,"air_x= %f\tair_y= %f\tirobot_X= %f\tirobot_x= %f\tirobot_Y= %f\tirobot_y= %f\tirobot_angle= %f\n",air_pos[0],air_pos[1],point2angle.x,obj.target_x[0],point2angle.y,obj.target_y[0],obj.target_dir[0]);
		}

		for(int i = 0; i < (int)obj.target_num; i++)
		{
			for(int j=0;j<pre_num;j++)
			{
				cout<<"sjkdfjadfasjdlfjasjkdf"<<endl;
				if(serial_pre[j]==point.serial_num[i])
				{
					now_time = ros::Time::now().sec+ros::Time::now().nsec*(1e-9);
					vx[i]=(point.x[i]-x_pre[j])/(now_time-pre_time);
					vy[i]=(point.y[i]-y_pre[j])/(now_time-pre_time);
					cout<< "*********************************************************";
					cout << i <<" "<< j<<endl;
					cout << "time "<< (now_time-pre_time)<<endl;
					cout << "time "<< now_time<<" "<<pre_time<<endl;
					obj.target_vx[i]=vx[i];
					obj.target_vy[i]=vy[i];
				}
			}
		}
		
		pre_num=(int)obj.target_num;
		for(int i = 0; i < (int)obj.target_num; i++)
		{
			cout << "i = " << i << "\t";
			cout << "obj.target_num = " << (int)obj.target_num << endl;
			obj.target_x[i] = (float)(point.x[i]);
			obj.target_y[i] = (float)(point.y[i]);
			obj.target_dir[i]=(float)(point.angle[i]);
			obj.serial_flag[i]=point.serial_num[i];

			x_pre[i] = (float)(point.x[i]);
			y_pre[i] = (float)(point.y[i]);
			pre_time=ros::Time::now().sec+ros::Time::now().nsec*(1e-9);
			
			serial_pre[i]=point.serial_num[i];

			
			cout << setw(10) << setiosflags(ios::left) << point.serial_num[i] << setw(10) << obj.target_x[i] << setw(10) << obj.target_y[i] << setw(20) << obj.target_dir[i]/3.14*180 << setw(20) << (int)obj.target_num << endl;
		}
		//obj.obstacle_flag=false;//todo
		
		obj.header.stamp=ros::Time::now();
		//printf("point=%f\ty=%f\n",point.x[0],point.y[0]);
		//printf("targetx=%f\ty=%f\tangle=%fnum=%d\n",obj.target_x[0],obj.target_y[0],obj.target_dir[0]/3.14*180,obj.target_num);
		cout << setw(10) << setiosflags(ios::left) << "num" << setw(10) << "x" << setw(10) << "y" << setw(20) << "direction" << setw(20) << "number"<< endl;
		cout << setw(10) << setiosflags(ios::left) << 1 << setw(10) << obj.target_x[0] << setw(10) << obj.target_y[0] << setw(20) << obj.target_dir[0]/3.14*180 << setw(20) << (int)obj.target_num << endl; 
	//obj.target_z[0] = (float)point.z[0];
 		object_pub.publish(obj);
		ros::spinOnce();
		rate.sleep();
		float t2 = ros::Time::now().nsec;
		//printf("time --- %f\t,hz=%f\n",(t2-t1)/1e9,1/((t2-t1)/1e9));

		cout << setw(20) << setiosflags(ios::left) << "period" << setw(20) << "hz" << endl;
		cout << setw(20) << setiosflags(ios::left) << (t2-t1)/1e9 << setw(20) << 1/((t2-t1)/1e9) << endl;
		cout << "******End! Frame_All = [ " << Frame_All << " ]******" << endl;
		Frame_All++;
		//cvZero(img);
	}//while(ros::ok())主循环
   	cout << "hy_opencv_sub: stop" << endl;

	fclose(fp);
	cvReleaseImage(&img);
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
	cvDestroyAllWindows();


	return 0;
}
