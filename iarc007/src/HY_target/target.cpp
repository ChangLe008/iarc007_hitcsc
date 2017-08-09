#include "iarc007/hy_target/target.h"
#include <ros/ros.h>
#include <fstream>

using namespace AprilTags;
#define LOWSPACE 80//80
#define HIGHSPACE 300//120
Direction_Mode pre_Direction_mode=DIRECTION_LOW;
Track_Mode pre_Track_mode=TRACK_LOW;
int track_num=0;
int miss_target=0;

static int Obj_Num_LF = 0;

void target(IplImage* img,CvMat* R,double h,PosPoint* point,int mode,IplImage* dst,float air_pos[],long s_num)
{
	IplImage* img_256 = cvCreateImage(cvSize(256,256),8,3);
	CvPoint Endpoint[2] = { 0 };
	CvMat* P=cvCreateMat(3,1,CV_32FC1);
	CvMat* intrinsic=(CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/Intrinsics.xml");
	CvMat* distortion=(CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc007/doc/Distortion.xml");
	IplImage* img_undistortion=cvCreateImage(cvGetSize(img),8,3);
	IplImage* mapx=cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,1);
	IplImage* mapy=cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,1);
	vector<Rect> rect;
	CvPoint2D32f  p={0.};
	CvPoint2D32f  p_start={0.};
	CvPoint2D32f  p_end={0.};
	PicPoint  picp={0};
	PosPoint posp={0};
	CvPoint ret={0};
 
	Track_Mode n_Track_mode;

	float angle=0;
	posp.num=0;
	posp.x[0]=0;
	posp.x[0]=0;
	int dir_flag=0;
	target_description H_target;
	for(int i=0;i<10;i++)
	{
		point->angle[i]=10.0;
	}
	if(img)
	{
		Mat srcimg(img);

		srcimg.copyTo(srcimgdraw);
		ellipse(srcimgdraw, ELL_CEN,Size(ELL_A,ELL_B),0., 0.,360. ,Scalar(255,0,255),2);
		circle(srcimgdraw,ELL_CEN,3,Scalar(255,0,255),-1);
		cvGEMM(intrinsic,R,1,NULL,0,R);
		cv::Rect Rect;
		if(robot_sta_all.target_count_now == 0)
		//if(mode == TARGET_SEARCH)
		{
			mode = TARGET_SEARCH ;
		}
		else if(robot_sta_all.target_count_now != 0)
		//else if(mode == TARGET_TRACKING)
		{
			mode = TARGET_TRACKING;
			s_num=robot_sta_all.robot_sta_single[0].Serial_Num;
		}
		
		if( mode == TARGET_SEARCH )//针对控制的搜索阶段  （此时没有确定目标）mode检测、跟踪、不执行
		{
			printf("检测\n");
			if(h > 80 && h < 250)
			{
				pre_Track_mode = TRACK_HIGH;
				if(robot_sta_all.target_count_now == 0)  //1.检测HOG+SVM
				{
					track_num=0;
					cout << "HOGDetection\n";
					
					Multi_Scale_Obj_Dec(h, srcimg, myHOG, { 0, 0 }, WIN, Size(16, 16), 1.1, 2, 1, 1);
					cout << "Target number: " << Red_Out.size() + Green_Out.size() << endl;	//Obj_Out.size()

					for(int i = 0; i < Obj_Out.size(); i++)
					{
						Mat Obj_ROI;
						srcimg(Obj_Out[i]).copyTo(Obj_ROI);
						if(Obj_Num_LF == 65535)
						{
							cout << "哈哈哈哈哈哈哈哈哈哈哈哈哈哈哈哈" << endl;
							break;
						}
						char  filename[100];
						sprintf (filename, "/home/hitcsc/save2/%d.jpg", Obj_Num_LF);

						imwrite(filename, Obj_ROI);

						Obj_Num_LF++;
					}
					Dec_Tra_Match(&robot_sta_all  , Red_Out  , Green_Out);
					for (int i = 0; i < Barrier_Out.size(); i++)
					{
						//barriercount++;
						cout << "This target is barrier!" << endl;
						rectangle(srcimgdraw, Barrier_Out[i], Scalar(255, 255, 255), 2);
						int xx1 = Barrier_Out[i].tl().x;
						int xx2 = Barrier_Out[i].br().x;
						int yy1 = Barrier_Out[i].tl().y;
						int yy2 = Barrier_Out[i].br().y;
						circle(srcimgdraw, Point((xx1 + xx2) / 2, (yy1 + yy2) / 2), 3, Scalar(255, 255, 255), -1);
					}
					
					cout << "In DECHOG robot_sta_all.target_count_now = " << robot_sta_all.target_count_now << endl;
					
					Vec_Init();
				}
				else if(robot_sta_all.target_count_now != 0)//2.多目标跟踪
				{
					if(track_num % 20 == 0 && track_num != 0)//跟踪阶段，每n帧检测一次
					{
						track_num=0;
						Multi_Scale_Obj_Dec(h, srcimg, myHOG, { 0, 0 }, WIN, Size(16, 16), 1.1, 2, 1, 1);
						cout << "Target number: " << Red_Out.size() + Green_Out.size() << endl;	//Obj_Out.size()

						for(int i = 0; i < Obj_Out.size(); i++)
						{
							Mat Obj_ROI;
							if(Obj_Num_LF == 65535)
							{
								cout << "哈哈哈哈哈哈哈哈哈哈哈哈哈哈哈哈" << endl;
								break;
							}
							srcimg(Obj_Out[i]).copyTo(Obj_ROI);
							char  filename[100];
							sprintf (filename, "/home/hitcsc/save2/%d.jpg", Obj_Num_LF);

							imwrite(filename, Obj_ROI);

							Obj_Num_LF++;
						}



						Dec_Tra_Match(&robot_sta_all  , Red_Out  , Green_Out);
						for (int i = 0; i < Barrier_Out.size(); i++)
						{
							//barriercount++;
							cout << "This target is barrier!" << endl;
							rectangle(srcimgdraw, Barrier_Out[i], Scalar(255, 255, 255), 2);
							int xx1 = Barrier_Out[i].tl().x;
							int xx2 = Barrier_Out[i].br().x;
							int yy1 = Barrier_Out[i].tl().y;
							int yy2 = Barrier_Out[i].br().y;
							circle(srcimgdraw, Point((xx1 + xx2) / 2, (yy1 + yy2) / 2), 3, Scalar(255, 255, 255), -1);
						}
						cout << "In DECHOG2 robot_sta_all.target_count_now = " << robot_sta_all.target_count_now << endl;						
						Vec_Init();
					}
					else
					{
						track_num++;
						All_Obj_Tra(srcimg,h);
						Refresh(h);
						dir_flag=1;

					}
					//方向
					for(int i=0;i<robot_sta_all.target_count_now;i++)
					{
						ExtendCV::High_Target(backproject, robot_sta_all.robot_sta_single[i].Box2D_Now,H_target);
						if(H_target.target_flag == true)
						{
							point->angle[i]= find_direction(H_target,intrinsic, distortion, R, h);
						}
						else
						{
							point->angle[i]=10.0;
						}
					}
				}
			}//if(h > 80 && h < 250)
			else//3.不处理
			{
				printf("search 模式下高度不对\n");
				pre_Track_mode = TRACK_LOW;
			}
				
		}
		else if(mode == TARGET_TRACKING)//已经制定好目标，跟上一过程类似，没有检测只有跟踪
		{
			printf("跟踪\n");
			n_Track_mode= huizhi(h);
			if(n_Track_mode == TRACK_HIGH)
			{
				int flag=-1;

				for(int i=0;i<robot_sta_all.target_count_now;i++)
				{
					if(robot_sta_all.robot_sta_single[i].Serial_Num==s_num)
					{
						flag=i;
					}
				}
				if(flag ==-1)
				{
					robot_sta_all.target_count_now=0;
					
				}
				else
				{
					printf("flag=%d\n",flag);
					Robot_Sta_Copy(&robot_sta_all.robot_sta_single[flag], &robot_sta_all.robot_sta_single[0]);	//结合策略
					
					int xx1 = robot_sta_all.robot_sta_single[0].Track_Now.tl().x;
					int xx2 = robot_sta_all.robot_sta_single[0].Track_Now.br().x;
					int yy1 = robot_sta_all.robot_sta_single[0].Track_Now.tl().y;
					int yy2 = robot_sta_all.robot_sta_single[0].Track_Now.br().y;
			
					cout << "xx11 = " << xx1 << endl;
					cout << "xx21 = " << xx2 << endl;
					cout << "yy11 = " << yy1 << endl;
					cout << "yy21 = " << yy2 << endl;
					
					int k = robot_sta_all.target_count_now;
					for(int i=1; i < k; i++)
					{
						 R_Sta_Reset(&(robot_sta_all.robot_sta_single[i]));
						 robot_sta_all.target_count_now--;
					}
					//robot_sta_all.target_count_now=1;
					All_Obj_Tra(srcimg, h);
					cout << "In All_Obj_Tra Total num = " << robot_sta_all.target_count_now << endl;
					Refresh(h);
					if(robot_sta_all.target_count_now!=0)
					{
						ExtendCV::High_Target(backproject, robot_sta_all.robot_sta_single[0].Box2D_Now,H_target);
						if(H_target.target_flag == true)
						{
							point->angle[0]= find_direction(H_target,intrinsic, distortion, R, h);
						}
						else
						{
							point->angle[0]=10.0;
						}
					}
					
				}

			}
			else if(n_Track_mode==TRACK_LOW)
			{
				if(robot_sta_all.target_count_now!=0)
				{
					ExtendCV::Low_Target(srcimg, H_target,robot_sta_all.robot_sta_single[0].logo_num);

					if(H_target.target_flag == true)
					{
						miss_target=0;
						robot_sta_all.robot_sta_single[0].Track_Now = H_target.roi_rect;
					
						robot_sta_all.target_count_now = 1;
						R_Mes_Up(&robot_sta_all.robot_sta_single[0]);
						point->angle[0]=find_direction( H_target, intrinsic, distortion, R, h);
					}
					else if(H_target.target_flag == false)
					{
						if(miss_target < 7)
						{
							miss_target++;
							cout << "hai TMD mei diu le!" << endl;
							robot_sta_all.target_count_now = 1;
							point->angle[0]=10.0;
						}
						else
						{
							cout << "zhen TMD diu le !" << endl;
							robot_sta_all.target_count_now = 0;
						}
					} 
					int xx1 = robot_sta_all.robot_sta_single[0].Track_Now.tl().x;
					int xx2 = robot_sta_all.robot_sta_single[0].Track_Now.br().x;
					int yy1 = robot_sta_all.robot_sta_single[0].Track_Now.tl().y;
					int yy2 = robot_sta_all.robot_sta_single[0].Track_Now.br().y;
		
					cout << "xx12 = " << xx1 << endl;
					cout << "xx22 = " << xx2 << endl;
					cout << "yy12 = " << yy1 << endl;
					cout << "yy22 = " << yy2 << endl;
				}
				else;
				
			}
			else;
		}
		else//初始化
		{
			printf("无操作\n");
			int k = robot_sta_all.target_count_now;
			for(int i=0; i<k; i++)
			{
				R_Sta_Reset(&(robot_sta_all.robot_sta_single[i]));
				robot_sta_all.target_count_now--;
			}			
		}

		for(int i=0; i < robot_sta_all.target_count_now; i++)   //根据图像位置计算实际位置
		{
			ret=DistortionPoint((robot_sta_all.robot_sta_single[i].Track_Now.tl().x+robot_sta_all.robot_sta_single[i].Track_Now.br().x)/2,(robot_sta_all.robot_sta_single[i].Track_Now.tl().y+robot_sta_all.robot_sta_single[i].Track_Now.br().y)/2,distortion, intrinsic);
			//printf("bbb=%d\t%d\n",ret.x,ret.y);
			dianbianhuan(R, P, h,ret.x,ret.y);
			point->x[i]=cvmGet(P,0,0);//结算的位置信息返回
			point->y[i]=cvmGet(P,1,0);
		}
		//目标点位置结算结束
		
		cout << "Start Writing serial_num" << endl;
		cout << "robot_sta_all.target_count_now = " << robot_sta_all.target_count_now << endl;
		for(int i=0; i < robot_sta_all.target_count_now; i++)
		{
			char serial[100] = "";
			sprintf(serial, "%d", robot_sta_all.robot_sta_single[i].Serial_Num);
			int xx = robot_sta_all.robot_sta_single[i].Track_Now.tl().x;
			int xx1 = robot_sta_all.robot_sta_single[i].Track_Now.br().x;
			int yy = robot_sta_all.robot_sta_single[i].Track_Now.tl().y;
			int yy1 = robot_sta_all.robot_sta_single[i].Track_Now.br().y;
			Letter_Show(serial, Point((xx + xx1)/2, (yy + yy1)/2), srcimgdraw);
		}
		cout << "Finish Writing serial_num" << endl;

		imshow("srcimgdraw",srcimgdraw);
		
		//目标编号赋值开始
		for(int i=0;i<robot_sta_all.target_count_now;i++)
		{
			point->serial_num[i]=robot_sta_all.robot_sta_single[i].Serial_Num;
		}
		//目标编号赋值结束

		//滤波开始
		float target_dir[10]={0.};
		for(int i = 0;i<robot_sta_all.target_count_now;i++)
		{
			robot_sta_all.robot_sta_single[i].point2angle.x = point->x[i];
			robot_sta_all.robot_sta_single[i].point2angle.y = point->y[i];
			robot_sta_all.robot_sta_single[i].angle.update(air_pos,robot_sta_all.robot_sta_single[i].point2angle,true);
			if(!robot_sta_all.robot_sta_single[i].angle.estimate())
			{
				target_dir[i]=(float)(point->angle[i]);
				//cout << "-----angle------: " << target_dir[0] << endl;
				//cout << "-----angle!-----: " << angle.angle << endl;
			}
			else
			{
				if(fabs(point->angle[i]-10.0) > 1.0)
				{
					float inv_angle;

					if(point->angle[i] > 0){
						inv_angle = point->angle[i] - PI;
					}
					else{
						inv_angle = point->angle[i] + PI;
					}

					if(abs(robot_sta_all.robot_sta_single[i].angle.angle-point->angle[i]) > abs(robot_sta_all.robot_sta_single[i].angle.angle-inv_angle)){
						target_dir[i]=inv_angle;

						cout << "--------------angle : " << robot_sta_all.robot_sta_single[i].angle.angle << ' ' << inv_angle << endl;
						robot_sta_all.robot_sta_single[i].angle.angle = inv_angle;
					}
					else{
						target_dir[i]=(float)(point->angle[i]);
						cout << "--------------angle : " << robot_sta_all.robot_sta_single[i].angle.angle << ' ' << point->angle[0] << endl;
						robot_sta_all.robot_sta_single[i].angle.angle = point->angle[i];
					}
				}
				else{
					target_dir[i]=robot_sta_all.robot_sta_single[i].angle.angle;
				}
			}

			float theta = target_dir[i];
			if(fabs(theta-10.0) > 1.0){
				if(robot_sta_all.robot_sta_single[i].angle.filter(robot_sta_all.robot_sta_single[i].point2angle,theta,true)){
					//obj.target_num = 1;
					point->x[i] = (float)robot_sta_all.robot_sta_single[i].point2angle.x;
					point->y[i] = (float)robot_sta_all.robot_sta_single[i].point2angle.y;
					point->angle[i] = theta;
				}
				else{
					;//obj.target_num = 0;//目标丢失
				}
			}
			else{
				if(robot_sta_all.robot_sta_single[i].angle.filter(robot_sta_all.robot_sta_single[i].point2angle,theta,false))
				{
					//obj.target_num = 1;
					point->x[i] = (float)robot_sta_all.robot_sta_single[i].point2angle.x;
					point->y[i] = (float)robot_sta_all.robot_sta_single[i].point2angle.y;
					point->angle[i] = theta;
				}
				else{
					;//obj.target_num = 0;//目标丢失
				}
			}
		}
		//滤波结束
		point->num=robot_sta_all.target_count_now;
	}
	cvShowImage("img",img);
	cvReleaseMat(&intrinsic);
	cvReleaseMat(&distortion);
	cvReleaseMat(&P);
	cvReleaseImage(&img_undistortion);
	cvReleaseImage(&mapx);
	cvReleaseImage(&mapy);
	cvReleaseImage(&img_256);
	return;
}

Track_Mode huizhi(float h)
{
	Track_Mode n_Track_mode;
	if(pre_Track_mode == TRACK_HIGH && h > LOWSPACE)
	{
		n_Track_mode = TRACK_HIGH;
		pre_Track_mode = n_Track_mode;
	}
	else if(pre_Track_mode == TRACK_HIGH && h < LOWSPACE)
	{
		n_Track_mode = TRACK_LOW;
		pre_Track_mode = n_Track_mode;
	}
	else if(pre_Track_mode == TRACK_LOW && h < LOWSPACE + 20)
	{
		n_Track_mode = TRACK_LOW;
		pre_Track_mode = n_Track_mode;
	}
	else if(pre_Track_mode == TRACK_LOW && h > LOWSPACE + 20)
	{
		n_Track_mode = TRACK_HIGH;
		pre_Track_mode = n_Track_mode;
	}
	return n_Track_mode;
}

float find_direction(target_description& H_target,CvMat* intrinsic,CvMat* distortion,CvMat* R,float h)
{
	CvMat *P = cvCreateMat(3,1,CV_32FC1);
	CvPoint Endpoint[2] = {0};
	float p_start_x,p_start_y,p_end_x,p_end_y;
	float angle;

	Endpoint[0].x=(int)(H_target.start_point.x);
	Endpoint[0].y=(int)(H_target.start_point.y);

	Endpoint[1].x=(int)(H_target.middle_point.x);
	Endpoint[1].y=(int)(H_target.middle_point.y);

	//cvLine(img,Endpoint[1],Endpoint[0],cvScalar(255,255,255),1,8,0);


	Endpoint[0]=DistortionPoint(Endpoint[0].x,Endpoint[0].y,distortion, intrinsic);
	dianbianhuan(R, P, h,Endpoint[0].x,Endpoint[0].y);
	p_start_x=cvmGet(P,0,0);
	p_start_y=cvmGet(P,1,0);

	Endpoint[1]=DistortionPoint(Endpoint[1].x,Endpoint[1].y,distortion, intrinsic);
	dianbianhuan(R, P, h,Endpoint[1].x,Endpoint[1].y);
	p_end_x=cvmGet(P,0,0);
	p_end_y=cvmGet(P,1,0);
	angle=atan2((p_start_y-p_end_y),(p_start_x-p_end_x));

	cvReleaseMat(&P);

	return angle;
}

