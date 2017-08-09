#include "iarc007/hy_target/Track.h"
#include <iarc007/hy_target/angleID.h>
#include "iarc007/hy_target/Filter_Statistic.h"
 
long Serial_Num=0;
Robot_Sta_All robot_sta_all = { 0 };
R_Watch r_watch = { 0 };
long Frame_Count = 0;
int Start_Flag = 0;
int backproject_mode = 0;
Mat Src_HSV, Src_Hue, mask, backproject, backprojectforerve, histimg, showimage, Hist_image;
Mat srcimgdraw;

float HYHYHY=0.0;
const int histSize = HIST_HDIMS;
float hranges_arr[2] = { 0, 180 };
const float* ranges[1] = { hranges_arr };

void R_Watch_Init(void)
{
	for (int i = 0; i < MAX_NUM; i++)
	{
		r_watch.R_Sta[i] = false;
		r_watch.R_Ch2Disap[i] = false;
		r_watch.R_Ch2Ap[i] = false;
	}
	r_watch.RC_Inc = false;
	r_watch.RC_Red = false;
}
void CamShift_Init(void)
{
	R_Watch_Init();

	Src_HSV.create(Size(IPL_WIDTH, IPL_HEIGHT), CV_8UC3);
	Src_Hue.create(Size(IPL_WIDTH, IPL_HEIGHT), CV_8UC1);
	mask.create(Size(IPL_WIDTH, IPL_HEIGHT), CV_8UC1);
	backproject.create(Size(IPL_WIDTH, IPL_HEIGHT), CV_8UC1);
	backprojectforerve.create(Size(IPL_WIDTH, IPL_HEIGHT), CV_8UC1);
	Hist_image.create(Size(400, 750), CV_8UC3);
	showimage.create(Size(400, 80), CV_8UC3);
	Src_HSV.setTo(0);
	Src_Hue.setTo(0);
	mask.setTo(0);
	backproject.setTo(0);
	Hist_image.setTo(0);
	showimage.setTo(0);
	for (int i = 0; i < MAX_NUM; i++)
	{
		robot_sta_all.robot_sta_single[i].robot_sta_now = ROBOT_NO;
		robot_sta_all.robot_sta_single[i].robot_sta_pre = ROBOT_NO;
		robot_sta_all.robot_sta_single[i].robot_sta_pre2 = ROBOT_NO;		
		robot_sta_all.robot_sta_single[i].Once_Start = 0;
		robot_sta_all.robot_sta_single[i].track_count = 0;
		robot_sta_all.robot_sta_single[i].logo_num = i;
		robot_sta_all.track[i] = i;
		robot_sta_all.robot_sta_single[i].hist.bhattacharyya = 1.0;
		robot_sta_all.robot_sta_single[i].hist.max_val = 0.;
		robot_sta_all.robot_sta_single[i].hist_now = Mat::zeros(Size(1, HIST_HDIMS), CV_32FC1);
		robot_sta_all.robot_sta_single[i].hist_pre = Mat::zeros(Size(1, HIST_HDIMS), CV_32FC1);
		robot_sta_all.robot_sta_single[i].hist_pre2 = Mat::zeros(Size(1, HIST_HDIMS), CV_32FC1);
		robot_sta_all.robot_sta_single[i].hist.hist = Mat::zeros(Size(1, HIST_HDIMS), CV_32FC1);
		robot_sta_all.robot_sta_single[i].logo_num = 3;
		robot_sta_all.robot_sta_single[i].Serial_Num = -1;
		robot_sta_all.robot_sta_single[i].angle.initialize();
	}
	robot_sta_all.target_count_now = 0;
	robot_sta_all.target_count_pre = 0;
	robot_sta_all.target_count_pre2 = 0;
	histimg.create(cvSize(320, 200), CV_8UC3);
	Start_Flag = 1;
}
void R_Sta_Reset(Robot_Sta_Single *robot)
{
	robot->Once_Start = 1;
	robot->robot_sta_now = ROBOT_NO;
	robot->robot_sta_pre = ROBOT_NO;
	robot->robot_sta_pre2 = ROBOT_NO;	
	robot->track_count = 0;
	robot->Serial_Num = -1;
	robot->angle.initialize();
}
void Hist_Up(Mat& hist1, Mat& hist2, float weight)
{
	float a1 = 0., a2 = 0.;
	float Array[HIST_HDIMS] = { 0. };
	for (int i = 0; i < HIST_HDIMS; i++)
	{
		a1 = hist1.at<float>(i);
		a2 = hist2.at<float>(i);
		Array[i] = a1*(1 - weight) + a2*weight;
		hist1.at<float>(i) = Array[i];
	}

	normalize(hist1, hist1, 0., 1., NORM_MINMAX, -1, Mat());

}
void Hist_Get(Hist_Store *hist, Mat img)
{
	rectangle(img, hist->Rec.tl(), hist->Rec.br(), Scalar(255, 0, 0), 1, 8, 0);
	cvtColor(img, img, CV_BGR2HSV);
	inRange(img, Scalar(0, SMIN, VMIN, 0), Scalar(180, 236, VMAX, 0), mask);

	vector<Mat>res;
	split(img, res);	//get H-S-V
	Src_Hue.copyTo(res[0]);

	Mat hueroi(Src_Hue, hist->Rec);
	Mat maskroi(mask, hist->Rec);

	calcHist(&hueroi, 1, 0, maskroi, hist->hist, 1, &histSize, ranges, true, false);
	normalize(hist->hist, hist->hist, 0., 1., NORM_MINMAX, -1, Mat());
	minMaxLoc(hist->hist, 0, &hist->max_val);
	//cvConvertScale((hist->hist)->bins, (hist->hist)->bins, hist->max_val ? 255. / hist->max_val : 0., 0);	//对直方图的数值转为0~255
}
void Num_Update(void)
{
	int tar = robot_sta_all.target_count_now, a = 0;
	for (int i = 0; i <tar; i++)
		R_Mes_Up(&robot_sta_all.robot_sta_single[i]);			//机器人信息已经储存
	for (int i = 0; i < tar - a; i++)
	{
		if (r_watch.R_Ch2Disap[i])		//如果第i个位置目标消失了
		{
			cout << "target " << i << " disappear\n";
			//waitKey(0);
			//a++;
			for (int j = i; j < tar - 1 - a; j++)
			{
				Robot_Sta_Copy(&robot_sta_all.robot_sta_single[j + 1], &robot_sta_all.robot_sta_single[j]);
				//robot_sta_all.robot_sta_single[j].Track_Now = robot_sta_all.robot_sta_single[j + 1].Track_Now;
				//robot_sta_all.robot_sta_single[j].Track_Pre = robot_sta_all.robot_sta_single[j + 1].Track_Pre;
				r_watch.R_Sta[j] = r_watch.R_Sta[j + 1];
				r_watch.R_Ch2Ap[j] = r_watch.R_Ch2Ap[j + 1];
				r_watch.R_Ch2Disap[j] = r_watch.R_Ch2Disap[j + 1];
			}
			//R_Watch_Refresh();
			R_Sta_Reset(&robot_sta_all.robot_sta_single[tar - a - 1]);
			a++;
			i = -1;
		}
	}
	robot_sta_all.target_count_now = robot_sta_all.target_count_now - a;
}
void R_Mes_Up(Robot_Sta_Single *robot)
{
	robot->robot_sta_pre2 = robot->robot_sta_pre;
	robot->robot_sta_pre = robot->robot_sta_now;

	robot->hist_pre.copyTo(robot->hist_pre2);
	robot->hist_now.copyTo(robot->hist_pre);

	robot->Track_Pre2 = robot->Track_Pre;
	robot->Track_Pre = robot->Track_Now;

	robot->Box2D_Pre2 = robot->Box2D_Pre;
	robot->Box2D_Pre = robot->Box2D_Now;
}
void Robot_Sta_Copy(Robot_Sta_Single *robot1, Robot_Sta_Single *robot2)
{
	/*
	typedef struct Robot_Sta_Single
	{
	Robot_Find robot_sta_now;			//the status of the target in this frame
	Robot_Find robot_sta_pre;			//the status of the target in last frame
	Rect Track_Now;
	Rect Track_Pre;
	RotatedRect Box2D_Now;
	RotatedRect Box2D_Pre;
	MatND hist_now;						//the histogram of a target in this frame
	MatND hist_pre;						//the histogram of a target in last frame
	Hist_Store hist;					//save the message of the hist
	int Once_Start;						//start a new trace-1
	int logo_num;						//the number of the robot
	int track_count;					//the nunber of the frames that we have been tracking now
	}Robot_Sta_Single;						//all messages of a single target
	*/
	//1给2
	robot2->robot_sta_now = robot1->robot_sta_now;
	robot2->robot_sta_pre = robot1->robot_sta_pre;
	robot2->Track_Now = robot1->Track_Now;
	robot2->Track_Pre = robot1->Track_Pre;
	robot2->Box2D_Now = robot1->Box2D_Now;
	robot2->Box2D_Pre = robot1->Box2D_Pre;
	robot1->hist_now.copyTo(robot2->hist_now);
	robot1->hist_pre.copyTo(robot2->hist_pre);
	robot2->hist.bhattacharyya = robot1->hist.bhattacharyya;
	robot2->hist.max_val = robot1->hist.max_val;
	robot2->hist.Rec = robot1->hist.Rec;
	robot1->hist.hist.copyTo(robot2->hist.hist);
	robot2->Once_Start = robot1->Once_Start;
	robot2->logo_num = robot1->logo_num;
	robot2->track_count = robot1->track_count;
	robot2->Serial_Num = robot1->Serial_Num;
	robot2->angle = robot1->angle;
}
void R_Watch_Refresh(void)
{
	for (int i = 0; i < MAX_NUM; i++)
	{
		bool flag1 = (robot_sta_all.robot_sta_single[i].robot_sta_now == ROBOT_GOT);
		bool flag2 = (robot_sta_all.robot_sta_single[i].robot_sta_now == ROBOT_NO || robot_sta_all.robot_sta_single[i].robot_sta_now == ROBOT_BARRIER);
		bool flag3 = (robot_sta_all.robot_sta_single[i].robot_sta_pre == ROBOT_GOT);
		bool flag4 = (robot_sta_all.robot_sta_single[i].robot_sta_pre == ROBOT_NO || robot_sta_all.robot_sta_single[i].robot_sta_pre == ROBOT_BARRIER);

		r_watch.R_Sta[i] = flag1;
		r_watch.R_Ch2Disap[i] = flag2&&flag3;
		r_watch.R_Ch2Ap[i] = flag1&&flag4;
	}
	bool flag1 = (robot_sta_all.target_count_now > robot_sta_all.target_count_pre);
	bool flag2 = (robot_sta_all.target_count_now < robot_sta_all.target_count_pre);
	bool flag3 = (robot_sta_all.target_count_now == robot_sta_all.target_count_pre);

	r_watch.RC_Inc = flag1;
	r_watch.RC_Red = flag2;
}
void Refresh(float height)
{
	//cout << "In refresh robot_sta_all.target_count_now = " << robot_sta_all.target_count_now << endl;
	for (int i = 0; i < robot_sta_all.target_count_now - 1; i++)		//判断重合
	{
		for (int j = i; j < robot_sta_all.target_count_now; j++)
		{
			int a = 0;
			float b = 0.;
			if (i != j && robot_sta_all.robot_sta_single[i].robot_sta_now == ROBOT_GOT && robot_sta_all.robot_sta_single[j].robot_sta_now == ROBOT_GOT)
			{
				b = Re_Coin_Degree(robot_sta_all.robot_sta_single[i].Track_Now, robot_sta_all.robot_sta_single[j].Track_Now, &a);	//判断重合度
				a = (a == 1) ? i : j;
			}
			//cout << "b = " << b << endl;			
			if (b > 0.15)
			{
				//如果重合，就把a这个目标信息清理
				robot_sta_all.robot_sta_single[a].Once_Start = 1;
				robot_sta_all.robot_sta_single[a].robot_sta_now = ROBOT_NO;
				robot_sta_all.robot_sta_single[a].robot_sta_pre = ROBOT_GOT;	//强行刷新
				robot_sta_all.robot_sta_single[a].track_count = 0;
				robot_sta_all.robot_sta_single[a].Serial_Num = -1;
				robot_sta_all.robot_sta_single[a].angle.initialize();
			}
		}
	}
	R_Watch_Refresh();
	Num_Update();
	robot_sta_all.target_count_pre2 = robot_sta_all.target_count_pre;
	robot_sta_all.target_count_pre = robot_sta_all.target_count_now;
}
void RoaRec_Long_Cal(RotatedRect res, vector<Point>&longcenter)
{
	double ess = 1e-1;
	Point2f pt[4];
	res.points(pt);	//得到四个点
	long distance1 = pow((pt[0].x - pt[1].x), 2) + pow((pt[0].y - pt[1].y), 2);
	long distance2 = pow((pt[1].x - pt[2].x), 2) + pow((pt[1].y - pt[2].y), 2);

	if (distance1 > distance2 + ess)		//0,1
	{
		cout << "找到！" << endl;
		longcenter.push_back(Point2f((pt[0].x + pt[1].x) / 2, (pt[0].y + pt[1].y) / 2));
		longcenter.push_back(Point2f((pt[2].x + pt[3].x) / 2, (pt[2].y + pt[3].y) / 2));
	}
	else if (distance1 < distance2 - ess)	//1,2
	{
		cout << "找到！" << endl;
		longcenter.push_back(Point2f((pt[1].x + pt[2].x) / 2, (pt[1].y + pt[2].y) / 2));
		longcenter.push_back(Point2f((pt[0].x + pt[3].x) / 2, (pt[0].y + pt[3].y) / 2));
	}
	else
	{
		cout << "没找到！" << endl;
		longcenter.push_back(Point2f(0., 0.));
		longcenter.push_back(Point2f(0., 0.));
	}
}
void RoaRec_Draw(Mat& img, RotatedRect res)
{
	Point2f pt[4];
	res.points(pt);	//得到四个点

	circle(img, pt[0], 5, Scalar(255, 0, 0), -1);		//第0个 blue
	circle(img, pt[1], 5, Scalar(0, 255, 0), -1);		//第1个 green
	circle(img, pt[2], 5, Scalar(0, 0, 255), -1);		//第2个 red
	circle(img, pt[3], 5, Scalar(0, 255, 255), -1);		//第2个 white

	for (int i = 0; i < 4; i++)
		line(img, pt[i], pt[(i + 1) % 4], Scalar(255, 255, 0), 2);

	//for (int i = 0; i < longcenter.size(); i++)
		//circle(img, longcenter[i], 5, Scalar(255, 255, 255), -1);
}
void Sig_Obj_Tra(Mat src, Robot_Sta_Single* rssig)
{
	if (rssig->robot_sta_now == ROBOT_NO)		//也许用不到
	{
		cout << "There is no target!\n";
		rssig->robot_sta_pre = rssig->robot_sta_now;
		rssig->robot_sta_now = ROBOT_NO;

		rssig->Track_Now.x = -1;
		rssig->Track_Now.y = -1;
		rssig->Track_Now.width = 0;
		rssig->Track_Now.height = 0;
		putText(srcimgdraw, "No target", { srcimgdraw.cols - 80, 20 }, FONT_HERSHEY_COMPLEX, 0.5f, Scalar(255, 0, 0));
		//waitKey(0);
	}
	else//这个编号是有目标的
	{
		if ((rssig->robot_sta_pre == ROBOT_NO || rssig->robot_sta_pre == ROBOT_BARRIER) && (rssig->robot_sta_now == ROBOT_GOT))
			rssig->Once_Start = 1;
		else;
		if (rssig->logo_num == 0)	//如果是红色
		{
			backproject.setTo(0);
			

			cvtColor(src, Src_HSV, CV_BGR2HSV);
			//inRange(Src_HSV, Scalar(0, SMIN, MIN(VMIN, VMAX)), Scalar(180, 256, MAX(VMIN, VMAX)), mask);
			inRange(Src_HSV, Scalar(0, SMIN, MIN(VMIN, VMAX)), Scalar(180, 255, MAX(VMIN, VMAX)), mask);
			//imshow("mask",mask);
			//得到 mask ，只处理像素值为H：0~180，S：smin(30)~256，V：vmin(10)~vmax(256)之间的部分
			vector< Mat > sp;
			cv::GaussianBlur(Src_HSV,Src_HSV,Size(5,5),0,0,BORDER_DEFAULT );
			for(int i=0;i<Src_HSV.cols;i++)
			for(int j=0;j<Src_HSV.rows;j++)
			{
				if(Src_HSV.at<cv::Vec3b>(j,i)[0]<15)
					Src_HSV.at<cv::Vec3b>(j,i)[0]=Src_HSV.at<cv::Vec3b>(j,i)[0]+165;
			}
			split(Src_HSV, sp);
			Src_Hue = sp[0];

			if (rssig->Once_Start == 1)//get the initial rectangle, a new track
			{
				cout << "Start a new tracking" << endl;
				rssig->Once_Start = 0;
			}
			double max_val = 0.f;

			Mat hueroii = Src_Hue(rssig->Track_Now);
			Mat maskroii = mask(rssig->Track_Now);

			imshow("hueroii",hueroii);
			imshow("maskroii",maskroii);

			calcHist(&hueroii, 1, 0, maskroii, rssig->hist_now, 1, &histSize, ranges, true, false);
			normalize(rssig->hist_now, rssig->hist_now, 0., 1., NORM_MINMAX, -1, Mat());
			minMaxLoc(rssig->hist_now, 0, &(rssig->hist.max_val), 0, 0);

			//dilate(Src_Hue, Src_Hue, Mat(), Point(-1, -1), 1);
			//cv::GaussianBlur(Src_Hue,Src_Hue,Size(5,5),0,0,BORDER_DEFAULT );
			imshow("srchue",Src_Hue);
			calcBackProject(&Src_Hue, 1, 0, rssig->hist_now, backproject, ranges);
			multiply(backproject, mask, backproject);
			//dilate(backproject, backproject, Mat(), Point(-1, -1), 1);
			dilate(backproject, backproject, Mat(), Point(-1, -1), 2);
		}
		else if (rssig->logo_num == 1)
		{
			if (rssig->Once_Start == 1)//get the initial rectangle, a new track
			{
				cout << "Start a new tracking\n";
				
				rssig->Once_Start = 0;
			}
 			Green_Tra_Back(src, backproject, rssig->Track_Now);
		}
		else
		{
			backproject.setTo(0);		//非红非绿
		}
		for (int i = 0; i < backproject.rows; i++)
		for (int j = 0; j < backproject.cols; j++)
		{
			uchar ptr1 = backproject.at<uchar>(i, j);
			if (ptr1 != 0)
				backproject.at<uchar>(i, j) = 255;				
		}
		backprojectforerve += backproject;
		imshow("backproject_mode", backprojectforerve);
		//imshow("mask", mask);
		//waitKey(0);
		//backproject.setTo(0);
		
		//if(rssig->track_count < 2)
		if(1)
		{
//			int xx1 = rssig->Track_Now.tl().x;
//			int xx2 = rssig->Track_Now.br().x;
//			int yy1 = rssig->Track_Now.tl().y;
//			int yy2 = rssig->Track_Now.br().y;
//	
//			cout << "xx1 = " << xx1 << endl;
//			cout << "xx2 = " << xx2 << endl;
//			cout << "yy1 = " << yy1 << endl;
//			cout << "yy2 = " << yy2 << endl;
			rssig->Box2D_Now = CamShift(backproject, rssig->Track_Now, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));
			
		}
		else
		{
//			int xx1 = rssig->Track_Predict.tl().x;
//			int xx2 = rssig->Track_Predict.br().x;
//			int yy1 = rssig->Track_Predict.tl().y;
//			int yy2 = rssig->Track_Predict.br().y;
//			cout << "xx1 = " << xx1 << endl;
//			cout << "xx2 = " << xx2 << endl;
//			cout << "yy1 = " << yy1 << endl;
//			cout << "yy2 = " << yy2 << endl;
			rssig->Box2D_Now = CamShift(backproject, rssig->Track_Predict, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));
		}
		
		rssig->Track_Now = rssig->Box2D_Now.boundingRect();
		rssig->Track_Now.x = MAX(0, rssig->Track_Now.x);
		rssig->Track_Now.y = MAX(0, rssig->Track_Now.y);
		if (rssig->Track_Now.x + rssig->Track_Now.width > I_W - 1)		
			rssig->Track_Now.width = I_W - 1 - rssig->Track_Now.x;
		if (rssig->Track_Now.y + rssig->Track_Now.height > I_H - 1)
			rssig->Track_Now.height = I_H - 1 - rssig->Track_Now.y;
	}
}
void Sig_Match(Robot_Sta_Single* rssig, Mat src, float height)
{
	if (rssig->logo_num == 0)
	{
		Mat hueroi = Src_Hue(rssig->Track_Now);
		Mat maskroi = mask(rssig->Track_Now);
		
		calcHist(&hueroi, 1, 0, maskroi, rssig->hist_now, 1, &histSize, ranges, true, false);
		normalize(rssig->hist_now, rssig->hist_now, 0., 1., NORM_MINMAX, -1, Mat());
		minMaxLoc(rssig->hist.hist, 0, &(rssig->hist.max_val));

		if (rssig->track_count == 0)	//如果是第一帧跟踪
			rssig->hist_now.copyTo(rssig->hist.hist);

		if (rssig->robot_sta_now == ROBOT_GOT)
			rssig->hist.bhattacharyya = compareHist(rssig->hist.hist, rssig->hist_now, CV_COMP_BHATTACHARYYA);
		else if (rssig->robot_sta_now == ROBOT_NO)
			rssig->hist.bhattacharyya = 2.0;
		else;
		float Ths_Bha = 0.;
		Ths_Bha = COM_Bhattacharyya + (1 - COM_Bhattacharyya)*exp(-(0.5*(double)rssig->track_count));

		int ptx = (int)(rssig->Track_Now.br().x + rssig->Track_Now.tl().x) / 2;
		int pty = (int)(rssig->Track_Now.br().y + rssig->Track_Now.tl().y) / 2;

		Point pt = Point(ptx, pty);
		//Point2f pt = rssig->Box2D_Now.center;

		bool flag1 = (Tra_Filter(rssig->Box2D_Now, 5.0, height, MIN_POL_3_CSOFF_ROBUSTOFF, -5, MAX_EXP_4_CSOFF, 25) == 0);	//尺寸丢失
		bool flag2 = (Ell_Inside_Cal(ELL_CEN, ELL_A, ELL_B, Point((int)pt.x, (int)pt.y)) == 0);		//位置丢失
		bool flag3 = (rssig->hist.bhattacharyya > Ths_Bha);		//直方图丢失
		if ((flag1 || flag2 || flag3) && rssig->track_count!=0)
		{
			if (flag1)
				cout << "RED LOSE! Because of too small or too big!" << endl;
			if (flag2)
				cout << "RED LOSE! Because of outside!" << endl;
			if (flag3)
				cout << "RED LOSE! Because of hist no not match!" << endl;
			rssig->robot_sta_now = ROBOT_NO;
			rssig->robot_sta_pre = ROBOT_GOT;
			rssig->Once_Start = 1;
			rssig->track_count = 0;
			//waitKey(0);
		}
		else
		{
			Hist_Up(rssig->hist.hist, rssig->hist_now, rssig->hist.bhattacharyya);	//直方图更新			
			rectangle(srcimgdraw, rssig->Track_Now.tl(), rssig->Track_Now.br(), Scalar(0, 0, 255), 2, 8, 0);
			Tra_Pos_Predict(rssig, 1.00);
			circle(srcimgdraw, Point((int)pt.x, (int)pt.y), 3, Scalar(0, 0, 255), -1);
			vector<Point>longcenter;
			RoaRec_Long_Cal(rssig->Box2D_Now, longcenter);
			RoaRec_Draw(srcimgdraw, rssig->Box2D_Now);
			rssig->track_count++;		//跟踪帧数加一
		}
	}
	else if (rssig->logo_num == 1)
	{
		//统计方差的方法
		float variance = 0.;
		if (rssig->robot_sta_now == ROBOT_NO)
			variance = 0.;
		else
		{
			//cout << "x = " << rssig->Track_Now.x << endl;
			//cout << "y = " << rssig->Track_Now.y << endl;
			//cout << "width = " << rssig->Track_Now.width << endl;
			//cout << "height = " << rssig->Track_Now.height << endl;

			Mat roi = src(rssig->Track_Now);
			cvtColor(roi, roi, CV_BGR2GRAY);
			
			variance = Sig_Ch_Variance(roi);
		}
		
		int ptx = (int)(rssig->Track_Now.br().x + rssig->Track_Now.tl().x) / 2;
		int pty = (int)(rssig->Track_Now.br().y + rssig->Track_Now.tl().y) / 2;

		Point pt = Point(ptx, pty);
		//Point2f pt = rssig->Box2D_Now.center;

		bool flag1 = (Tra_Filter(rssig->Box2D_Now, 5.0, height, MIN_POL_3_CSOFF_ROBUSTOFF, -5, MAX_EXP_4_CSOFF, 25) == 0);	//尺寸丢失
		bool flag2 = (Ell_Inside_Cal(ELL_CEN, ELL_A, ELL_B, Point((int)pt.x, (int)pt.y)) == 0);		//位置丢失
		bool flag3 = (variance < VarThres);
		if ((flag1 || flag2 || flag3) && rssig->track_count != 0)
		{
			if (flag1)
				cout << "RED LOSE! Because of too small or too big!" << endl;
			if (flag2)
				cout << "RED LOSE! Because of outside!" << endl;
			if (flag3)
				cout << "GREEN LOSE! Because of variance too small!" << endl;
			rssig->robot_sta_now = ROBOT_NO;
			rssig->robot_sta_pre = ROBOT_GOT;
			rssig->Once_Start = 1;
			rssig->track_count = 0;
			//waitKey(0);
		}
		else
		{
			rssig->Track_Now = Dec_Tra_Connect(rssig->Track_Now, 1.8);	//以系数1.1更新位置
			rectangle(srcimgdraw, rssig->Track_Now.tl(), rssig->Track_Now.br(), Scalar(0, 255, 0), 2, 8, 0);
			Tra_Pos_Predict(rssig, 1.00);
			circle(srcimgdraw, Point((int)pt.x, (int)pt.y), 3, Scalar(0, 255, 0), -1);
			vector<Point>longcenter;
			RoaRec_Long_Cal(rssig->Box2D_Now, longcenter);
			RoaRec_Draw(srcimgdraw, rssig->Box2D_Now);
			rssig->track_count++;		//跟踪帧数加一
		}
	}
	else;
}
void All_Obj_Tra(Mat img, float height)
{
	cout << "In All_Obj_Tra!" << endl;
	
	backprojectforerve.setTo(0);
	int temp = robot_sta_all.target_count_now;
	for (int i = 0; i < robot_sta_all.target_count_now; i++)
	{
		cout << "robot_sta_all.robot_sta_single[" << i << "].logo_num = " << robot_sta_all.robot_sta_single[i].logo_num << endl;
		//
		if (robot_sta_all.robot_sta_single[i].logo_num == 0 || robot_sta_all.robot_sta_single[i].logo_num == 1)
		{
			cout << "logo = " << robot_sta_all.robot_sta_single[i].logo_num << endl;
			
			Sig_Obj_Tra(img, &robot_sta_all.robot_sta_single[i]);
			cout << "logo2 = " << robot_sta_all.robot_sta_single[i].logo_num << endl;
			Sig_Match(&robot_sta_all.robot_sta_single[i], img, height);
		}
		else
		{
			cout << "No such logo!\n";
			robot_sta_all.robot_sta_single[i].robot_sta_now = ROBOT_NO;
			robot_sta_all.robot_sta_single[i].robot_sta_pre = ROBOT_GOT;
			robot_sta_all.robot_sta_single[i].Once_Start = 1;
			robot_sta_all.robot_sta_single[i].track_count = 0;
			//temp--;
			if (robot_sta_all.target_count_now == 0)
				robot_sta_all.target_count_now = 0;
		}
	}
	robot_sta_all.target_count_now = temp;
}
void Green_Tra_Back(Mat src, Mat& backproject, Rect rect)
{
	backproject.setTo(0);

	Mat srctemp;
	src.copyTo(srctemp);
	Mat green = srctemp(rect);	//公用内存

	//Mat img_gray(green.size(), CV_8UC1);
	cvtColor(green, green, CV_BGR2GRAY);

	int thre = OTSU_Th(green);
	threshold(green, green, thre, 255, THRESH_BINARY_INV);

	int morph_size_1 = 1;
	Mat element_1 = getStructuringElement(0, Size(2 * morph_size_1 + 1, 2 * morph_size_1 + 1), Point(morph_size_1, morph_size_1));
	erode(green, green, element_1);
	dilate(green, green, element_1);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(green, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	double maxArea = 0.;
	double area = 0.;
	int maxAreaIdex = 0;

	for (int i = 0; i<contours.size(); i++)
	{
		area = fabs(contourArea(contours[i]));
		if (area>maxArea)
		{
			maxArea = area;
			maxAreaIdex = i;
		}
	}
	green.setTo(0);
	if (maxArea>10)
	{
		drawContours(green, contours, maxAreaIdex, Scalar(255), -1);
	}
	else;

	Mat pp = backproject(rect);
	green.copyTo(pp);
}

void Dec_Tra_Match(Robot_Sta_All* r, vector<Rect>Red_Out, vector<Rect>Green_Out)
{
//	float predict_thres = 50.0;		//判断为一个目标的匹配阈值

	int rc = r->target_count_now;	//目标总数
	int drc = Red_Out.size();		//红色目标总数
	int dgc = Green_Out.size();		//绿色目标总数

	int i = 0;

	for (int p = 0; p < rc; p++)
		Tra_Pos_Predict(&(r->robot_sta_single[p]), 1.05);		//给出预测

	int matflag = 0;

	while (i != drc)
	{
		matflag = 0;

		float dcx1 = (float)(Red_Out[i].tl().x)*1.0;
		float dcx2 = (float)(Red_Out[i].br().x)*1.0;
		float dcy1 = (float)(Red_Out[i].tl().y)*1.0;
		float dcy2 = (float)(Red_Out[i].br().y)*1.0;
		float dcx = (dcx1 + dcx2) / 2;
		float dcy = (dcy1 + dcy2) / 2;					//检测目标中心

		for (int k = 0; k < rc; k++)
		{
//			float tcx1 = (float)(r->robot_sta_single[k].Track_Predict.tl().x)*1.0;
//			float tcx2 = (float)(r->robot_sta_single[k].Track_Predict.br().x)*1.0;
//			float tcy1 = (float)(r->robot_sta_single[k].Track_Predict.tl().y)*1.0;
//			float tcy2 = (float)(r->robot_sta_single[k].Track_Predict.br().y)*1.0;
			float tcx1 = (float)(r->robot_sta_single[k].Track_Now.tl().x)*1.0;
			float tcx2 = (float)(r->robot_sta_single[k].Track_Now.br().x)*1.0;
			float tcy1 = (float)(r->robot_sta_single[k].Track_Now.tl().y)*1.0;
			float tcy2 = (float)(r->robot_sta_single[k].Track_Now.br().y)*1.0;
			float tcx = (tcx1 + tcx2) / 2;
			float tcy = (tcy1 + tcy2) / 2;				//跟踪预测中心

			HYHYHY = pow(dcx - tcx, 2) + pow(dcy - tcy, 2);

			if (pow(dcx - tcx, 2) + pow(dcy - tcy, 2) <= pow(DEC_TRA_MAT, 2))		//满足同一目标条件，对结构体不做刷新
			{
				matflag = 1;		//置匹配标志位
				i++;
				if (r->robot_sta_single[k].logo_num == 1)	//如果类型不匹配，刷新
					r->robot_sta_single[k].logo_num = 0;
				break;
			}
		}
		if (matflag == 0)	//在之前的目标中没有找到一个和这个新检测的匹配，那么就把这个目标放在后面
		{
			cout << "TOO FAR!!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
			cout << HYHYHY <<endl;

			//imshow("PPP",srcimgdraw);
			//waitKey(0);
			cout << "This new target is red!" << endl;
			r->robot_sta_single[rc].Track_Now = Dec_Tra_Connect(Red_Out[i], 1.0);
			r->robot_sta_single[rc].robot_sta_now = ROBOT_GOT;
			r->robot_sta_single[rc].logo_num = 0;//标签为0表示红色
			r->robot_sta_single[rc].Serial_Num=Serial_Num;
			Serial_Num++;
			
			rectangle(srcimgdraw, r->robot_sta_single[i].Track_Now, Scalar(0, 0, 255), 2);
			int xx1 = r->robot_sta_single[i].Track_Now.tl().x;
			int xx2 = r->robot_sta_single[i].Track_Now.br().x;
			int yy1 = r->robot_sta_single[i].Track_Now.tl().y;
			int yy2 = r->robot_sta_single[i].Track_Now.br().y;									
			circle(srcimgdraw, Point((xx1 + xx2) / 2, (yy1 + yy2) / 2), 3, Scalar(0, 0, 255), -1);
			i++;
			if (rc < 9)
				rc++;
		}
	}
	i = 0;
	while (i != dgc)
	{
		matflag = 0;

		float dcx1 = (float)(Green_Out[i].tl().x)*1.0;
		float dcx2 = (float)(Green_Out[i].br().x)*1.0;
		float dcy1 = (float)(Green_Out[i].tl().y)*1.0;
		float dcy2 = (float)(Green_Out[i].br().y)*1.0;
		float dcx = (dcx1 + dcx2) / 2;
		float dcy = (dcy1 + dcy2) / 2;					//检测目标中心

		for (int k = 0; k < rc; k++)
		{
//			float tcx1 = (float)(r->robot_sta_single[k].Track_Predict.tl().x)*1.0;
//			float tcx2 = (float)(r->robot_sta_single[k].Track_Predict.br().x)*1.0;
//			float tcy1 = (float)(r->robot_sta_single[k].Track_Predict.tl().y)*1.0;
//			float tcy2 = (float)(r->robot_sta_single[k].Track_Predict.br().y)*1.0;
			float tcx1 = (float)(r->robot_sta_single[k].Track_Now.tl().x)*1.0;
			float tcx2 = (float)(r->robot_sta_single[k].Track_Now.br().x)*1.0;
			float tcy1 = (float)(r->robot_sta_single[k].Track_Now.tl().y)*1.0;
			float tcy2 = (float)(r->robot_sta_single[k].Track_Now.br().y)*1.0;
			float tcx = (tcx1 + tcx2) / 2;
			float tcy = (tcy1 + tcy2) / 2;				//跟踪预测中心

			HYHYHY = pow(dcx - tcx, 2) + pow(dcy - tcy, 2) ;

			if (pow(dcx - tcx, 2) + pow(dcy - tcy, 2) <= pow(DEC_TRA_MAT, 2))		//满足同一目标条件，对结构体不做刷新
			{
				matflag = 1;		//置匹配标志位
				i++;
				if (r->robot_sta_single[k].logo_num == 0)	//如果类型不匹配，刷新
					r->robot_sta_single[k].logo_num = 1;
				break;
			}
		}
		if (matflag == 0)	//在之前的目标中没有找到一个和这个新检测的匹配，那么就把这个目标放在后面
		{
			cout << "This new target is green!" << endl;
			cout << HYHYHY <<endl;
			//imshow("PPP",srcimgdraw);
			//waitKey(0);
			cout << "TOO FAR!!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
			r->robot_sta_single[rc].Track_Now = Dec_Tra_Connect(Green_Out[i], 0.9);	//0.33333
			r->robot_sta_single[rc].robot_sta_now = ROBOT_GOT;
			r->robot_sta_single[rc].logo_num = 1;//标签为0表示红色
			r->robot_sta_single[rc].Serial_Num=Serial_Num;
			Serial_Num++;

			rectangle(srcimgdraw, r->robot_sta_single[i].Track_Now, Scalar(0, 255, 0), 2);		
			int xx1 = r->robot_sta_single[i].Track_Now.tl().x;
			int xx2 = r->robot_sta_single[i].Track_Now.br().x;
			int yy1 = r->robot_sta_single[i].Track_Now.tl().y;
			int yy2 = r->robot_sta_single[i].Track_Now.br().y;
			circle(srcimgdraw, Point((xx1 + xx2) / 2, (yy1 + yy2) / 2), 3, Scalar(0, 255, 0), -1);
			i++;
			if (rc < 9)
				rc++;
		}
	}
	r->target_count_now = rc;
}

void Tra_Pos_Predict(Robot_Sta_Single* rssig, float scale)
{
	if (rssig->track_count <= 1)
	{
		//cout << "In function Tra_Pos_Pre track_count too few!" << endl;
		rssig->Track_Predict = rssig->Track_Now;
		//return;
	}
	else
	{
		float dx = rssig->Box2D_Pre.center.x - rssig->Box2D_Pre2.center.x;
		float dy = rssig->Box2D_Pre.center.y - rssig->Box2D_Pre2.center.y;		//前2与前1差分

		int xnow = rssig->Box2D_Now.center.x + (int)dx;
		int ynow = rssig->Box2D_Now.center.y + (int)dy;
		
		xnow = MIN(xnow, I_W -1);
		ynow = MIN(xnow, I_H -1);
		
		rssig->Track_Predict.x = MAX(0, xnow - rssig->Track_Now.width / 2);
		rssig->Track_Predict.x = MIN(rssig->Track_Predict.x, I_W - 1 - rssig->Track_Now.width);
		rssig->Track_Predict.y = MAX(0, ynow - rssig->Track_Now.height / 2);
		rssig->Track_Predict.y = MIN(rssig->Track_Predict.y , I_H - 1 - rssig->Track_Now.height);
		
		rssig->Track_Predict.width = rssig->Track_Now.width;
		rssig->Track_Predict.height = rssig->Track_Now.height;
//		cout << "Tra_Pos_Predict xx1 = " << rssig->Track_Predict.width << endl;
//		cout << "Tra_Pos_Predict xx2 = " << rssig->Track_Predict.height << endl;

		rssig->Track_Predict = Dec_Tra_Connect(rssig->Track_Predict, 1.1);		//预测
	}
	
//	int xx1 = rssig->Track_Predict.tl().x;
//	int xx2 = rssig->Track_Predict.br().x;
//	int yy1 = rssig->Track_Predict.tl().y;
//	int yy2 = rssig->Track_Predict.br().y;

//	cout << "prexx1 = " << xx1 << endl;
//	cout << "prexx2 = " << xx2 << endl;
//	cout << "preyy1 = " << yy1 << endl;
//	cout << "preyy2 = " << yy2 << endl;
	
}
