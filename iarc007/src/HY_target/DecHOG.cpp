#include "iarc007/hy_target/DecHOG.h"

static int lalala = 0;
CvSVM mySVM;
Mat Text_Win;
HOGDescriptor myHOG;
vector<Rect>found;
vector<Rect>Obj_Out;
vector<Rect>Red_Out;
vector<Rect>Green_Out;
vector<Rect>Barrier_Out;
vector<Rect>Dec_Wrong;

void Hog_Init(void)
{
	myHOG.winSize = WIN;							//检测窗口大小
	myHOG.blockSize = BLOCK;						//块大小，目前只支持Size(16, 16)
	myHOG.blockStride = STEP;						//块的滑动步长，大小只支持是单元格cell_size大小的倍数
	myHOG.cellSize = CELL;							//单元格的大小，目前只支持Size(8, 8)
	myHOG.nbins = 9;								//单元格的大小，目前只支持Size(8, 8)
	myHOG.derivAperture = 1;						//
	myHOG.winSigma = -1.0;							//高斯滤波窗口的参数
	myHOG.histogramNormType = 0;
	myHOG.L2HysThreshold = 0.20000000000000001;		//块内直方图归一化类型L2-Hys的归一化收缩率
	myHOG.gammaCorrection = false;					//是否gamma校正
	myHOG.nlevels = 64;								//检测窗口的最大数量
}
void Multi_Scale_Obj_Dec(int height, Mat src, HOGDescriptor myHOG, Point start, Size winsize, Size step, float scale, int S, int hflag, int fflag)
{
	Mat srctemp = src;

	int count = 0, time = 1, minpredict = 0, maxpredict = 0;

	Obj_MaxMinSize_Cal(height, MIN_POL_3_CSOFF_ROBUSTOFF, MAX_EXP_4_CSOFF, 5, 15, &minpredict, &maxpredict);	//高度限制
	minpredict = (int)(minpredict / S);
	maxpredict = (int)(maxpredict / S);
	//cout << "minpredict = " << minpredict << ",  " << "maxpredict = " << maxpredict << "\n";

	int minpredictfordec = minpredict;
	int maxpredictfordec = maxpredict;		//检测

	int minpredictforfilter = minpredict;
	int maxpredictforfilter = maxpredict;	//滤波

	if (!hflag)
	{
		minpredictfordec = 0;
		maxpredictfordec = MIN(I_H, I_W);
	}

	int widthref = (int)(winsize.width);
	int heightref = (int)(winsize.height);	//搜索框参考值

	int widthnew = (int)(srctemp.cols / S);
	int heightnew = (int)(srctemp.rows / S);	//图片大小

	resize(srctemp, srctemp, Size((int)(widthnew), (int)(heightnew)), 0, 0, INTER_CUBIC);

	if (maxpredictfordec <= widthref)			//如果拟合目标尺度最大值都要比初始搜索框小，那么就只进行一次全搜索
	{
		printf("Once enough!\n");
		Full_Search_Img(srctemp, start, winsize, step, myHOG, found, time, S, scale);
		time++;
	}
	else
	{
		//printf("Need to carry on many times!\n");
		while (widthref < minpredictfordec || heightref < minpredictfordec)
		{
			widthref = (int)(widthref*scale);		//搜索框扩大
			heightref = (int)(heightref*scale);

			widthnew = (int)(widthnew / scale);		//图幅缩小
			heightnew = (int)(heightnew / scale);
			time++;
		}

		while (srctemp.cols >= winsize.width && srctemp.rows >= winsize.height)		//如果图像尺度大于窗口，越变目标越小
		{
			resize(srctemp, srctemp, Size(widthnew, heightnew), 0, 0, INTER_CUBIC);
			if (srctemp.cols < winsize.width || srctemp.rows < winsize.height)
				break;

			Full_Search_Img(srctemp, start, winsize, step, myHOG, found, time, S, scale);	//一次全搜索
			time++;

			widthnew = (int)(widthnew / scale);
			heightnew = (int)(heightnew / scale);		//图片大小

			widthref = (int)(widthref*scale);
			heightref = (int)(heightref*scale);
			if (widthref > maxpredictfordec || heightref > maxpredictfordec)
				break;
		}
	}

	Mat srcfilter = Mat::zeros(Size(src.cols, src.rows), CV_8UC3);
	src.copyTo(srcfilter);

	if (!fflag)
	{
		minpredictforfilter = 0;
		maxpredictforfilter = MIN(I_H, I_W);
	}
	//cout << "min = " << minpredictforfilter*S << ",  " << "max = " << maxpredictforfilter*S << "\n";

	if (height < 250)
		Obj_Filter(src, maxpredictforfilter*S, minpredictforfilter*S, found, Obj_Out); 
	return;
}
void Full_Search_Img(Mat src, Point start, Size winsize, Size stepsize, HOGDescriptor myHOG, vector<Rect>&found, int time, int S, float scale)
{
	//cout << "Image size: " << src.cols << "  " << src.rows << "\n";
	int xbin = (src.cols - winsize.width) / stepsize.width + 1;
	int ybin = (src.rows - winsize.height) / stepsize.height + 1;

	int totalcount = 0;
	for (int i = 0; i < xbin; i++)
	for (int j = 0; j < ybin; j++)
	{
		totalcount++;
		vector<float>descriptors;		//结果数组

		Rect win = Rect(start.x + i*stepsize.width, start.y + j*stepsize.height, winsize.width, winsize.height);
		Mat imghog = src(win);

		myHOG.compute(imghog, descriptors, Size(8, 8), Size(0, 0)); //调用计算函数开始计算

		Mat SVMtrainMat = Mat::zeros(1, descriptors.size(), CV_32FC1);
		int n = 0;
		for (vector<float>::iterator iter = descriptors.begin(); iter != descriptors.end(); iter++)
		{
			SVMtrainMat.at<float>(0, n) = *iter;
			n++;
		}
		int ret = mySVM.predict(SVMtrainMat);

		if (ret == 0)
		{
			//cout << "Target get!" << "ret = " << ret << "\n";
			win.x = (int)((win.x*pow(scale, time - 1))*S);
			win.y = (int)((win.y*pow(scale, time - 1))*S);
			win.width = (int)((win.width*pow(scale, time - 1))*S);
			win.height = (int)((win.height*pow(scale, time - 1))*S);
			found.push_back(win);
		}
		else if (ret == 1)
		{
			//cout << "Target get!" << "ret = " << ret << "\n";
			win.x = (int)((win.x*pow(scale, time - 1))*S);
			win.y = (int)((win.y*pow(scale, time - 1))*S);
			win.width = (int)((win.width*pow(scale, time - 1))*S);
			win.height = (int)((win.height*pow(scale, time - 1))*S);
			found.push_back(win);
		}
		else if (ret == 2)
		{
			//cout << "Target get!" << "ret = " << ret << "\n";
			win.x = (int)((win.x*pow(scale, time - 1))*S);
			win.y = (int)((win.y*pow(scale, time - 1))*S);
			win.width = (int)((win.width*pow(scale, time - 1))*S);
			win.height = (int)((win.height*pow(scale, time - 1))*S);
			found.push_back(win);
		}
		else;
	}
	//cout << "totalcount = " << totalcount << "\n";
}
