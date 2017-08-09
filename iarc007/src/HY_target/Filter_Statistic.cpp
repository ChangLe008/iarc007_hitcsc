#include "iarc007/hy_target/Filter_Statistic.h"

const int histSizeFiler = HIST_STA;
float hranges_arrf[2] = { 0, 255 };
const float* rangesf[3] = { hranges_arrf, hranges_arrf, hranges_arrf };
int channelsf[] = { 0, 1, 2 };

/*ͳ����غ���*/
void Hist_Statistic_Init(Hist_Statistic* statistic)
{
	/*
	typedef struct Hist_Statistic		//ֱ��ͼͳ�ƽṹ��
	{
	//ͳ��ʱ��Ҫ������ͼƬresize��64*64��
	int Is_Num_Count[6];			//B-G-R-H-S-V��256����ֵ�в�Ϊ��ĸ���
	int Node_Thres[2][6];			//B-G-R-H-S-Vѡ���12���ڵ�
	float Bet_Node_Count[6];		//�ڽڵ�������Ҳ�Ϊ��ĸ���
	}Hist_Statistic;
	*/
	for (int i = 0; i < 6; i++)
	{
		statistic->Is_Num_Count[i] = 0;
		statistic->Node_Thres[0][i] = 0;
		statistic->Node_Thres[1][i] = 0;
		statistic->Bet_Node_Count[i] = 0.;
		statistic->Is_Num_Mean[i] = 0.;
		statistic->Is_Num_Stddev[i] = 0.;
		statistic->Two_Dim_Entropy[i] = 0.;
		statistic->Node_Proportion[i] = 0.;
	}
}
int Sig_Ch_Greater(Mat img, int threshold)
{
	if (img.channels() != 1)
		return 0;
	Mat hist = Mat::zeros(Size(1, HIST_HDIMS), CV_32FC1);
	calcHist(&img, 1, channelsf, Mat(), hist, 1, &histSizeFiler, rangesf, true, false);
	int s = 0;
	for (int i = 0; i < HIST_HDIMS; i++)
	if (hist.at<float>(i) > threshold)
		s++;
	return s;
}
void Any_Ch_Greater(Mat img, Hist_Statistic *hist_statistic, int threshold, uchar flag)
{
	Mat hsv;
	vector<Mat>spbgr;
	vector<Mat>sphsv;

	cvtColor(img, hsv, CV_BGR2HSV_FULL);
	split(hsv, sphsv);
	split(img, spbgr);

	if ((flag >> 6) != 0)	//���ǰ��λ����ֵ
	{
		cout << "Input error!" << endl;
		return;
	}
	for (int i = 0; i < 6; i++)		//BGR
	{
		uchar flagtemp = (flag >> i) & 0x01;
		if (i < 3)	//B-G-R
		{
			if (flagtemp)		//˵�����ͨ����Ҫͳ��
			{
				//cout << "i = " << i << endl;
				hist_statistic->Is_Num_Count[i] = Sig_Ch_Greater(spbgr[i % 3], threshold);
			}
		}
		if (i >= 3)
		{
			if (flagtemp)
			{
				//cout << "i = " << i << endl;
				hist_statistic->Is_Num_Count[i] = Sig_Ch_Greater(sphsv[i % 3], threshold);
			}
		}
	}
	return;
}
void Ch_Node_Set(Hist_Statistic *hist_statistic, int start[], int end[], uchar flag)
{
	if ((flag >> 6) != 0)	//���ǰ��λ����ֵ
	{
		cout << "Input error!" << endl;
		return;
	}
	for (int i = 0; i < 6; i++)		//BGR
	{
		uchar flagtemp = (flag >> i) & 0x01;
		if (flagtemp)		//˵�����ͨ����Ҫͳ��
		{
			hist_statistic->Node_Thres[0][i] = start[i];
			hist_statistic->Node_Thres[1][i] = end[i];
		}
	}
	return;
}
int Sig_Ch_Node_Greater(Mat img, int start, int end, int threshold)
{
	if (img.channels() != 1)
		return 0;
	int binstart = start%HIST_STA;
	int binend = end%HIST_STA;
	Mat hist = Mat::zeros(Size(1, HIST_STA), CV_32FC1);
	calcHist(&img, 1, channelsf, Mat(), hist, 1, &histSizeFiler, rangesf, true, false);
	int s = 0;
	for (int i = binstart; i <= binend; i++)
	if (hist.at<float>(i) > threshold)
		s++;
	return s;
}
void Any_Ch_Node_Greater(Mat img, Hist_Statistic *hist_statistic, int threshold, uchar flag)
{
	Mat hsv;
	vector<Mat>spbgr;
	vector<Mat>sphsv;

	cvtColor(img, hsv, CV_BGR2HSV_FULL);
	split(hsv, sphsv);
	split(img, spbgr);

	if ((flag >> 6) != 0)	//���ǰ��λ����ֵ
	{
		cout << "Input error!" << endl;
		return;
	}
	for (int i = 0; i < 6; i++)		//BGR
	{
		uchar flagtemp = (flag >> i) & 0x01;
		if (i < 3)	//B-G-R
		{
			if (flagtemp)		//˵�����ͨ����Ҫͳ��
			{
				hist_statistic->Bet_Node_Count[i] = Sig_Ch_Node_Greater(spbgr[i % 3], hist_statistic->Node_Thres[0][i], hist_statistic->Node_Thres[1][i], threshold);
			}
		}
		if (i >= 3)
		{
			if (flagtemp)
			{
				//cout << "��ʼ�ڵ㣺" << hist_statistic->Node_Thres[0][i] << endl;
				//cout << "�����ڵ㣺" << hist_statistic->Node_Thres[1][i] << endl;
				//cout << "��ֵ��" << threshold << endl;
				hist_statistic->Bet_Node_Count[i] = Sig_Ch_Node_Greater(sphsv[i % 3], hist_statistic->Node_Thres[0][i], hist_statistic->Node_Thres[1][i], threshold);
			}
		}
		hist_statistic->Node_Proportion[i] = (float)(hist_statistic->Bet_Node_Count[i] * 1.0) / (float)(SAM_W*SAM_H*1.0);
	}
	return;
}
float Sig_Ch_Variance(Mat img)
{
	if (img.channels() != 1)
		return 0.0;
	//����ƽ��
	float average = 0.0;
	for (int i = 0; i < img.rows; i++)
	for (int j = 0; j < img.cols; j++)
		average += (float)(img.at<uchar>(i, j)*1.0);
	average = average / (float)(SAM_W*SAM_H*1.0);
	//���󷽲�
	float variance = 0.0;
	for (int i = 0; i < img.rows; i++)
	for (int j = 0; j < img.cols; j++)
		variance += pow(((float)(img.at<uchar>(i, j)*1.0) - average), 2);
	variance = variance / ((float)(SAM_W*SAM_H*1.0) - 1.0);
	return variance;
}
void Any_Ch_Variance(Mat img, Hist_Statistic *hist_statistic, uchar flag)
{
	Mat hsv;
	vector<Mat>spbgr;
	vector<Mat>sphsv;

	cvtColor(img, hsv, CV_BGR2HSV_FULL);
	split(hsv, sphsv);
	split(img, spbgr);

	if ((flag >> 6) != 0)	//���ǰ��λ����ֵ
	{
		cout << "Input error!" << endl;
		return;
	}

	for (int i = 0; i < 6; i++)		//BGR
	{
		uchar flagtemp = (flag >> i) & 0x01;
		if (i < 3)	//B-G-R
		{
			if (flagtemp)		//˵�����ͨ����Ҫͳ��
			{
				hist_statistic->Is_Num_Stddev[i] = Sig_Ch_Variance(spbgr[i % 3]);
			}
		}
		if (i >= 3)
		{
			if (flagtemp)
			{
				hist_statistic->Is_Num_Stddev[i] = Sig_Ch_Variance(sphsv[i % 3]);
			}
		}
	}
	return;
}
uchar Eight_NP_Cal(Mat img, int x, int y)
{
	int res = 0;
	int f = 0;
	for (int i = y - 1; i <= y + 1; i++)
	for (int j = x - 1; j <= x + 1; j++)
	{
		if (i == y && j == x)
			continue;
		if (i >= 0 && i < img.cols && j >= 0 && j < img.rows)
		{
			res += img.at<uchar>(i, j);
			f++;
		}
	}
	if (f != 0)
	{
		res = res / f;
		return res;
	}
	else
		return 0;
}
float Sig_Ch_Entropy(Mat img)
{
	if (img.channels() != 1)
		return 0.0;
	Mat entropy = Mat::zeros(Size(img.cols, img.rows), CV_8UC1);

	float temp[256][256] = { 0.0 };
	float NN = SAM_H*SAM_W*1.0;
	float result = 0.0;

	for (int x = 0; x < img.cols; x++)
	for (int y = 0; y < img.rows; y++)
	{
		entropy.at<uchar>(y, x) = Eight_NP_Cal(img, x, y);	//���ɾ�ֵͼ
		temp[img.at<uchar>(y, x)][entropy.at<uchar>(y, x)] = temp[img.at<uchar>(y, x)][entropy.at<uchar>(y, x)] + 1;
	}
	for (int i = 0; i < 256; i++)
	for (int j = 0; j < 256; j++)
	{
		if (temp[i][j] != 0)
		{
			temp[i][j] = temp[i][j] / NN;
			result = result - temp[i][j] * (log(temp[i][j]) / log(2.0));
		}
	}
	return result;
}
void Any_Ch_Entropy(Mat img, Hist_Statistic *hist_statistic, uchar flag)
{
	Mat hsv;
	vector<Mat>spbgr;
	vector<Mat>sphsv;

	cvtColor(img, hsv, CV_BGR2HSV_FULL);
	split(hsv, sphsv);
	split(img, spbgr);

	if ((flag >> 6) != 0)	//���ǰ��λ����ֵ
	{
		cout << "Input error!" << endl;
		return;
	}
	for (int i = 0; i < 6; i++)		//BGR
	{
		uchar flagtemp = (flag >> i) & 0x01;
		if (i < 3)	//B-G-R
		{
			if (flagtemp)		//˵�����ͨ����Ҫͳ��
			{
				hist_statistic->Two_Dim_Entropy[i] = Sig_Ch_Entropy(spbgr[i % 3]);
			}
		}
		if (i >= 3)
		{
			if (flagtemp)
			{
				hist_statistic->Two_Dim_Entropy[i] = Sig_Ch_Entropy(sphsv[i % 3]);
			}
		}
	}
	return;
}
int Gre_Bar_Chose(Mat img, int r, Point center)
{
	Mat imgtemp;
	vector<Mat>imgtempsp;
	img.copyTo(imgtemp);
	
	//Bright_Adjust(imgtemp, imgtemp, 1.5, -80);
	
	//cvtColor(imgtemp, imgtemp, COLOR_BGR2HSV);
	//split(imgtemp, imgtempsp);
	
	//equalizeHist(imgtempsp[2], imgtempsp[2]);		//Vͨ�����⻯
	//equalizeHist(imgtempsp[0], imgtempsp[0]);		//Hͨ�����⻯
	//equalizeHist(imgtempsp[2], imgtempsp[2]);		//Sͨ�����⻯
	
	//merge(imgtempsp, imgtemp);
	//cvtColor(imgtemp, imgtemp, COLOR_HSV2BGR);
	
	//imshow("srcbalance", imgtemp);
	
	cvtColor(imgtemp, imgtemp, COLOR_BGR2GRAY);
	
	//imshow("gray", imgtemp);
	
	//threshold(imgtemp, imgtemp, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	
	threshold(imgtemp, imgtemp, 100, 200, CV_THRESH_BINARY);		//130
	//dilate(imgtemp, imgtemp, Mat(), Point(-1, -1), 2);
	//erode(imgtemp, imgtemp, Mat(), Point(-1, -1), 2);
	
	//imshow("binary", imgtemp);
	
	int sum = 0;
	for (int i = 0; i < imgtemp.cols; i++)
	for (int j = 0; j < imgtemp.rows; j++)
	{
		if (pow(center.x - i, 2) + pow(center.y - j, 2) - pow(r, 2) <= 0)
		if (imgtemp.at<uchar>(i, j) == 0)
			sum++;
	}
	return sum;
}
void Bright_Adjust(Mat &src, Mat& dst,double dContrast, double dBright)
{
	int nVal;

	unsigned char* SrcData = (unsigned char*)src.data;
	unsigned char* DstData = (unsigned char*)dst.data;

	int step = src.step / sizeof(unsigned char) / 3;	//��ͨ��ͼ��Ĳ���

	for (int nI = 0; nI< src.rows; nI++)	
	for (int nJ = 0; nJ <src.cols; nJ++)
	for (int nK = 0; nK < 3; nK++)
	{
		nVal = (int)(dContrast * SrcData[(nI*step + nJ) * 3 + nK] + dBright);
		if (nVal < 0)
			nVal = 0;
		if (nVal > 255)
			nVal = 255;
		DstData[(nI*step + nJ) * 3 + nK] = nVal;
	}
}
int Red_Gre_Cho(Mat img)
{
	Mat temp = Mat::zeros(img.size(), CV_8UC1);
	cvtColor(img, img, CV_BGR2HSV);
	//int red[3] = { 140, 165, 90 };
	//int redthreshold[3] = { 40, 60, 65 };

	int red[3] = { 80, 130, 180 };
	int redthreshold[3] = { 80, 255, 255 };
	
	int sum = 0;

	for(int i=0;i<img.rows;i++)
	for(int j=0;j<img.cols;j++)
	{
		if ((img.at<Vec3b>(i, j)[0] < red[0] + redthreshold[0]) && (img.at<Vec3b>(i, j)[0] > red[0] - redthreshold[0]) || (img.at<Vec3b>(i, j)[0] < 30) && (img.at<Vec3b>(i, j)[0] > 0))
		if ((img.at<Vec3b>(i, j)[1] < red[1] + redthreshold[1]) && (img.at<Vec3b>(i, j)[1] > red[1] - redthreshold[1]))
		if ((img.at<Vec3b>(i, j)[2] < red[2] + redthreshold[2]) && (img.at<Vec3b>(i, j)[2] > red[2] - redthreshold[2]))
		{
			temp.at<uchar>(i, j) = 255;
			sum++;
		}
		else;
	}

	//imshow("Red_Gre_Cho", temp);

	return sum;
}
/*ͳ����غ���*/
