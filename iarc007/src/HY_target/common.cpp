#include "iarc007/hy_target/common.h"
#include "iarc007/hy_target/Filter_Statistic.h"

int startnode[6] = { 230, 230, 230, 230, 230, 230 };
int endnode[6] = { 255, 255, 255, 255, 255, 255 };

 
void Letter_Show(char *buf, Point point, Mat& img)
{
	putText(img, buf, point, FONT_HERSHEY_COMPLEX, 0.5f, Scalar(255, 255, 255));
}
float Re_Coin_Degree(Rect r1, Rect r2, int *num)
{
	Point pt1, pt2, pt3, pt4;
	pt1 = { r1.x, r1.y };								//1左上
	pt2 = { r1.x + r1.width, r1.y + r1.height };		//1右下
	pt3 = { r2.x, r2.y };								//2左上
	pt4 = { r2.x + r2.width, r2.y + r2.height };		//2右下

	long s1 = r1.height*r1.width;	//1面积
	long s2 = r2.height*r2.width;	//2面积

	*num = s1 > s2 ? 1 : 2;

	int a1 = MIN(pt1.x, pt3.x);	//最左
	int a2 = MAX(pt2.x, pt4.x);	//最右
	int a3 = MIN(pt1.y, pt3.y);	//最上
	int a4 = MAX(pt2.y, pt4.y);	//最下

	int coinw = a2 - a1 - r1.width - r2.width < 0 ? -(a2 - a1 - r1.width - r2.width) : 0;		//横向重合
	int coinh = a4 - a3 - r1.height - r2.height < 0 ? -(a4 - a3 - r1.height - r2.height) : 0;	//纵向重合

	if (*num == 2)			//r1小
		return (((float)(coinw*1.0)) / ((float)(r1.width*1.0)))* (((float)(coinh*1.0)) / ((float)(r1.height*1.0)));
	else if (*num == 1)		//r2小
		return (((float)(coinw*1.0)) / ((float)(r2.width*1.0)))* (((float)(coinh*1.0)) / ((float)(r2.height*1.0)));
	else
		return 1.0;
}
void Hist_Draw(Mat hist, float maxval, int yoffset)		//0: 模板一 250:模板二 500:测试
{
	int offset = 5;			//zero offset
	int scale = 5;			//width of each reatangle
	float kain = 180;		//height kain

	for (int i = 0; i < HIST_HDIMS; i++)
	{
		float bin_val = hist.at<float>(i);
		int intensity = cvRound(bin_val*kain);
		//cout << "bin_val[" << i << "] = " << bin_val << "\n";
		rectangle(Hist_image, Point(offset + i*scale, IPL_HEIGHT - yoffset + intensity), Point((i + 1)*scale, IPL_HEIGHT - yoffset), Scalar(255, 255, 255), CV_FILLED);
	}
	//cout << "maxval = " << maxval << "\n\n\n";
}
void SVM_Load(CvSVM *svm, char *filename)
{
	printf("SVM加载开始！\n");
	(*svm).load(filename);
	printf("SVM加载结束！\n");
	return;
}
void Obj_MaxMinSize_Cal(float height, int Min_Fit_Met, int Max_Fit_Met, int minoff, int maxoff, int *sizemin, int *sizemax)
{
	*sizemin = Obj_MinSize_Cal(height, Min_Fit_Met, minoff);
	*sizemax = Obj_MaxSize_Cal(height, Max_Fit_Met, maxoff);
	//printf("min = %d\n", *sizemin);
	//printf("max = %d\n", *sizemax);
	if (*sizemin >= *sizemax)
	{
		*sizemin = 0;
		*sizemax = MIN(I_W, I_H);
	}
}
int Obj_MinSize_Cal(float height, int Min_Fit_Met, int minoff)
{
	float molecule = 0.;
	float denominator = 0.;
	float fitsize = 0.;
	if (height > 5.0)
	{
		switch (Min_Fit_Met)
		{
		case MIN_EXP_2_CSOFF:	//0 a*exp(b*h)
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * exp(DATA_FIT[Min_Fit_Met].coefficient[1] * height);
			break;
		case MIN_EXP_2_CSON:	//1
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * exp(DATA_FIT[Min_Fit_Met].coefficient[1] * height);
			break;
		case MIN_EXP_4_CSOFF:	//2
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * exp(DATA_FIT[Min_Fit_Met].coefficient[1] * height) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * exp(DATA_FIT[Min_Fit_Met].coefficient[3] * height);
			break;
		case MIN_EXP_4_CSON:	//3
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * exp(DATA_FIT[Min_Fit_Met].coefficient[1] * height) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * exp(DATA_FIT[Min_Fit_Met].coefficient[3] * height);
			break;
		case MIN_FOU_4_CSOFF:	//4
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] + DATA_FIT[Min_Fit_Met].coefficient[1] * cos(DATA_FIT[Min_Fit_Met].coefficient[3] * height) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * sin(DATA_FIT[Min_Fit_Met].coefficient[3] * height);
			break;
		case MIN_FOU_4_CSON:	//5
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] + DATA_FIT[Min_Fit_Met].coefficient[1] * cos(DATA_FIT[Min_Fit_Met].coefficient[3] * height) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * sin(DATA_FIT[Min_Fit_Met].coefficient[3] * height);
			break;
		case MIN_FOU_6_CSOFF:	//6
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] + DATA_FIT[Min_Fit_Met].coefficient[1] * cos(DATA_FIT[Min_Fit_Met].coefficient[5] * height) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * sin(DATA_FIT[Min_Fit_Met].coefficient[5] * height) +
				DATA_FIT[Min_Fit_Met].coefficient[3] * cos(DATA_FIT[Min_Fit_Met].coefficient[5] * height * 2) +
				DATA_FIT[Min_Fit_Met].coefficient[4] * sin(DATA_FIT[Min_Fit_Met].coefficient[5] * height * 2);
			break;
		case MIN_FOU_6_CSON:	//7
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] + DATA_FIT[Min_Fit_Met].coefficient[1] * cos(DATA_FIT[Min_Fit_Met].coefficient[5] * height) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * sin(DATA_FIT[Min_Fit_Met].coefficient[5] * height) +
				DATA_FIT[Min_Fit_Met].coefficient[3] * cos(DATA_FIT[Min_Fit_Met].coefficient[5] * height * 2) +
				DATA_FIT[Min_Fit_Met].coefficient[4] * sin(DATA_FIT[Min_Fit_Met].coefficient[5] * height * 2);
			break;
		case MIN_POL_3_CSOFF_ROBUSTOFF:	//8
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * height * height + DATA_FIT[Min_Fit_Met].coefficient[1] * height + DATA_FIT[Min_Fit_Met].coefficient[2];
			break;
		case MIN_POL_3_CSOFF_ROBUSTLAR:	//9
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * height * height + DATA_FIT[Min_Fit_Met].coefficient[1] * height + DATA_FIT[Min_Fit_Met].coefficient[2];
			break;
		case MIN_POL_3_CSOFF_ROBUSTBIS:	//10
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * height * height + DATA_FIT[Min_Fit_Met].coefficient[1] * height + DATA_FIT[Min_Fit_Met].coefficient[2];
			break;
		case MIN_POL_3_CSON_ROBUSTOFF:	//11
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * height * height + DATA_FIT[Min_Fit_Met].coefficient[1] * height + DATA_FIT[Min_Fit_Met].coefficient[2];
			break;
		case MIN_POL_3_CSON_ROBUSTLAR:	//12
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * height * height + DATA_FIT[Min_Fit_Met].coefficient[1] * height + DATA_FIT[Min_Fit_Met].coefficient[2];
			break;
		case MIN_POL_3_CSON_ROBUSTBIS:	//13
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * height * height + DATA_FIT[Min_Fit_Met].coefficient[1] * height + DATA_FIT[Min_Fit_Met].coefficient[2];
			break;
		case MIN_POL_4_CSOFF_ROBUSTOFF:	//14
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * pow(height, 3) + DATA_FIT[Min_Fit_Met].coefficient[1] * pow(height, 2) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * height + DATA_FIT[Min_Fit_Met].coefficient[3];
			break;
		case MIN_POL_4_CSON_ROBUSTOFF:	//15
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * pow(height, 3) + DATA_FIT[Min_Fit_Met].coefficient[1] * pow(height, 2) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * height + DATA_FIT[Min_Fit_Met].coefficient[3];
			break;
		case MIN_POL_4_CSON_ROBUSTLAR:	//16
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * pow(height, 3) + DATA_FIT[Min_Fit_Met].coefficient[1] * pow(height, 2) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * height + DATA_FIT[Min_Fit_Met].coefficient[3];
			break;
		case MIN_POL_4_CSOFF_ROBUSTBIS:	//17
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * pow(height, 3) + DATA_FIT[Min_Fit_Met].coefficient[1] * pow(height, 2) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * height + DATA_FIT[Min_Fit_Met].coefficient[3];
			break;
		case MIN_POL_4_CSON_ROBUSTBIS:	//18
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * pow(height, 3) + DATA_FIT[Min_Fit_Met].coefficient[1] * pow(height, 2) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * height + DATA_FIT[Min_Fit_Met].coefficient[3];
			break;
		case MIN_POW_2_CSOFF:	//19
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * pow(height, DATA_FIT[Min_Fit_Met].coefficient[1]);
			break;
		case MIN_POW_3_CSOFF:	//20
			fitsize = DATA_FIT[Min_Fit_Met].coefficient[0] * pow(height, DATA_FIT[Min_Fit_Met].coefficient[1]) + DATA_FIT[Min_Fit_Met].coefficient[2];
			break;
		case MIN_RAT_7_CSOFF:	//21
			molecule = DATA_FIT[Min_Fit_Met].coefficient[0] * pow(height, 4) + DATA_FIT[Min_Fit_Met].coefficient[1] * pow(height, 3) +
				DATA_FIT[Min_Fit_Met].coefficient[2] * pow(height, 2) + DATA_FIT[Min_Fit_Met].coefficient[3] * height +
				DATA_FIT[Min_Fit_Met].coefficient[4];	//分子
			denominator = pow(height, 2) + DATA_FIT[Min_Fit_Met].coefficient[5] * height + DATA_FIT[Min_Fit_Met].coefficient[6];

			if (denominator < 1e-1)
			{
				printf("分子太小！\n");
				fitsize = (float)(SAM_W*1.0);
			}
			else
				fitsize = molecule / denominator;
			break;
		case MIN_SELF:	//46
			printf("公式不知道！\n");
			fitsize = (float)(SAM_W*1.0);
			break;
		default:
			printf("没有这个参数！\n");
			fitsize = (float)(SAM_W*1.0);
			break;
		}
	}
	else
	{
		fitsize = 10.0;
	}
	//printf("偏移之前mmin = %d\n", (int)fitsize);
	return MAX(0, (int)(fitsize - minoff));
}
int Obj_MaxSize_Cal(float height, int Max_Fit_Met, int maxoff)
{
	float molecule = 0.;
	float denominator = 0.;
	float fitsize = 0.;
	if (height > 5.0)
	{
		switch (Max_Fit_Met)
		{
		case MAX_EXP_2_CSOFF:	//22
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * exp(DATA_FIT[Max_Fit_Met].coefficient[1] * height);
			break;
		case MAX_EXP_2_CSON:	//23
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * exp(DATA_FIT[Max_Fit_Met].coefficient[1] * height);
			break;
		case MAX_EXP_4_CSOFF:	//24
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * exp(DATA_FIT[Max_Fit_Met].coefficient[1] * height) +
				DATA_FIT[Max_Fit_Met].coefficient[2] * exp(DATA_FIT[Max_Fit_Met].coefficient[3] * height);
			break;
		case MAX_EXP_4_CSON:	//25
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * exp(DATA_FIT[Max_Fit_Met].coefficient[1] * height) +
				DATA_FIT[Max_Fit_Met].coefficient[2] * exp(DATA_FIT[Max_Fit_Met].coefficient[3] * height);
			break;
		case MAX_FOU_4_CSOFF:	//26
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] + DATA_FIT[Max_Fit_Met].coefficient[1] * cos(DATA_FIT[Max_Fit_Met].coefficient[3] * height) +
				DATA_FIT[Max_Fit_Met].coefficient[2] * sin(DATA_FIT[Max_Fit_Met].coefficient[3] * height);
			break;
		case MAX_FOU_4_CSON:	//27
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] + DATA_FIT[Max_Fit_Met].coefficient[1] * cos(DATA_FIT[Max_Fit_Met].coefficient[3] * height) +
				DATA_FIT[Max_Fit_Met].coefficient[2] * sin(DATA_FIT[Max_Fit_Met].coefficient[3] * height);
			break;
		case MAX_FOU_6_CSOFF:	//28
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] + DATA_FIT[Max_Fit_Met].coefficient[1] * cos(DATA_FIT[Max_Fit_Met].coefficient[5] * height) +
				DATA_FIT[Max_Fit_Met].coefficient[2] * sin(DATA_FIT[Max_Fit_Met].coefficient[5] * height) +
				DATA_FIT[Max_Fit_Met].coefficient[3] * cos(DATA_FIT[Max_Fit_Met].coefficient[5] * height * 2) +
				DATA_FIT[Max_Fit_Met].coefficient[4] * sin(DATA_FIT[Max_Fit_Met].coefficient[5] * height * 2);
			break;
		case MAX_FOU_6_CSON:	//29
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] + DATA_FIT[Max_Fit_Met].coefficient[1] * cos(DATA_FIT[Max_Fit_Met].coefficient[5] * height) +
				DATA_FIT[Max_Fit_Met].coefficient[2] * sin(DATA_FIT[Max_Fit_Met].coefficient[5] * height) +
				DATA_FIT[Max_Fit_Met].coefficient[3] * cos(DATA_FIT[Max_Fit_Met].coefficient[5] * height * 2) +
				DATA_FIT[Max_Fit_Met].coefficient[4] * sin(DATA_FIT[Max_Fit_Met].coefficient[5] * height * 2);
			break;
		case MAX_POL_3_CSOFF_ROBUSTOFF:	//30
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * height * height + DATA_FIT[Max_Fit_Met].coefficient[1] * height + DATA_FIT[Max_Fit_Met].coefficient[2];
			break;
		case MAX_POL_3_CSON_ROBUSTOFF:	//31
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * height * height + DATA_FIT[Max_Fit_Met].coefficient[1] * height + DATA_FIT[Max_Fit_Met].coefficient[2];
			break;
		case MAX_POL_3_CSOFF_ROBUSTLAR:	//32
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * height * height + DATA_FIT[Max_Fit_Met].coefficient[1] * height + DATA_FIT[Max_Fit_Met].coefficient[2];
			break;
		case MAX_POL_3_CSON_ROBUSTLAR:	//33
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * height * height + DATA_FIT[Max_Fit_Met].coefficient[1] * height + DATA_FIT[Max_Fit_Met].coefficient[2];
			break;
		case MAX_POL_3_CSOFF_ROBUSTBIS:	//34
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * height * height + DATA_FIT[Max_Fit_Met].coefficient[1] * height + DATA_FIT[Max_Fit_Met].coefficient[2];
			break;
		case MAX_POL_3_CSON_ROBUSTBIS:	//35
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * height * height + DATA_FIT[Max_Fit_Met].coefficient[1] * height + DATA_FIT[Max_Fit_Met].coefficient[2];
			break;
		case MAX_POL_4_CSOFF_ROBUSTOFF:	//36
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * pow(height, 3) + DATA_FIT[Max_Fit_Met].coefficient[1] * pow(height, 2) +
				DATA_FIT[Max_Fit_Met].coefficient[2] * height + DATA_FIT[Max_Fit_Met].coefficient[3];
			break;
		case MAX_POL_4_CSON_ROBUSTOFF:	//37
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * pow(height, 3) + DATA_FIT[Max_Fit_Met].coefficient[1] * pow(height, 2) +
				DATA_FIT[Max_Fit_Met].coefficient[2] * height + DATA_FIT[Max_Fit_Met].coefficient[3];
			break;
		case MAX_POL_4_CSON_ROBUSTLAR:	//38
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * pow(height, 3) + DATA_FIT[Max_Fit_Met].coefficient[1] * pow(height, 2) +
				DATA_FIT[Max_Fit_Met].coefficient[2] * height + DATA_FIT[Max_Fit_Met].coefficient[3];
			break;
		case MAX_POL_4_CSOFF_ROBUSTBIS:	//39
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * pow(height, 3) + DATA_FIT[Max_Fit_Met].coefficient[1] * pow(height, 2) +
				DATA_FIT[Max_Fit_Met].coefficient[2] * height + DATA_FIT[Max_Fit_Met].coefficient[3];
			break;
		case MAX_POL_4_CSON_ROBUSTBIS:	//40
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * pow(height, 3) + DATA_FIT[Max_Fit_Met].coefficient[1] * pow(height, 2) +
				DATA_FIT[Max_Fit_Met].coefficient[2] * height + DATA_FIT[Max_Fit_Met].coefficient[3];
			break;
		case MAX_POW_2_CSOFF:	//41
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * pow(height, DATA_FIT[Max_Fit_Met].coefficient[1]);
			break;
		case MAX_POW_3_CSOFF:	//42
			fitsize = DATA_FIT[Max_Fit_Met].coefficient[0] * pow(height, DATA_FIT[Max_Fit_Met].coefficient[1]) + DATA_FIT[Max_Fit_Met].coefficient[2];
			break;
		case MAX_RAT_3_CSOFF:	//43
			molecule = DATA_FIT[Max_Fit_Met].coefficient[0] * height + DATA_FIT[Max_Fit_Met].coefficient[1];
			denominator = height + DATA_FIT[Max_Fit_Met].coefficient[2];
			if (denominator < 1e-1)
			{
				printf("分子太小！\n");
				fitsize = (float)(I_H*1.0);
			}
			else
			{
				fitsize = molecule / denominator;
			}
			break;
		case MAX_RAT_4_CSON:	//44
			molecule = DATA_FIT[Max_Fit_Met].coefficient[0] * pow(height, 2) + DATA_FIT[Max_Fit_Met].coefficient[1] * height + DATA_FIT[Max_Fit_Met].coefficient[2];
			denominator = height + DATA_FIT[Max_Fit_Met].coefficient[3];
			if (denominator < 1e-1)
			{
				printf("分子太小\n");
				fitsize = (float)(I_H*1.0);
			}
			else
			{
				fitsize = molecule / denominator;
			}
			break;
		case MAX_RAT_5_CSOFF:	//45
			printf("感觉不对，不用了！\n");
			fitsize = (float)(I_H*1.0);
			break;
		case MAX_SELF:	//47
			break;
		default:
			printf("没有这个参数！\n");
			fitsize = (float)(I_H*1.0);
			break;
		}
	}
	else
	{
		fitsize = 10.0;
	}
	//printf("偏移之前mmax = %d\n", (int)fitsize);
	return MIN(I_H, (int)(fitsize + maxoff));
}
void Vec_Init(void)
{
	found.clear();
	Obj_Out.clear();
}
void Obj_H_Filter(int lmax, int lmin, vector<Rect>&found)
{
	vector<Rect>temp;
	temp.assign(found.begin(), found.end());
	found.clear();
	for (int i = 0; i < temp.size(); i++)
	if (!(temp[i].width >lmax || temp[i].width < lmin))
		found.push_back(temp[i]);
}
void Obj_Filter(Mat src, int lmax, int lmin, vector<Rect>found, vector<Rect>&Obj_Out)
{
	Obj_Out.clear();

	Obj_H_Filter(lmax, lmin, found);

	//cout << "Atfer Obj_H_Filter" << "\n";
	//cout << "Target number: " << found.size() << "\n";
	//for (int i = 0; i < found.size(); i++)
	//{
	//	rectangle(src, found[i].tl(), found[i].br(), Scalar(0, 0, 255), 3);
	//	cout << "Size( " << found[i].width << " ," << found[i].height << " )\n";
	//}
	//imshow("src", src);

//	Hist_Statistic s = { 0 };
//	Hist_Statistic_Init(&s);
//	Ch_Node_Set(&s, startnode, endnode, CHH);
//	
//	int changeflag = 0;
//	
//	while(!changeflag)
//	{
//		changeflag = 1;
//		for (int i = 0; i < found.size(); i++)
//		{
//			Mat foundroi = src(found[i]);
//			resize(foundroi, foundroi, SAMSZIZE, 0., 0., 2);					//取结果矩形框为ROI，并且缩放到样本大小
//			Any_Ch_Variance(foundroi, &s, CHV);	
//			if (s.Is_Num_Stddev[5] <= VarThres )
//			{
//				found.erase(found.begin()+i);
//				changeflag = 0;
//				break;
//			}
//			
//		}
//	}	
		
	Obj_Classify_Filter(found, 0.125);
	Obj_Ell_Filter(found, ELL_CEN, ELL_A, ELL_B);
	Obj_Statistic_Filter(src, found);

	Obj_Out.assign(found.begin(), found.end());
}
void Obj_Classify_Filter(vector<Rect>&found, float threshold)
{
	int deflag = 1;
	if (found.size() > 1)
	{
		while (deflag)
		{
			deflag = 0;
			for (int i = 0; i < found.size(); i++)
			for (int j = i + 1; j < found.size(); j++)
			{
				int nouse = 0;
				if (Re_Coin_Degree(found[i], found[j], &nouse) > threshold)		//则判断两个矩形属于一个目标
				{
					if (i != j)
					{
						Rect newr = Rect_Merge(found[i], found[j]);
						found.push_back(newr);
						found.erase(found.begin() + j);
						found.erase(found.begin() + i);		//j的标号大，必须先擦除j

						deflag = 1;
					}
					//把两个矩形合并
				}
			}
		}
	}
}
void Obj_Ell_Filter(vector<Rect>&found, Point center, float a, float b)
{
	int deflag = 1;
	//cout << "The number of the target  "<< found.size()<<endl;
	if (found.size() > 0)
	{
		while (deflag)
		{
			deflag = 0;
			for (int i = 0; i < found.size(); i++)
			{
				int x1 = found[i].tl().x;
				int x2 = found[i].br().x;
				int y1 = found[i].tl().y;
				int y2 = found[i].br().y;
				if (Ell_Inside_Cal(center, a, b, Point((x1 + x2) / 2, (y1 + y2) / 2)) == 0)	//说明出了视野
				{
					found.erase(found.begin() + i);
					deflag = 1;
				}
			}
		}
	}
	//cout <<"End"<<endl;
}
void Obj_Statistic_Filter(Mat img, vector<Rect>&found)
{
	Red_Out.clear();
	Green_Out.clear();
	Barrier_Out.clear();
	Dec_Wrong.clear();

	Hist_Statistic s = { 0 };
	Hist_Statistic_Init(&s);
	Ch_Node_Set(&s, startnode, endnode, CHH);

	for (int i = 0; i < found.size(); i++)
	{
		Mat foundroi = img(found[i]);
		resize(foundroi, foundroi, SAMSZIZE, 0., 0., 2);					//取结果矩形框为ROI，并且缩放到样本大小
		char name[100] = "foundroi.jpg";
		imwrite(name, foundroi);
		foundroi = imread(name);
		
		//Any_Ch_Node_Greater(foundroi, &s, 0, CHH);	//统计H通道的区间分布
		
		int sum = Gre_Bar_Chose(foundroi, 15, Point(32, 32));
		Any_Ch_Variance(foundroi, &s, CHV);			//方差
		int sum2 = Red_Gre_Cho(foundroi);
		
		//cout << "Variance = " << s.Is_Num_Stddev[5] << endl;
		//cout << "H = " << s.Bet_Node_Count[3] << endl;

//		if (s.Is_Num_Stddev[5] <= VarThres)
//			Dec_Wrong.push_back(found[i]);		
//		else
//		{
//			Obj_Out.push_back(found[i]);
//			if(sum <= 110)
//				Barrier_Out.push_back(found[i]); 
//				
//			else
//			{
//				if (s.Bet_Node_Count[3] >= HueThres)
//					Red_Out.push_back(found[i]);
//				else if(sum >= 150 && s.Bet_Node_Count[3] <= 14)
//					Green_Out.push_back(found[i]);
//			}
//		}

		if(sum2 > 120)
			Red_Out.push_back(found[i]);
		else
		{
			if (s.Is_Num_Stddev[5] <= VarThres)
				Dec_Wrong.push_back(found[i]);
			else
			{
				if(sum > 150)
					Green_Out.push_back(found[i]);
				else
					Barrier_Out.push_back(found[i]);
			}
		}
		
//		if (s.Is_Num_Stddev[5] <= VarThres)
//			Dec_Wrong.push_back(found[i]);		
//		else
//		{
//			Obj_Out.push_back(found[i]);
//			if(sum2 >= 150)
//				Red_Out.push_back(found[i]);
//			else
//			{
//				if (sum <= 110)
//					Barrier_Out.push_back(found[i]);
//				else if(sum >= 150 && sum2 <= 40)
//					Green_Out.push_back(found[i]);
//			}
//		}
	}
}
Rect Rect_Merge(Rect r1, Rect r2)
{
	Rect out;
	out.x = MIN(r1.tl().x, r2.tl().x);
	out.y = MIN(r1.tl().y, r2.tl().y);
	out.width = MAX(r1.br().x, r2.br().x) - out.x;
	out.height = MAX(r1.br().y, r2.br().y) - out.y;

	out.width = out.height = MAX(out.width, out.height);

	if (out.x + out.width > I_W - 1)
		out.x = I_W - out.width - 1;
	if (out.y + out.height > I_H - 1)
		out.y = I_H - out.height - 1;

	if (out.x < 0)
	{
		out.x = 0;
		out.width = I_W - 1;
	}
	if (out.y < 0)
	{
		out.y = 0;
		out.height = I_H - 1;
	}

	out.width = out.height = MIN(out.width, out.height);

	return out;
}
void dianbianhuan(CvMat* R,CvMat* P,double h,int u,int v)
{
	//CvMat* intrinsic=(CvMat*)cvLoad("src/iarc007/doc/Intrinsics.xml");
	CvMat* UV=cvCreateMat(3,1,CV_32FC1);

	CvMat* inverse=cvCreateMat(3,3,CV_32FC1);
	double s;

	//为图像坐标赋值
	cvmSet(UV,0,0,u);
	cvmSet(UV,1,0,v);
	cvmSet(UV,2,0,1);
	/*
			printf("R11= \n");
			for(int i=0;i<3;i++)
			{
				for(int j=0;j<3;j++)
				{
					printf("%f\t",cvmGet(R,i,j));
				}
				printf("\n");
			}*/

	cvInvert(R,inverse,CV_LU);

	//cvGEMM(intrinsic,UV,1,vc,0,UV);
	cvGEMM(inverse,UV,1,NULL,0,UV);

	double uvx,uvy,uvz;
	uvx=cvmGet(UV,0,0);
	uvy=cvmGet(UV,1,0);
	uvz=cvmGet(UV,2,0);
	s=uvz/(h-7.5);

	cvmSet(P,0,0,uvx/s);
	cvmSet(P,1,0,uvy/s);
	cvmSet(P,2,0,uvz/s);


	cvReleaseMat(&inverse);

	cvReleaseMat(&UV);

}
CvPoint DistortionPoint(int x,int y,CvMat* distortion,CvMat* intrinsic)
{
	float k1=cvmGet(distortion,0,0);
	float k2=cvmGet(distortion,1,0);
	float p1=cvmGet(distortion,2,0);
	float p2=cvmGet(distortion,3,0);
	float k3=cvmGet(distortion,4,0);
	float cx=cvmGet(intrinsic,0,2);
	float cy=cvmGet(intrinsic,1,2);
	float fx=cvmGet(intrinsic,0,0);
	float fy=cvmGet(intrinsic,1,1);
	int flag=0;
	CvPoint ud={0};

	for (int nx = -100; nx < 640 + 100; nx++)
	{
		for (int ny = -100; ny < 640 + 100; ny++)
		{
			if(flag==1)
				break;
			float xx = (nx - cx) / fx;
			float yy = (ny - cy) / fy;
			float r2 = pow(xx, 2) + pow(yy, 2);
			float r4 = pow(r2, 2);
			float xxx = xx*(1 + k1*r2 + k2*r4) + 2 * p1*xx*yy + p2*(r2 + 2 * xx*xx);
			float yyy = yy*(1 + k1*r2 + k2*r4) + 2 * p1*xx*yy + p2*(r2 + 2 * yy*yy);
			float xxxx = xxx*fx + cx;
			float yyyy = yyy*fy + cy;
			if(fabs(xxxx-x*1.0)<1.0&&fabs(yyyy-y*1.0)<1.0)
			{
				ud.x=nx;
				ud.y=ny;
				flag=1;
			}
		}
	}
	return ud;

}
Rect Dec_Tra_Connect(Rect rec, float kain)
{
	if (kain > 1.00)
	{
		//cout << "Reeor kain!" << endl;
		//return rec;
	}

	Point center = Point((rec.tl().x + rec.br().x) / 2, (rec.tl().y + rec.br().y) / 2);		//中心位置尽最大可能保持不动

	Rect dst;
	dst.width = (int)(rec.width*kain);		//新的宽
	dst.height = (int)(rec.height*kain);	//新的高
	
	dst.x = center.x - dst.width / 2;		//新的x
	dst.y = center.y - dst.height / 2;		//新的y

	if (dst.width > I_W / 2 || dst.height > I_H / 2)		//如果矩形框太大
		return rec;

	//如果新的坐标碰到边界
	dst.x = MAX(0, dst.x);
	dst.y = MAX(0, dst.y);	//如果小于零
	dst.x = MIN(dst.x, I_W - 1 - dst.width);
	dst.y = MIN(dst.y, I_H - 1 - dst.height);
	return dst;
}
int Tra_Filter(RotatedRect res, float kainthres, float height, int Min_Fit_Met, int minoff, int Max_Fit_Met, int maxoff)
{
	float scale = RoaRec_Len_Scale(res);

	cout << "In Tra_Filter: scale = " << scale << endl;

	Rect bound = res.boundingRect();

	int recmin = MIN(bound.width, bound.height);
	int recmax = MAX(bound.width, bound.height);

	//cout << "In Tra_Filter: recmin = " << recmin << endl;
	cout << "In Tra_Filter: recmax = " << recmax << endl;

	int  minlimit  =  0;
	int  maxlimit = 480;
	if(height > 60)
	{
		//minlimit = (int)(Obj_MinSize_Cal(height, Min_Fit_Met, minoff)*1.0);
		maxlimit = (int)(Obj_MaxSize_Cal(height, Max_Fit_Met, maxoff)*1.0);

		maxlimit=maxlimit*2;
	}
	else
	{
		minlimit = 0;
		maxlimit = 480;
	}

	//minlimit = 0;
	//maxlimit = 480;

	cout << "In Tra_Filter: minlimit = " << minlimit << endl;
	cout << "In Tra_Filter: maxlimit = " << maxlimit << endl;

	//if (scale > kainthres || recmin <minlimit || recmax>maxlimit)
	if (scale > kainthres || recmax>maxlimit)
		return 0;
	else
		return 1;
}
double point2Line(Point2f pt, Point2f lp1, Point2f lp2)
{
	// 两点式公式为(y - y1)/(x - x1) = (y2 - y1)/ (x2 - x1)
	// 化简为一般式为(y2 - y1)x + (x1 - x2)y + (x2y1 - x1y2) = 0  ax + by + c = 0
	double a = lp2.y - lp1.y;
	double b = lp1.x - lp2.x;
	double c = lp2.x * lp1.y - lp1.x * lp2.y;
	// 距离公式为d = |A*x0 + B*y0 + C|/√(A^2 + B^2)
	//double dis = abs(a * pt.x + b * pt.y + c) / sqrt(a * a + b * b);
	double dis = (a * pt.x + b * pt.y + c) / sqrt(a * a + b * b);
	return dis;
}
void PointInArea(vector<Point>&area, Rect rec, Mat img)
{
	area.clear();
	for (int i = 0; i < backproject.rows - 1; i++)
	for (int j = 0; j < backproject.cols - 1; j++)
	{
		uchar ptr1 = img.at<uchar>(i, j);
		if (ptr1 != 0)
			area.push_back(Point(j, i));
	}
}
int Ell_Inside_Cal(Point center, float a, float b, Point test)
{
	/*
	(x - x0)^2 / a^2 + (y - y0)^2 / b^2 = 1		//椭圆公式，小于1表示在内部
	*/
	float ess = 0.1;
	float res = pow((test.x - center.x), 2) / (a * a) + pow((test.y - center.y), 2) / (b * b) + ess;
	if (res < 1.00)
		return 1;
	else
		return 0;
}
float RoaRec_Len_Scale(RotatedRect res)
{
	double ess = 1e-1;
	Point2f pt[4];
	res.points(pt);	//得到四个点
	float distance1 = pow((pt[0].x - pt[1].x), 2) + pow((pt[0].y - pt[1].y), 2);		//0-1
	float distance2 = pow((pt[1].x - pt[2].x), 2) + pow((pt[1].y - pt[2].y), 2);		//1-2

	if (distance1 > distance2 + ess)		//0,1
		return ((float)(distance1 *1.0) / (float)(distance2*1.0));
	else if (distance1 < distance2 - ess)	//1,2
		return ((float)(distance2 *1.0) / (float)(distance1*1.0));
	else
		return 1.00;
}
int OTSU_Th(Mat img)
{
	if (img.channels() != 1)
	{
		cout << "Input Error!" << endl;
		return -1;
	}

	float histogram[256] = { 0. };
	int gray_num = 0;

	for (int i = 0; i < img.rows; i++)
	for (int j = 0; j < img.cols; j++)
		histogram[(int)img.at<uchar>(i, j)]++;

	int size_H = img.rows*img.cols;
	float avgValue_H = 0.;
	int thre_H = 0;
	float maxVariance_H = 0;
	float w_H = 0, u_H = 0;

	for (int i = 0; i < 256; i++)
	{
		histogram[i] = histogram[i] / size_H;
		avgValue_H += i * histogram[i];
	}
	
	for (int i = 0; i < 256; i++)
	{
		w_H += histogram[i];
		u_H += i * histogram[i];

		float t = avgValue_H * w_H - u_H;
		float variance = t * t / (w_H * (1 - w_H));
		if (variance > maxVariance_H)
		{
			maxVariance_H = variance;
			thre_H = i;
		}
	}

	Mat img_binary(img.size(), CV_8UC1);
	threshold(img, img_binary, thre_H, 255, THRESH_BINARY);

	float histogram_L[256] = { 0 };
	int size_L = 0;
	for (int i = 0; i < img.rows; i++)
	for (int j = 0; j < img.cols; j++)
	{
		if (img_binary.at<uchar>(i, j) == 255)
			continue;
		else
		{
			gray_num = img.at<uchar>(i, j);
			histogram_L[gray_num]++;
			size_L++;
		}
	}

	float avgValue_L = 0;
	for (int i = 0; i < 256; i++)
	{
		histogram_L[i] = histogram_L[i] / size_L;
		avgValue_L += i * histogram_L[i];  //整幅图像的平均灰度
	}
	int thre_L = 0;
	float maxVariance_L = 0;
	float w_L = 0, u_L = 0;
	for (int i = 0; i < 256; i++)
	{
		w_L += histogram_L[i];
		u_L += i * histogram_L[i];

		float t = avgValue_L * w_L - u_L;
		float variance = t * t / (w_L * (1 - w_L));
		if (variance > maxVariance_L)
		{
			maxVariance_L = variance;
			thre_L = i;
		}
	}
	return thre_L;
}
