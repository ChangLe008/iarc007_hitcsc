#ifndef _FILTER_STATISTIC_H_
#define _FILTER_STATISTIC_H_

#include "parameter.h"
//#include "common.h"

extern void Hist_Statistic_Init(Hist_Statistic* statistic);
//ʱ�䣺2016.8.22
//���ܣ���ʼ��ֱ��ͼͳ�ƽṹ��
//������ͳ�ƽṹ��ָ��
//˵��������
extern int Sig_Ch_Greater(Mat img, int threshold);
//ʱ�䣺2016.8.22
//���ܣ�����ĳһ��ͨ��ͼ��ֱ��ͼ�д�����ֵ��ֱ�������
//��������ͨ��ͼ��
//˵��������
extern void Any_Ch_Greater(Mat img, Hist_Statistic *hist_statistic, int threshold, uchar flag);
//ʱ�䣺2016.8.22
//���ܣ�ͳ��ֱ��ͼ���������䲻Ϊ��ĸ���
//������ԭͼ��ֱ��ͼͳ�ƽṹ�壬��ֵ�������Ʊ�־λ
//˵����flag��ʹ�ú궨�壬thanks
extern void Ch_Node_Set(Hist_Statistic *hist_statistic, int start[], int end[], uchar flag);
//ʱ�䣺2016.8.22
//���ܣ��趨ͳ������
//������ֱ��ͼ�ṹ�壬��ʼ��ֵ��������ֵ�������Ʊ�־λ
//˵��������
extern int Sig_Ch_Node_Greater(Mat img, int start, int end ,int threshold);
//ʱ�䣺2016.8.22
//���ܣ���ͨ��ͼ��ڵ�֮�䲻Ϊ������ͳ��
//������ֱ��ͼ�ṹ�壬��ʼ��ֵ��������ֵ����ֵ
//˵��������
extern void Any_Ch_Node_Greater(Mat img, Hist_Statistic *hist_statistic, int threshold, uchar flag);
//ʱ�䣺2016.8.22
//���ܣ���ͨ��ͼ��ڵ�֮�䲻Ϊ������ͳ��
//������ԭͼ��ͳ�ƽṹ�壬��ֵ�������Ʊ�־λ
//˵��������
extern float Sig_Ch_Variance(Mat img);
//ʱ�䣺2016.8.22
//���ܣ�ͳ�Ƶ�ͨ��ͼ��ķ���
//��������ͨ��ͼ��
//˵��������
extern void Any_Ch_Variance(Mat img, Hist_Statistic *hist_statistic, uchar flag);
//ʱ�䣺2016.8.23
//���ܣ�ͳ�Ƹ���ͨ���ķ���
//������ͳ�ƽṹ�壬�����Ʊ�־λ
//˵��������
extern uchar Eight_NP_Cal(Mat img, int x, int y);
//ʱ�䣺2016.8.24
//���ܣ�ͼ��������ֵ����
//������ͼ��x��y
//˵��������
extern float Sig_Ch_Entropy(Mat img);
//ʱ�䣺2016.8.24
//���ܣ���ͨ��ͼ���ά�ؼ���
//��������ͨ��ͼ��
//˵�������뵥ͨ��
extern void Any_Ch_Entropy(Mat img, Hist_Statistic *hist_statistic, uchar flag);
//ʱ�䣺2016.8.24
//���ܣ�����ͨ��ͼ���ά�ؼ���
//������ͼ��
//˵��������
extern int Gre_Bar_Chose(Mat img, int r, Point center);
//ʱ�䣺2016.9.3
//���ܣ���ɫĿ����ϰ��������
//������ͼ����Ұ�뾶����Ұ���ģ�ѡ����ֵ
//˵��������1˵�����ϰ������0˵������ɫ
extern void Bright_Adjust(Mat& src, Mat& dst, double dContrast, double dBright);
//ʱ�䣺2016.9.3
//���ܣ�ͼ�����ȵĵ���
//������Դͼ��Ŀ��ͼ�񡢱����������ӡ�ƽ��
//˵��������
extern int Red_Gre_Cho(Mat img);
//ʱ�䣺2016.9.13
//���ܣ���ɫĿ���ɸѡ
//������ͼ��
//˵��������
#endif
