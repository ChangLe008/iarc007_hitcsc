#include "iarc007/mjh_pos/common.h"
#include "iarc007/mjh_pos/threshold_image.h"
#include <algorithm>

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
  void RedLine(const Mat &ImageSource, Mat &ImageBinary);
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
void  ThresholdGreenLine(const Mat &ImageSource, Mat &ImageBinary)
{
	Mat ImageHSV;
	cvtColor(ImageSource, ImageHSV, CV_BGR2HSV);
	for(int i = 0; i < ImageHSV.rows; i++)
	{
		for(int j = 0; j < ImageHSV.cols; j++)
		{
			//cout << "i: " << i << "\t" << "j: " << j << endl;
			float HValue;
			float SValue;
			HValue = ImageHSV.at<Vec3b>(i, j)[0];
			SValue = ImageHSV.at<Vec3b>(i, j)[1];
			if(fabs(HValue-70) < 40)
			{
				if(fabs(SValue-130) < 40)
				{
					ImageBinary.at<uchar>(i, j) = 255;
				}
			}
		}
	}
	
	Mat Element = getStructuringElement(MORPH_RECT, Size(3, 3));
	erode(ImageBinary, ImageBinary, Element, Point(-1,-1), 1);
	Canny(ImageBinary, ImageBinary, 10,100);
}
void  ThresholdRedLine(const Mat &ImageSource, Mat &ImageBinary)
{
	Mat ImageHSV;
	cvtColor(ImageSource, ImageHSV, CV_BGR2HSV);
	for(int i = 0; i < ImageHSV.rows; i++)
	{
		for(int j = 0; j < ImageHSV.cols; j++)
		{
			//cout << "i: " << i << "\t" << "j: " << j << endl;
			float HValue;
			float SValue;
			HValue = ImageHSV.at<Vec3b>(i, j)[0];
			SValue = ImageHSV.at<Vec3b>(i, j)[1];
			//cout << "H: " << HValue << "\t" << "S: " << SValue << endl; 
			if( HValue > 120)
			{
				if(SValue > 70)
				{
					ImageBinary.at<uchar>(i, j) = 255;
				}
			}
		}
	}
	
	Mat Element = getStructuringElement(MORPH_RECT, Size(3, 3));
	erode(ImageBinary, ImageBinary, Element, Point(-1,-1), 1);
	Canny(ImageBinary, ImageBinary, 10,100);
}

/*void  ThresholdWhiteLine(const Mat &ImageSource, Mat &ImageBinary)
{	
	Mat ImageGray;
	cvtColor(ImageSource, ImageGray, CV_RGB2GRAY);
	
	Mat ImageCanny;
	Canny(ImageGray, ImageCanny, 10,100);
	imshow("边缘", ImageCanny);
	vector<int> GrayValue;
	long int NotZeroNumber = 0;
	for(int i = 0; i < ImageRows; i ++)
	{
		for(int j = 0; j < ImageCols; j++)
		{
			if(ImageCanny.at<uchar>(i,j) != 0)
			{
				GrayValue.push_back(ImageGray.at<uchar>(i, j));
				NotZeroNumber++;
			}
		}
	}
	sort(GrayValue.begin(), GrayValue.end());
	int ThresholdValue;
	if(NotZeroNumber > 5000)
	{
		ThresholdValue = GrayValue[NotZeroNumber * 0.5];
	}
	else
	{
		ThresholdValue = 255;
	}
	threshold( ImageGray, ImageBinary, ThresholdValue, 255, THRESH_BINARY);
	Mat Element = getStructuringElement(MORPH_RECT, Size(3, 3));
	erode(ImageBinary, ImageBinary, Element, Point(-1,-1), 1);
	Canny(ImageBinary, ImageBinary, 10,100);
	//imshow("边缘", ImageBinary);
	//waitKey(0);
}*/


void WhiteBinary(const Mat &ImageSource, Mat &ImageBinary, int ThinTime)
{	
	Mat ImageGray;
	cvtColor(ImageSource, ImageGray, CV_RGB2GRAY);
	
	Mat ImageCanny;
	Canny(ImageGray, ImageCanny, 10,100);
	imshow("边缘", ImageCanny);
	waitKey(1);
	threshold(ImageGray, ImageBinary, 140, 255, THRESH_BINARY);
	
	Mat Element = getStructuringElement(MORPH_RECT, Size(3, 3));
	
	erode(ImageBinary, ImageBinary, Element, Point(-1,-1), 1);
	//imshow("二值图", ImageBinary);
	
	Mat ImageThin;
	ImageThin = thinImage(ImageBinary/255, ThinTime); //细化次数与高度有关
	
	ImageBinary = ImageThin*255;
	erode(ImageBinary, ImageBinary, Element, Point(-1,-1), 1);
}
void ThresholdWhiteLine(const Mat &ImageSource, Mat &ImageBinary, int ThinTime)
{
	WhiteBinary(ImageSource, ImageBinary, ThinTime);

	Mat ImageThin;
	ImageThin = thinImage(ImageBinary/255, -1); 
	ImageBinary = ImageThin * 255;
	imshow("二值图", ImageBinary);
	waitKey(1);
	
	/*Mat ImageGray;
	cvtColor(ImageSource, ImageGray, CV_RGB2GRAY);
	
	Canny(ImageGray, ImageBinary, 10,100);
	imshow("边缘", ImageBinary);
	waitKey(1);*/
}