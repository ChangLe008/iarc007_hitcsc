#include "iarc/navigation/threshold_image.h"

void ThresholdWhite(const Mat &_ImageSource, Mat & ImageLine)
{
    Mat ImageSource;
    ImageSource = _ImageSource.clone();
    GaussianBlur(ImageSource, ImageSource, Size(5,5), 0, 0);
    
    Mat ImageGray;
    cvtColor(ImageSource, ImageGray, CV_RGB2GRAY);
    Mat ImageBinary;
    threshold(ImageGray, ImageBinary, 140, 255, CV_THRESH_BINARY);
    
    Mat Element = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(ImageBinary, ImageBinary, Element, Point(-1,-1), 2);
    dilate(ImageBinary, ImageBinary, Element, Point(-1,-1), 2);
    
    Canny(ImageBinary, ImageLine, 50, 150, 3);
}

void ThresholdRed(const Mat& _ImageSource, Mat &_ImageBinary)
{
    Mat ImageSource;
    _ImageSource.copyTo(ImageSource);
    Mat ImageHSV;
    Mat ImageBinary = Mat(ImageSource.rows, ImageSource.cols, CV_8UC1, Scalar(0));
    cvtColor(ImageSource, ImageHSV, CV_BGR2HSV);
    for(int i = 0; i < ImageHSV.rows; i++)
    {
	for(int j = 0; j < ImageHSV.cols; j++)
	{
	float HValue;
	float SValue;
	HValue = ImageHSV.at<Vec3b>(i, j)[0];
	SValue = ImageHSV.at<Vec3b>(i, j)[1];
	if(HValue > 170 || HValue < 10)
	{
	    //ImageBinary.at<uchar>(i, j) = 255;
	    if(abs(SValue - 150) <  50)
	    {
	    ImageBinary.at<uchar>(i, j) = 255;
	    }
	}
	}
    }
    Mat Element = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(ImageBinary, ImageBinary, Element, Point(-1,-1), 2);
    
    dilate(ImageBinary, ImageBinary, Element, Point(-1,-1), 2);
    
    Mat ImageThin;
    ImageThin = thinImage(ImageBinary/255, -1);
    _ImageBinary = ImageThin * 255;
}

void ThresholdGreen(const Mat &_ImageSource, Mat & _ImageBinary)
{
    Mat ImageSource;
    _ImageSource.copyTo(ImageSource);
    Mat ImageHSV;
    Mat ImageBinary = Mat(ImageSource.rows, ImageSource.cols, CV_8UC1, Scalar(0));
    cvtColor(ImageSource, ImageHSV, CV_BGR2HSV);
    for(int i = 0; i < ImageHSV.rows; i++)
    {
	for(int j = 0; j < ImageHSV.cols; j++)
	{
	float HValue;
	float SValue;
	HValue = ImageHSV.at<Vec3b>(i, j)[0];
	SValue = ImageHSV.at<Vec3b>(i, j)[1];
	if(abs(HValue - 70) < 10)
	{
	    //ImageBinary.at<uchar>(i, j) = 255;
	    if(abs(SValue - 150) <  50)
	    {
	    ImageBinary.at<uchar>(i, j) = 255;
	    }
	}
	}
    }
    Mat Element = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(ImageBinary, ImageBinary, Element, Point(-1,-1), 2);
    
    dilate(ImageBinary, ImageBinary, Element, Point(-1,-1), 2);
    
    Mat ImageThin;
    ImageThin = thinImage(ImageBinary/255, -1);
    _ImageBinary = ImageThin * 255;
}
Mat thinImage(const Mat & src, const int maxIterations)
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