/*
 * fun.cpp
 *
 *  Created on: 2016年8月24日
 *      Author: hou
 */

#include "iarc007/hy_target/parameter.h"

int g_point[310000];
int Qflag;//队列标识 0为不存在，1为存在
int front=0,rear=-1; //队列头，队列尾

int panduan(IplImage* img,IplImage* dst,CvPoint* p)
{

/*	//printf("HSV=%d\t%d\t%d\n",((uchar *)(img->imageData +  320 * img->widthStep ))[240 * img->nChannels + 0],((uchar *)(img->imageData +  320 * img->widthStep ))[240 * img->nChannels + 1],((uchar *)(img->imageData +  320 * img->widthStep ))[240 * img->nChannels + 2]);
*/
	cvZero(dst);
	int red[3];
	int redthreshold[3];
	red[0]=170;
	red[1]=165;
	red[2]=150;
	redthreshold[0]=10;
	redthreshold[1]=55;
	redthreshold[2]=60;
	//red[0]=155;
	//red[1]=130;
	//red[2]=180;
	//redthreshold[0]=25;
	//redthreshold[1]=70;
	//redthreshold[2]=255;

	int green[3];
	int greenthreshold[3];
	green[0]=45;
	green[1]=35;
	green[2]=100;
	greenthreshold[0]=0;
	greenthreshold[1]=0;
	greenthreshold[2]=0;

	int n=0;
	int nx=0;
	int ny=0;
	for(int i=0;i<img->height;i++)
	{
		for(int j=0;j<img->width;j++) 
		{
			if(((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 0]<red[0]+redthreshold[0]&&((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 0]>red[0]-redthreshold[0])
			{
				if(((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 1]<red[1]+redthreshold[1]&&((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 1]>red[1]-redthreshold[1])
				{
					if(((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 2]<red[2]+redthreshold[2]&&((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 2]>red[2]-redthreshold[2])
					{
						((uchar *)(dst->imageData +  i * dst->widthStep ))[j] =100;
						nx=nx+j;
						ny=ny+i;
						n++;
					}
				}
			}

			else if(((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 0]<green[0]+greenthreshold[0]&&((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 0]>green[0]-greenthreshold[0])
			{
				if(((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 1]<green[1]+greenthreshold[1]&&((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 1]>green[1]-greenthreshold[1])
				{
					if(((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 2]<green[2]+greenthreshold[2]&&((uchar *)(img->imageData +  i * img->widthStep ))[j * img->nChannels + 2]>green[2]-greenthreshold[2])
					{
						((uchar *)(dst->imageData +  i * dst->widthStep ))[j] =200;
					}
				}
			}
		}
	}
	if(n!=0)
	{
		nx=nx/n;
		ny=ny/n;
		p->x=nx;
		p->y=ny;
	}
	cvNamedWindow("dst");
	cvShowImage("dst",dst);
	return n;
}

//连通域分析
int liantongyu(IplImage* src,float cx[],float cy[],double h)
{
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contour = 0;

    int contour_num = cvFindContours(src, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //cvZero(dst);        // 清空数组
    CvSeq *_contour = contour;
    double maxarea = 60000000/(h*h);
    double minarea = 150;
    int m = 0;
    for( ; contour != 0; contour = contour->h_next )
    {

        double tmparea = fabs(cvContourArea(contour));
        //printf("area=%f\n",tmparea);
        if(tmparea < minarea||tmparea > maxarea)
        {

        	cvSeqRemove(contour, 0); // 删除面积小于设定值的轮廓   ?？？？
            continue;
        }
        CvRect aRect = cvBoundingRect( contour, 0 );
        float g=((float)aRect.width/aRect.height);
        if (g<1.0/5||g>5)
        {
        	cvSeqRemove(contour, 0); //删除宽高比例小于设定值的轮廓
            continue;
        }

        int u=0;
        float xx=0,yy=0;
        for(int j = 0; j < contour->total; j++)    // 提取一个轮廓的所有坐标点
        {
        	u++;
        	CvPoint *pt = (CvPoint*) cvGetSeqElem(contour, j);   // 得到一个轮廓中一个点的函数cvGetSeqElem
        	yy=pt->y+yy;
        	xx=pt->x+xx;
        }
////printf("u=%d   xx=%f\tyy%f\n",u,xx,yy);
        xx=xx/(float)u;
        yy=yy/(float)u;
        cx[m]=xx;
        cy[m]=yy;
        m++;
////printf("%f\t,%f\n",xx,yy);
/*	for(int i=0;i<m;i++)
	{
		//printf("cx=%f\tcy=%f\n",cx[i],cy[i]);
	}
*/
  }
   // //printf("m=%d\n",m);


    cvReleaseMemStorage(&storage);
	return m;
}

void creat(int x,int y)
{
	if(Qflag==1)
	{
		return;
	}
	else
	{
		Qflag=1;
		rear++;
		g_point[rear]=(y<<16)+x;
	}
}

void pushback(int x,int y)
{
	if(Qflag==0)
	{
		return;
	}
	else
	{
		rear++;
		g_point[rear]=(y<<16)+x;
	}
}

//如果队列为空，返回0
int empty()
{
	if(rear<front)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void clear()
{
	front=0;
	rear=-1;
	Qflag=0;
}

int RegionGrow(const IplImage* src,IplImage* dst, int u,int v,int threshold,CvPoint* p,double h)
{
	int x,y,g,n;
	int  a,b,c;
	int  xx[10],yy[10];
	int flag;

	IplImage* img=cvCreateImage(cvGetSize(src),IPL_DEPTH_8U,3);
	IplImage* imgH=cvCreateImage(cvGetSize(src),IPL_DEPTH_8U,1);
	IplImage* backup=cvCreateImage(cvGetSize(src),IPL_DEPTH_8U,1);
	cvCopy(dst,backup);

	//cvCvtColor(src,img,CV_BGR2HSV);
	cvCvtColor(src,imgH,CV_BGR2GRAY);
	//cvSmooth(img,img,CV_MEDIAN);
	//cvSplit(img,0,0,imgH,0);
	//memset(dst->imageData,0,dst->width*dst->height*sizeof(uchar));
	for(int i=0;i<1;i++)
	{
		creat(u, v);
		((uchar*)(dst->imageData +v * dst->widthStep))[u]=255;

		a=(int)((img->imageData +v * img->widthStep)[3*u+0]);
		b=(int)((img->imageData +v * img->widthStep)[3*u+1]);
		c=  (int)((img->imageData +v * img->widthStep)[3*u+2]);

		if((int)((img->imageData +v * img->widthStep)[3*u+0])<0)
		{
			a+=256;
		}
		if( (int)((img->imageData +v * img->widthStep)[3*u+2])<0)
		{
			c+=256;
		}
		if((int)((img->imageData +v * img->widthStep)[3*u+1])<0)
		{
			b+=256;
		}

		xx[i]=u;
		yy[i]=v;
		n=1;

		if(empty())
		{
			while(front!=(rear+1))// while(iter_cont != cont_pts_pre.end())
			{
				x=g_point[front]&65535;
				y=(g_point[front]>>16)&65535;

				if(!(x-1<0))
				{
					if(((uchar*)(dst->imageData +y* dst->widthStep))[x-1]==0)
					{
						uchar last=((uchar*)(imgH->imageData + y*imgH->widthStep))[x];
						uchar next= ((uchar*)(imgH->imageData + y*imgH->widthStep))[x-1];

						if(last<0)
						{
							last+=256;
						}
						if(next<0)
						{
							next+=256;
						}
						g=last-next;
						if(abs(g) < threshold)      //满足阈值要求
						{
							pushback(x-1, y);//cont_pts_pre.push_back( g_point(x-1, y, -2));
							((uchar*)(dst->imageData +y* dst->widthStep))[x-1]=255;
							if((int)((img->imageData +y * img->widthStep)[3*(x-1)+0])<0)
							{
								a+=256;
							}
							if( (int)((img->imageData +y * img->widthStep)[3*(x-1)+2])<0)
							{
								c+=256;
							}
							if((int)((img->imageData +y * img->widthStep)[3*(x-1)+1])<0)
							{
								b+=256;
							}
							a=a+(int)((img->imageData +y* img->widthStep)[(x-1)*3+0]);
							b=b+(int)((img->imageData +y* img->widthStep)[(x-1)*3+1]);
							c=c+(int)((img->imageData +y* img->widthStep)[(x-1)*3+2]);

							xx[i]=xx[i]+x-1;
							yy[i]=yy[i]+y;
							n++;
						}
						else
						{
							((uchar*)(dst->imageData +y * dst->widthStep))[x-1]=0;
						}
					}
				}

	            if(!(y+1>=imgH->height))
	            {
	            	if(((uchar*)(dst->imageData +(y+1) * dst->widthStep))[x]==0)
	            	{
	            		uchar last=((uchar*)(imgH->imageData + y*imgH->widthStep))[x];
	            		uchar next= ((uchar*)(imgH->imageData + (y+1)*imgH->widthStep))[x];

	            		if(last<0)
	            		{
	            			last+=256;
	            		}
	            		if(next<0)
	            		{
	            			next+=256;
	            		}
	            		g=last-next;

	            		if(abs(g) < threshold)      //满足阈值要求
	            		{
	            			pushback(x, y+1);//cont_pts_pre.push_back( g_point(x, y+1, -2));
	            			((uchar*)(dst->imageData +(y+1) * dst->widthStep))[x]=255;
	            			if((int)((img->imageData +(y+1) * img->widthStep)[3*x+0])<0)
	            			{
	            				a+=256;
	            			}
	            			if( (int)((img->imageData +(y+1) * img->widthStep)[3*x+2])<0)
	            			{
	            				c+=256;
	            			}
	            			if((int)((img->imageData +(y+1) * img->widthStep)[3*x+1])<0)
	            			{
	            				b+=256;
	            			}
	            			a=a+(int)((img->imageData +(y+1)* img->widthStep)[x*3+0]);
	            			b=b+(int)((img->imageData +(y+1)* img->widthStep)[x*3+1]);
	            			c=c+(int)((img->imageData +(y+1)* img->widthStep)[x*3+2]);

	            			xx[i]=xx[i]+x;
	            			yy[i]=yy[i]+y+1;
	            			n++;
	            		}
	            		else
	            		{
	            			((uchar*)(dst->imageData +(y+1) * dst->widthStep))[x]=0;
	            		}
	            	}
	            }


	            if(!(x+1>imgH->width))
	            {
	            	if(((uchar*)(dst->imageData +y * dst->widthStep))[x+1]==0)
	            	{
	            		uchar last=((uchar*)(imgH->imageData + y*imgH->widthStep))[x];
	            		uchar next= ((uchar*)(imgH->imageData + y*imgH->widthStep))[x+1];
	            		if(last<0)
	            		{
	            			last+=256;
	            		}
	            		if(next<0)
	            		{
	            			next+=256;
	            		}
	            		g=last-next;
	            		if(abs(g) < threshold)      //满足阈值要求
	            		{
	            			pushback(x+1, y);//cont_pts_pre.push_back( g_point(x+1, y, -2));
	            			((uchar*)(dst->imageData +y * dst->widthStep))[x+1]=255;
	            			if((int)((img->imageData +y* img->widthStep)[3*(x+1)+0])<0)
	            			{
	            				a+=256;
	            			}
	            			if( (int)((img->imageData +y * img->widthStep)[3*(x+1)+2])<0)
	            			{
									c+=256;
	            			}
	            			if((int)((img->imageData +y * img->widthStep)[3*(x+1)+1])<0)
	            			{
	            				b+=256;
	            			}

	            			a=a+(int)((img->imageData +y* img->widthStep)[3*(x+1)+0]);
	            			b=b+(int)((img->imageData +y* img->widthStep)[3*(x+1)+1]);
	            			c=c+(int)((img->imageData +y* img->widthStep)[3*(x+1)+2]);

	            			xx[i]=xx[i]+x+1;
	            			yy[i]=yy[i]+y;
	            			n++;

	            		}
	            		else
	            		{
	            			((uchar*)(dst->imageData +y * dst->widthStep))[x+1]=0;
	            		}

	            	}

	            }


	            if(!(y-1<0))
	            {
	            	if(((uchar*)(dst->imageData +(y-1) * dst->widthStep))[x]==0)
	            	{
	            		uchar last=((uchar*)(imgH->imageData + y*imgH->widthStep))[x];
	            		uchar next= ((uchar*)(imgH->imageData + (y-1)*imgH->widthStep))[x];
	            		if(last<0)
	            		{
	            			last+=256;
	            		}
	            		if(next<0)
	            		{
	            			next+=256;
	            		}
	            		g=last-next;
	            		if(abs(g) < threshold)      //满足阈值要求
	            		{
	            			pushback(x, y-1);//cont_pts_pre.push_back( g_point(x, y-1, -2));
	            			((uchar*)(dst->imageData +(y -1)* dst->widthStep))[x]=255;

	            			if((int)((img->imageData +(y-1) * img->widthStep)[3*x+0])<0)
	            			{
	            				a+=256;
	            			}
	            			if( (int)((img->imageData +(y-1)  * img->widthStep)[3*x+2])<0)
	            			{
	            				c+=256;
	            			}
	            			if((int)((img->imageData +(y-1)  * img->widthStep)[3*x+1])<0)
	            			{
	            				b+=256;
	            			}
	            			a=a+(int)((img->imageData +(y-1)* img->widthStep)[x*3+0]);
	            			b=b+(int)((img->imageData +(y-1)* img->widthStep)[x*3+1]);
	            			c=c+(int)((img->imageData +(y-1)* img->widthStep)[x*3+2]);

	            			xx[i]=xx[i]+x;
	            			yy[i]=yy[i]+y-1;
	            			n++;
	            		}
	            		else
	            		{
	            			((uchar*)(dst->imageData +(y -1)* dst->widthStep))[x]=0;
	            		}
	            	}
	            }

	            front++;//iter_cont ++;
			}
			clear();
		}
		a=a/n;
		b=b/n;
		c=c/n;
		xx[i]=xx[i]/n;
		yy[i]=yy[i]/n;
		clear();
	}
	p->x=xx[0];
	p->y=yy[0];
	////printf("asdfadfa n=%d %d\t%d\n",n,p->x=xx[0],p->y=yy[0]);

	//cvNamedWindow("H");
	//cvShowImage("H",dst);

//
//	if(n>((int)(10*5*20000000/(h*h)))||n<(20000000/(h*h*10)))
//	{
//		//printf("nnnn===%f\t n=%d\n",20000000/(h*h),n);
//		cvCopy(backup,dst);
//		p->x=0;
//		p->y=0;
//
//	}
	//else
	{
		//printf("c\n");
		p->x=xx[0];
		p->y=yy[0];
	}

	//p->x=xx[0];
	//p->y=yy[0];
	cvReleaseImage(&img);
	cvReleaseImage(&imgH);
	cvReleaseImage(&backup);
	return 0;

}

void imageprocessing(IplImage* img,PicPoint *picp,double h)
{
	IplImage* dst=cvCreateImage(cvGetSize(img),8,1);
	IplImage* imgH=cvCreateImage(cvGetSize(img),8,1);
	IplImage* src=cvCreateImage(cvGetSize(img),8,3);
	cvCvtColor(img,src,CV_BGR2HSV);
	cvZero(dst);
	int ret=1;
	CvPoint* p=(CvPoint*)malloc(1*sizeof(CvPoint));
	ret=panduan(src,dst,p);
	int temp;

	temp=(int)h;
	int num=0;
	if(ret<(60000000/(temp*temp*10))||ret>(60000000/(temp*temp)))
	{
	//printf("ddd\n");
		num=0;
	}
	else
	{
		num=1;
	}

	float cx[50];
	float cy[50];
	num=liantongyu(dst,cx,cy,h);

	//printf("well");
	printf("111number=%d\n",num);

	picp->num=num;

	for(int i=0;i<num;i++)
	{
		cvZero(dst);
		ret=RegionGrow(img,dst,(int)cx[i],(int)cy[i],3,p,h);
		//cvShowImage("dst",dst);
		//cvWaitKey(0);

		//printf("2222  %d\t%d\n",(int)cx[i],(int)cy[i]);
		if(p->x!=0&&p->y!=0)
		{
			picp->u[i]=p->x;
			picp->v[i]=p->y;
		}
	}
	for(int i=0;i<num;i++)
	{
		//printf("111  %d\t%d\n",picp->u[i],picp->v[i]);
	}

	free(p);
	cvReleaseImage(&dst);
	cvReleaseImage(&imgH);
	cvReleaseImage(&src);
	return ;
}

int featuredetector(IplImage* img,CvPoint2D32f* corners,double h,int u,int v)
{

	IplImage* img_gray = cvCreateImage(cvGetSize( img ),8, 1) ;
	IplImage* eig_image = cvCreateImage(cvGetSize( img_gray ),8, 1);
	IplImage* temp_image =  cvCreateImage(cvGetSize( img_gray ),8, 1);
	const int MAX_CORNERS = 1000;
	int corner_count = MAX_CORNERS;
	double quality_level = 0.05; //OR 0.01
	double min_distance = 1000/h;
	int rect=3000/h;

	rect=rect*2;
	if(rect>160)
	{
		rect=160;
	}
	//printf("vrect=%d\n",rect);
	cvCvtColor(img,img_gray,CV_BGR2GRAY);
	//cvCopy(img,img_gray);
	if(u-rect*4/2<0)
	{
		u=rect*4/2;
	}
	else if(u+rect*4/2>640)
	{
		u=640-rect*4/2;
	}
	else
	{
	;
	}

	if(v-rect*3/2<0)
	{
		v=rect*3/2;
	}
	else if(v+rect*3/2>480)
	{
		v=480-rect*3/2;
	}
	else
	{
	;
	}
	cvSetImageROI(img_gray,cvRect(u-rect*4/2,v-rect*3/2,rect*4,rect*3));

	//	IplImage* img_copy = cvCreateImage(cvGetSize( img_gray ),8, 1);
	//	cvCopy( img_gray ,img_copy) ;//a copy of img
	cvCopy( eig_image ,temp_image) ;

	cvNamedWindow("n");

	cvGoodFeaturesToTrack(
	img_gray,
	eig_image,
	temp_image,
	corners,
	&corner_count,
	quality_level,
	min_distance
	);

	//draw corners from cvGoodFeaturesToTrack on "img"


	int half_win_size = 3;//the window size will be 3+1+3=7
	int iteration = 20;
	double epislon = 0.05;
	cvFindCornerSubPix(
	img_gray,
	corners,
	corner_count,
	cvSize(half_win_size,half_win_size),
	cvSize(-1,-1),//no ignoring the neighbours of the center corner
	cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,iteration,epislon)
	);
	for (int i=0;i<corner_count;i++)
	{
		cvLine(
		img_gray,
		//  cvPoint(corners[i].x,corners[i].y) ,
		// cvPoint(corners[i].x,corners[i].y),
		cvPoint(corners[i].x,corners[i].y) ,
		cvPoint(corners[i].x,corners[i].y),
		CV_RGB(255,0,0),
		5
		);
	}
	cvShowImage("n",img_gray);
	//draw subpix corners on "img_copy"


	cvResetImageROI(img_gray);
	for(int i=0;i<corner_count;i++)
	{
		corners[i].x=corners[i].x+u-rect*4/2;
		corners[i].y=corners[i].y+v-rect*4/2;

	}

	cvReleaseImage(&img_gray);
	//cvReleaseImage(&img_copy);
	cvReleaseImage(&eig_image);
	cvReleaseImage(&temp_image);

	return corner_count;
}

