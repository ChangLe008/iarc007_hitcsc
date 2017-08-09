#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stdio.h>
#include <stdlib.h>

int n_boards=30;
const int board_dt=1;
int board_w=11;
int board_h=7;

void help()
{
	printf("Calibration from disk. Call convention:\n"
		"hit \"p\" to choose properly detected image\n"
		"hit \"q\" to quit calibration\n"
		"when collect 10 properly detected images, calibration succede\n"
	);
}

int calibrate(IplImage* image,CvCapture* capture)
{
	int board_n=board_w*board_h;
	CvSize board_sz=cvSize(board_w,board_h);

	cvNamedWindow("Calibration");
	CvMat* image_points=cvCreateMat(n_boards*board_n,2,CV_32FC1);
	CvMat* object_points=cvCreateMat(n_boards*board_n,3,CV_32FC1);
	CvMat* point_counts=cvCreateMat(n_boards,1,CV_32SC1);
	CvMat* intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat* distortion_coeffs=cvCreateMat(5,1,CV_32FC1);

	CvPoint2D32f* corners=new CvPoint2D32f[board_n];
	int corner_count;
	int successes=0;
	int step,frame=0;

	char c;
	IplImage* gray_image=cvCreateImage(cvGetSize(image),8,1);

	while(successes<n_boards)
	{
		image=cvQueryFrame(capture);

		if(frame++%board_dt==0)
		{
			int found=cvFindChessboardCorners(image,board_sz,corners,&corner_count,CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);

			cvCvtColor(image,gray_image,CV_BGR2GRAY);
			cvFindCornerSubPix(gray_image,corners,corner_count,cvSize(11,11),cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			cvDrawChessboardCorners(image,board_sz,corners,corner_count,found);
			
			printf("Num of images collected = %d\n",successes);

			cvShowImage("Calibration",image);

			c = cvWaitKey(0);
			if(c!='p'){
				continue;
			}

			if(corner_count==board_n)
			{
				step=successes*board_n;
				for(int i=step,j=0;j<board_n;i++,j++)
				{
					CV_MAT_ELEM(*image_points,float,i,0)=corners[j].x;
					CV_MAT_ELEM(*image_points,float,i,1)=corners[j].y;
					CV_MAT_ELEM(*object_points,float,i,0)=j/board_w;
					CV_MAT_ELEM(*object_points,float,i,1)=j%board_w;
					CV_MAT_ELEM(*object_points,float,i,2)=0.0f;
				}
				CV_MAT_ELEM(*point_counts,int,successes,0)=board_n;
				successes++;
			}
		}

		if(c=='q'){
			cvReleaseMat(&intrinsic_matrix);
			cvReleaseMat(&distortion_coeffs);

			cvReleaseMat(&object_points);
			cvReleaseMat(&image_points);
			cvReleaseMat(&point_counts);

			delete[] corners;

			cvReleaseImage(&gray_image);

			cvDestroyAllWindows();

			return -1;
		}
	}

	CvMat* object_points2=cvCreateMat(successes*board_n,3,CV_32FC1);
	CvMat* image_points2=cvCreateMat(successes*board_n,2,CV_32FC1);
	CvMat* point_counts2=cvCreateMat(successes,1,CV_32SC1);

	for(int i=0;i<successes*board_n;i++)
	{
		CV_MAT_ELEM(*image_points2,float,i,0)=CV_MAT_ELEM(*image_points,float,i,0);
		CV_MAT_ELEM(*image_points2,float,i,1)=CV_MAT_ELEM(*image_points,float,i,1);
		CV_MAT_ELEM(*object_points2,float,i,0)=CV_MAT_ELEM(*object_points,float,i,0);
		CV_MAT_ELEM(*object_points2,float,i,1)=CV_MAT_ELEM(*object_points,float,i,1);
		CV_MAT_ELEM(*object_points2,float,i,2)=CV_MAT_ELEM(*object_points,float,i,2);

	}

	for(int i=0;i<successes;i++)
	{
		CV_MAT_ELEM(*point_counts2,int,i,0)=CV_MAT_ELEM(*point_counts,int,i,0);

	}
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);

	CvMat* cam_rotation_all = cvCreateMat( successes, 3, CV_32FC1);
	CvMat* cam_translation_vector_all = cvCreateMat( successes,3, CV_32FC1);
	cvCalibrateCamera2(object_points2,image_points2,point_counts2,cvGetSize(image),intrinsic_matrix,distortion_coeffs,cam_rotation_all,cam_translation_vector_all,0);

	cvSave("/home/hitcsc/catkin_ws/src/iarc007/doc/Intrinsics.xml",intrinsic_matrix);
	cvSave("/home/hitcsc/catkin_ws/src/iarc007/doc/Distortion.xml",distortion_coeffs);
	cvSave("/home/hitcsc/catkin_ws/src/iarc007/doc/cam_rotation_all.xml",cam_rotation_all);
	cvSave("/home/hitcsc/catkin_ws/src/iarc007/doc/cam_translation_vector_all.xml",cam_translation_vector_all);
	
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);

	cvReleaseMat(&object_points2);
	cvReleaseMat(&image_points2);
	cvReleaseMat(&point_counts2);

	cvReleaseMat(&cam_rotation_all);
	cvReleaseMat(&cam_translation_vector_all);

	delete[] corners;

	cvReleaseImage(&gray_image);

	cvDestroyAllWindows();

	return 0;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"calibration");
	ros::NodeHandle nh;

	/*if(argc!=4)
	{
		printf("ERROR:Wrong number of input parameters\n");
		return -1;
	}
	board_w = atoi(argv[1]);
	board_h = atoi(argv[2]);
	n_boards=atoi(argv[3]);*/

	CvCapture* capture=cvCreateCameraCapture(7);
	assert(capture);
	IplImage* image=cvQueryFrame(capture);

	
	int cali = calibrate(image,capture);
	if(cali == -1){
		printf("calibration failed!\n");
		return 0;		
	}
	else{
		printf("calibration ok!\n");
	}	

	CvMat* intrinsic=(CvMat*)cvLoad("src/iarc007/doc/Intrinsics.xml");
	CvMat* distortion=(CvMat*)cvLoad("src/iarc007/doc/Distortion.xml");

	IplImage* mapx=cvCreateImage(cvGetSize(image),IPL_DEPTH_32F,1);
	IplImage* mapy=cvCreateImage(cvGetSize(image),IPL_DEPTH_32F,1);
	cvInitUndistortMap(intrinsic,distortion,mapx,mapy);

	cvNamedWindow("Undistort",0);
	cvNamedWindow("Calibrate",0);
	
	IplImage* t=cvCloneImage(image);

	while(image && ros::ok())
	{
		t=cvCloneImage(image);
		
		cvRemap(image,t,mapx,mapy);

		cvShowImage("Undistort",image);
		cvShowImage("Calibrate",t);

		char c=cvWaitKey(15);

		if(c==27){
			break;
		}

		image=cvQueryFrame(capture);
	}

	cvReleaseImage(&t);
	cvReleaseImage(&mapx);
	cvReleaseImage(&mapy);

	cvDestroyAllWindows();

	return 0;
}
