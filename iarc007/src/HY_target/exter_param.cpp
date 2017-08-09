#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>


int n_boards;
const int board_dt = 10;
/* Num of corners along the x-y direction of chessboard */
int board_w;
int board_h;


void help()
{
	printf("Calibration from disk. Call convention:\n"
		"Where: board_{w,h} are the # of internal corners in the checkerboard\n"
		"       width (board_w) and height (board_h)\n"
		"	when board_w*board_h corners completely and properly found,\n"
		"	Hit \"ESC\" to complete calibration and quit the program.\n");
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"exter_param");
	ros::NodeHandle nh;
	
	CvCapture* capture= cvCreateCameraCapture(7);
	assert(capture);

	help();
	board_w = 11;
	board_h = 7;
	/* Num of the all corners of chessboard */
	int board_n  = board_w * board_h;
	CvSize board_sz = cvSize( board_w, board_h );

	char names[2048];

	n_boards=1;

	cvNamedWindow( "Calibration" );

	/* ALLOCATE STORAGE of corners */
	CvMat* image_points      = cvCreateMat(board_n,2,CV_32FC1);
	CvMat* object_points     = cvCreateMat(board_n,3,CV_32FC1);
	/* Param of camera */
	CvMat *intrinsic = (CvMat*)cvLoad("src/iarc007/doc/Intrinsics.xml");
	CvMat *distortion = (CvMat*)cvLoad("src/iarc007/doc/Distortion.xml");

	CvMat* rotation_vector=cvCreateMat(3,1,CV_32FC1);
	CvMat* translation_vector=cvCreateMat(3,1,CV_32FC1);
	CvMat* rotation_mat=cvCreateMat(3,3,CV_32FC1);
	CvMat* jacobian=cvCreateMat(3,1,CV_32FC1);

	IplImage* image = cvQueryFrame(capture);;
	IplImage* gray_image = 0;
	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
	int corner_count;

	//We'll need this for subpixel accurate stuff
	if(gray_image == 0  && image){
		gray_image = cvCreateImage(cvGetSize(image),8,1);
	}

	while(ros::ok())
	{

		if(!image){
			printf("null image\n");
			break;		
		}

		int found = cvFindChessboardCorners(image,board_sz,corners,&corner_count,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		//Get Subpixel accuracy on those corners
		cvCvtColor(image, gray_image, CV_RGB2GRAY);
		cvFindCornerSubPix(gray_image, corners, corner_count,cvSize(11,11),cvSize(-1,-1), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
		//Draw it
		cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);
		cvShowImage( "Calibration", image );

		char c = cvWaitKey(0);

		if( corner_count == board_n ){
			printf("Found = %d for %s  --- key = %c\n",found,names,c);
		}


		if(c == 27){	
			for( int i=0; i<board_n; ++i )
			{
				CV_MAT_ELEM(*image_points, float,i,0) = corners[i].x;
				CV_MAT_ELEM(*image_points, float,i,1) = corners[i].y;
				CV_MAT_ELEM(*object_points,float,i,0) = (float)(i/board_w);
				CV_MAT_ELEM(*object_points,float,i,1) = (float)(i%board_w);
				CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
			}

			cvFindExtrinsicCameraParams2(object_points,image_points,intrinsic,distortion,rotation_vector,translation_vector );

			cvRodrigues2(rotation_vector,rotation_mat,jacobian=NULL);

			cvSave("src/iarc007/doc/rotation_vector",rotation_vector);
			cvSave("src/iarc007/doc/Rotation.xml",rotation_mat);
			cvSave("src/iarc007/doc/Translation.xml",translation_vector);
			break;
		}
		
		image = cvQueryFrame(capture);
	}

	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&rotation_vector);
	cvReleaseMat(&translation_vector);
	cvReleaseMat(&rotation_mat);
	cvReleaseMat(&jacobian);

	cvReleaseImage(&gray_image);

	delete[] corners;

	cvDestroyAllWindows();

	return 0;
}
