#include <ros/ros.h>

#include <iostream>
#include <fstream>
//#include <io.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

 const int board_w = 7;
 const int board_h = 6;
 const int boardSize = 20; //mm
 const int success = 30;

int main(int argc, char** argv)
{   

	ros::init(argc,argv,"calibration");
	ros::NodeHandle nh;
    //string filePath = ".\\720PPcalib\\front";
    //ector<string> files;

    ////获取该路径下的所有文件
    //getFiles(filePath, files);
	VideoCapture capture(7);
	
	for(int i=1;i<argc;i++) 
	{
		;
	}
	
    const int NPoints = board_w * board_h;//棋盘格内角点总数

    Mat image,grayimage;
    Size ChessBoardSize = cv::Size(board_w, board_h);
    vector<Point2f> tempcorners;
	
    int flag = 0;
    flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    //flag |= cv::fisheye::CALIB_CHECK_COND;
    flag |= cv::fisheye::CALIB_FIX_SKEW;
    //flag |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;

    vector<Point3f> object;
    for (int j = 0; j < NPoints; j++)
    {
        object.push_back(Point3f((j % board_w) * boardSize, (j / board_w) * boardSize, 0)); 
    }

    cv::Matx33d intrinsics;//z:相机内参
    cv::Vec4d distortion_coeff;//z:相机畸变系数

    vector<vector<Point3f> > objectv;
    vector<vector<Point2f> > imagev;

    Size corrected_size(640, 480);
    Mat mapx, mapy;
    Mat corrected;

    ofstream intrinsicfile("doc/intrinsics_front1103.txt");
    ofstream disfile("doc/dis_coeff_front1103.txt");
    
    int num = 0;
    bool bCalib = false;
    int num_get = 0;
    
    while(waitKey(1)==-1)
    {	
        capture >> image;

        if (image.empty())
            break;
        imshow("original_image", image);

        cvtColor(image, grayimage, CV_BGR2GRAY);
        IplImage tempgray = grayimage;
        bool findchessboard = cvCheckChessboard(&tempgray, ChessBoardSize);

        if (findchessboard)
        {
            bool find_corners_result = findChessboardCorners(grayimage, ChessBoardSize, tempcorners, 3);
            if (find_corners_result)
            {
                cornerSubPix(grayimage, tempcorners, cvSize(5, 5), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                drawChessboardCorners(image, ChessBoardSize, tempcorners, find_corners_result);
                imshow("corner_image", image);
                				
				objectv.push_back(object);
		        imagev.push_back(tempcorners);
		        
		        cout << "number " << num_get << " pictures" << endl;
		        num_get++;
            }
        }
        tempcorners.clear();
        //cout <<  num << " pictures" << endl;
    }
	if(num_get > 10){
		destroyAllWindows();
		cout << "-----begin to calibrate-----" << endl;
		cv::fisheye::calibrate(objectv, imagev, cv::Size(image.cols,image.rows), intrinsics, distortion_coeff, cv::noArray(), cv::noArray(), flag, cv::TermCriteria(3, 20, 1e-6));  
		fisheye::initUndistortRectifyMap(intrinsics, distortion_coeff, cv::Matx33d::eye(), intrinsics, corrected_size, CV_16SC2, mapx, mapy);
		cout << "-----calibrate successfully----" << endl;
		
		for(int i=0; i<3; ++i)
		{
		    for(int j=0; j<3; ++j)
		    {
		        intrinsicfile<<intrinsics(i,j)<<"\t";
		    }
		    intrinsicfile<<endl;
		}
		for(int i=0; i<4; ++i)
		{
		    disfile<<distortion_coeff(i)<<"\t";
		}
		intrinsicfile.close();
		disfile.close();
		
		while (waitKey(1)==-1)
		{
		    capture.read(image);

		    if (image.empty())
		        break;
		    remap(image, corrected, mapx, mapy, INTER_LINEAR, BORDER_TRANSPARENT);

		    imshow("original_image", image);
		    imshow("corrected", corrected);		     
		}
	}
	else{
		cout << "------calibrate failure-----" << endl;
	}
	
    return 0;
}
