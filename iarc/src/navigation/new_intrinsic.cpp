#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv/cv.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    CvMat* Intrinsic0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/Intrinsics.xml");
    Mat Intrinsic = Mat(Intrinsic0, true);
    CvMat* Distortion0 = (CvMat*)cvLoad("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/Distortion.xml");
    Mat Distortion = Mat(Distortion0, true);
    
    Mat NewCameraMatrix =Mat(3, 3, CV_32FC1, Scalar(0));
    Size size=Size(640,480);
    NewCameraMatrix = getOptimalNewCameraMatrix(Intrinsic, Distortion, size, 0);
    cout << NewCameraMatrix << endl;
    CvMat* NewCameraMatrix1 = cvCreateMat(3, 3, CV_32FC1);
    CvMat temp = NewCameraMatrix;
    cvCopy(&temp, NewCameraMatrix1);
    cvSave("/home/hitcsc/catkin_ws/src/iarc/doc/navigation/NewIntrinsics.xml", NewCameraMatrix1);
    cvReleaseMat(&NewCameraMatrix1);
    return 0;
}
