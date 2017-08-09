// Farneback dense optical flow calculate and show in Munsell system of colors  
// Author : Zouxy  
// Date   : 2013-3-15  
// HomePage : http://blog.csdn.net/zouxy09  
// Email  : zouxy09@qq.com  

// API calcOpticalFlowFarneback() comes from OpenCV, and this  
// 2D dense optical flow algorithm from the following paper:  
// Gunnar Farneback. "Two-Frame Motion Estimation Based on Polynomial Expansion".  
// And the OpenCV source code locate in ..\opencv2.4.3\modules\video\src\optflowgf.cpp  
// Color encoding of flow vectors from:  
// http://members.shaw.ca/quadibloc/other/colint.htm  
// This code is modified from:  
// http://vision.middlebury.edu/flow/data/  

#include <iarc007/hy_target/opFlow.h>
#include <Eigen/Geometry>
#include <ros/ros.h>

/* setup color panel of Munsell color system */
void opFlow::makecolorwheel(vector<Scalar> &colorwheel)
{  
	int RY = 15;
	int YG = 6;
	int GC = 4;
	int CB = 11;
	int BM = 13;  
	int MR = 6;
	int i;

	for (i = 0; i < RY; i++) 
		colorwheel.push_back(Scalar(255,255*i/RY,0));  
	for (i = 0; i < YG; i++) 
		colorwheel.push_back(Scalar(255-255*i/YG,255,0));  
	for (i = 0; i < GC; i++) 
		colorwheel.push_back(Scalar(0,255,255*i/GC));  
	for (i = 0; i < CB; i++) 
		colorwheel.push_back(Scalar(0,255-255*i/CB,255));  
	for (i = 0; i < BM; i++) 
		colorwheel.push_back(Scalar(255*i/BM,0,255));  
	for (i = 0; i < MR; i++) 
		colorwheel.push_back(Scalar(255,0,255-255*i/MR));  
}  
/* transfer optical flow to  color image of Munsell color system*/
void opFlow::motionToColor(Mat flow,Mat& color)
{
	if (color.empty())  
	    color.create(flow.rows,flow.cols,CV_8UC3);  
	
	if (colorwheel.empty())  
	    makecolorwheel(colorwheel);  

	// determine motion range:  
	float maxrad = -1;

	// Find max flow to normalize fx and fy  s
	for (int i= 0;i < flow.rows;++i)
	{
		for (int j = 0;j < flow.cols;++j)   
		{  
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);
			float fx = flow_at_point[0];
			float fy = flow_at_point[1];  
			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
			    continue;  
			float rad = sqrt(fx * fx + fy * fy);  
			maxrad = maxrad > rad ? maxrad : rad;
		}  
	}  

	for (int i= 0; i < flow.rows; ++i)   
	{  
		for (int j = 0; j < flow.cols; ++j)   
		{  
			uchar *data = color.data + color.step[0] * i + color.step[1] * j;  

			Vec2f flow_at_point = flow.at<Vec2f>(i, j);  

			float fx = flow_at_point[0] / maxrad;  
			float fy = flow_at_point[1] / maxrad;  

			if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))  
			{  
				data[0] = data[1] = data[2] = 0;  
				continue;  
			}  

			float rad = sqrt(fx * fx + fy * fy);

			float angle = atan2(-fy, -fx) / CV_PI;

			float fk = (angle + 1.0) / 2.0 * (colorwheel.size()-1);
			int k0 = (int)fk;
			int k1 = (k0 + 1) % colorwheel.size();
			float f = fk - k0;

			for (int b = 0; b < 3; b++)
			{
				float col0 = colorwheel[k0][b] / 255.0;  
				float col1 = colorwheel[k1][b] / 255.0;  
				float col = (1 - f) * col0 + f * col1;
				if(rad <= 1)
					col = 1 - rad * (1 - col); // increase saturation with radius  
				else
				col *= .75;//out of range
				data[2 - b] = (int)(255.0 * col);
			}
		}
	}
}
/* subscribe msg from sensors or bags */
void opFlow::callback(const sensor_msgs::ImageConstPtr& left_img,const dji_sdk::AttitudeQuaternionConstPtr& f_att,const sensor_msgs::LaserScanConstPtr& g_ul,const geometry_msgs::Vector3StampedConstPtr& g_vo)
{
	//mutexOp.lock();	

	double t = (double)cvGetTickCount();  		

	cv_bridge::CvImagePtr cv_ptr;

	try {
		cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	cv_ptr->image.copyTo(gray);
	
	if(!gray.empty()){
		cout << "(cols,rows): " << gray.cols << " " << gray.rows << endl;
		ROS_INFO("Subscribe image successfully!");
	}
	//if(img_sub.channels() == 3){
		//cvtColor(img_sub,gray,COLOR_BGR2GRAY);
	//}
	//else{
		//img_sub.copyTo(gray);
	//}
	
	//TODO:	
	w_3d.at<float>(0,0) = f_att->wy;
	w_3d.at<float>(1,0) = -f_att->wx;
	w_3d.at<float>(2,0) = f_att->wz;	

 	Eigen::Quaterniond q_(f_att->q0,f_att->q1,f_att->q2,f_att->q3);
	Eigen::Matrix3d mRot = q_.toRotationMatrix();

	Eigen::Vector3d vo_guid(g_vo->vector.x,g_vo->vector.y,g_vo->vector.z);

	vo_guid = mRot*vo_guid;

	vo_3d.at<float>(0,0) = vo_guid(1);//g_vo->vector.x;
	vo_3d.at<float>(1,0) = -vo_guid(0);//g_vo->vector.y;
	vo_3d.at<float>(2,0) = vo_guid(2);//g_vo->vector.z;

	//ROS_INFO("optical flow field calculated!");

	if(fabs(g_ul->ranges[0] - d) < 0.3){
		d = g_ul->ranges[0];
	}
	else{
		//
	}
	
	//ROS_INFO("optical flow field calculated!");

	//S_INFO("optical flow field calculated!");
	

	if(!prevgray.empty())  
	{  
		if(!prevgray.empty()){
			ROS_INFO("optical flow field calculated!0-1");
			cout << "(cols,rows): " << prevgray.cols << " " << prevgray.rows << endl;
		}
		if(!flow.empty()){
			cout << "(cols,rows): " << flow.cols << " " << flow.rows << endl;
			ROS_INFO("optical flow field calculated!0-2");
		}
		else{
		
		}
		ROS_INFO("optical flow field calculated!0-3");
		
		//ROS_INFO("optical flow field calculated! 00");
		imshow("gray",gray);
		//imshow("flow",flow);
		imshow("prevgray",prevgray);
		waitKey(1);
		
		calcOpticalFlowFarneback(prevgray,gray,flow, 0.5, 3, 15, 3, 5, 1.2, 0);

		if(!flow.empty()){
			cout << "(cols,rows): " << flow.cols << " " << flow.rows << endl;
			ROS_INFO("optical flow field calculated!0-2");
		}
		
		//motionToColor(flow,color);
		updateFlag = true;
		
		ROS_INFO("optical flow field calculated!");
		
		std::swap(prevgray,gray);
		//gray.copyTo(prevgray);
	}
	else{
		ROS_INFO("optical flow field calculated!----");
		//prevgray.create(cv_ptr->image.rows,cv_ptr->image.cols,CV_8UC(1));  	
		
		gray.copyTo(prevgray);
		updateFlag = false;
	}  

	prevTimeStamp = nextTimeStamp;
	nextTimeStamp = left_img->header.stamp.sec+left_img->header.stamp.nsec*1e-9;


	time = ((double)cvGetTickCount() - t)/((double)cvGetTickFrequency()*1000.);
	
	//mutexOp.unlock();

	//ROS_INFO("optical flow field calculated!");
	//cout << "period " << time << endl;
}
/* recover motion from optical flow */
void opFlow::voFromOpFlow()
{
	float sum_u = 0., sum_v = 0.;

	Mat ofWx = Mat(1,3,CV_32FC1,Scalar::all(0));
	Mat ofWy = Mat(1,3,CV_32FC1,Scalar::all(0));

	if (mFlowT.empty())
	    mFlowT.create(flow.rows,flow.cols,CV_32FC2);  
	
	//cout << "image time interval:  " << (nextTimeStamp-prevTimeStamp) << endl;

	double t = (nextTimeStamp-prevTimeStamp);
	
	for (int i= 0;i < mFlowT.rows;++i)   
	{  
		for (int j = 0;j < mFlowT.cols;++j)   
		{  
			Vec2f flow_at_point = flow.at<Vec2f>(i, j);  

			float fx = flow_at_point[0]/t;  
			float fy = flow_at_point[1]/t;

			float u = i - cu;
			float v = j - cv;

			ofWx.at<float>(0,0) = -u*v/fu;
			ofWx.at<float>(0,1) = (fu*fu+u*u)/fu;			
			ofWx.at<float>(0,2) = -v;

			ofWy.at<float>(0,0) = -(fv*fv+v*v)/fv;
			ofWy.at<float>(0,1) = u*v/fv;
			ofWy.at<float>(0,2) = u;
			
			Mat ofx = -ofWx*w_3d;
			Mat ofy = -ofWy*w_3d;

			mFlowT.at<Vec2f>(i,j)[0] = fx;//ofx.at<float>(0,0);
			mFlowT.at<Vec3f>(i,j)[1] = fy;//ofy.at<float>(0,0);
		}
	}
}
void opFlow::voEstimate()
{
	float sum_u = 0.,sum_v = 0.;

	for(int i = 0;i < mFlowT.rows;++i){
		for(int j = 0;j < mFlowT.cols;++j)
		{
			sum_u += mFlowT.at<Vec2f>(i,j)[0];
			sum_v += mFlowT.at<Vec2f>(i,j)[1];
		}
	}
	vo[0] = -(sum_u/(float)(mFlowT.rows*mFlowT.cols))/fu*d;
	vo[1] = -(sum_v/(float)(mFlowT.rows*mFlowT.cols))/fv*d;	
	
	cout << "fu: " << fu << " "\
	     << "fv: " << fv << " "\
	     << "d:  " << d  << " "\
	     << "num: "<< (mFlowT.rows*mFlowT.cols) << " "\
	     << "sum: "<< sum_u << " "\
	     << "sum: "<< sum_v << " " << endl;
}
/*  */
void opFlow::motionEstimate()
{
	//ros::Rate rate(10);

	//while(ros::ok()){
		
		//mutexOp.lock();		
		
		if(updateFlag){
			voFromOpFlow();//消除旋转运动
			//motionToColor(mFlowT,colorT);
			voEstimate();
			updateFlag = false;
			/*cout << "time cost(ms): " << vFlow.time << endl;
			cout << "vo: " << vFlow.vo[0] << " " << vFlow.vo[1] << endl;
			cout << "guidance: " << vFlow.vo_3d << endl;
			cout << "height: " << vFlow.d << endl;
			*/
			outfile << time << " "\
			<< vo[0] << " " << vo[1] << " "\
			<< vo_3d.at<float>(0,0) << " " << vo_3d.at<float>(1,0) << " "\
			<< d << endl;	
			ROS_INFO("motion estimate complete!");	
		}

		//mutexOp.unlock();

/*		if(!colorT.empty()){
			imshow("ofT",colorT);
		}
		if(!gray.empty()){
			imshow("gray",gray);
		}
		if(!color.empty()){
			imshow("of",color);
		}
		cv::waitKey(1);	
*/
	

		//rate.sleep();
	//}

	//ROS_INFO("thread over!");
}
