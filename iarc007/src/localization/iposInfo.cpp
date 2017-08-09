#include <localization/iposInfo.h>
#include <localization/localization.h>

void iposInfo::callback(const dji_sdk::AccelerationConstPtr& f_acc,const dji_sdk::AttitudeQuaternionConstPtr& f_att,const \
sensor_msgs::LaserScanConstPtr& g_ul,const geometry_msgs::Vector3StampedConstPtr& g_pos,const geometry_msgs::Vector3StampedConstPtr& g_vo)
{
 	Eigen::Quaterniond q_(f_att->q0,f_att->q1,f_att->q2,f_att->q3);
	mmWPosG = q_.toRotationMatrix();

	mvWSpeedG << f_att->wx,f_att->wy,f_att->wz;
	mvWSpeedG = mmWPos.transpose()*mmWPosG*mvWSpeedG;

	mvTAccG << f_acc->ax,f_acc->ay,f_acc->az;
	mvTAccG = mmWPos.transpose()*mmWPosG*mvTAccG;

	mvTSpeedG << g_vo->vector.x,g_vo->vector.y,g_vo->vector.z;
	mvTSpeedG = mmWPosG.transpose()*mvTSpeedG;

	mvTPosG << g_pos->vector.x,g_pos->vector.y,g_pos->vector.z;
	
	if(g_ul->ranges[0] < 0.){
		
	}
	else{
		mvTHeightG = 0.4*mvTHeightG + 0.6*g_ul->ranges[0];	
	}
	//mvTPos[2] = -mvTHeightG;
}
void iposInfo::image_callback(const sensor_msgs::ImageConstPtr& left_img)
{
	double t = (double)cvGetTickCount();

	cv_bridge::CvImagePtr cv_ptr;

	try {
		cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	cv::cvtColor(cv_ptr->image,gray,CV_BGR2GRAY);

	if(!gray.empty())  
	{  
		updateFlag = true;
//!=====================================================================
/*! step1: Detection features
* lines-points-quads
*/
		points.clear();
		pointsFilter.clear();
		points = localization::extractTags(gray,50,50);
		float maxCombineDistance = 5.0;			

		featursSorting(maxCombineDistance);
		featureMerging(maxCombineDistance);

/*		cout << "feature1: " << endl;
		for(int i=0;i<pointsFilter.size();i++){
			cout << "pixels: (" << pointsFilter[i].first << "," << pointsFilter[i].second << ")" << endl;
		}
*/
		vector<vector<cv::Point2f>> quadsPoints;
		vector<vector<std::pair<float,float>>> pQuadsPoints;

		pQuadsPoints=featureQuad(5.0);

		//cout << "extractTageS3" << endl;
		for(int i=0;i<pQuadsPoints.size();i++){
			vector<Point2f> vPts(4);
			for(int j=0;j<4;j++){
				vPts[j] = Point2f(pQuadsPoints[i][j].first,pQuadsPoints[i][j].second);
			}
			quadsPoints.push_back(vPts);
		}
		ROS_INFO("!!! Detetion END !!!");
//! ==============================================================================================
/*! step2: matching step
*  projection and quads matching
*/
		Matrix3d mmHomography;

		mmWPosInv = mmWPos.transpose();
		mmHomography.topLeftCorner(3,2) = mmWPosInv.topLeftCorner(3,2);
		mmHomography.bottomRightCorner(3,1) = -mmWPosInv*mvTPos;

		cout << "matrix: " << endl << mmWPos << endl;
		cout << "translation: " << endl << mvTPos << endl;

		mmHomography = mmCamera*mmHomography;

		mapProjection(mmHomography);

		//cout << "segment1" << endl;
		cout << "mapPoints: " << mapPoints.size() << endl;
		for(int i=0;i<mapPoints.size();i++){
			cout << "pixels: (" << mapPoints[i].first << "," << mapPoints[i].second << ")" << endl;
		}

		mapSorting(maxCombineDistance);
		mapMerging(maxCombineDistance);

		vector<vector<cv::Point2f>> quadsMapP;
		vector<vector<cv::Point3f>> quadsMapW;
		//vector<vector<std::pair<float,float>>> pQuadsPoints;
		std::pair<float,float> pt;

		pQuadsPoints=mapQuad(5.0);
		
		//cout << "segments3" << endl;

		for(int i=0;i<pQuadsPoints.size();i++){
			vector<Point2f> vPts(4);
			vector<Point3f> v3Pts(4);
			for(int j=0;j<4;j++){
				vPts[j] = Point2f(pQuadsPoints[i][j].first,pQuadsPoints[i][j].second);
				pt = mapReprojection(mmHomography,pQuadsPoints[i][j]);
				v3Pts[j] = Point3f(pt.first,pt.second,1.0);
			}
			quadsMapP.push_back(vPts);
			quadsMapW.push_back(v3Pts);
		}

		//cout << "segments2" << endl;

		std::vector<cv::Point3f> objPts;
		std::vector<cv::Point2f> imgPts;

		vector<float> vDistance;

		for(int i=0;i<quadsPoints.size();i++){
			for(int j=0;j<quadsMapP.size();j++){
				float m = 0.;
				for(int k = 0;k<4;k++){
					m += fabs(quadsPoints[i][k].x-quadsMapP[j][k].x) + \
					fabs(quadsPoints[i][k].y-quadsMapP[j][k].y);
				}
				vDistance.push_back(m);							
			}
			if(vDistance.size() > 0){
				auto smallest = std::min_element(std::begin(vDistance), std::end(vDistance));
				int pos = std::distance(std::begin(vDistance), smallest);
				cout << "smallest" << *smallest << endl;
				//if(*smallest < 100){
				{
				  for(int n=0;n<4;n++){
					imgPts.push_back(quadsPoints[i][n]);
				
					objPts.push_back(quadsMapW[pos][n]);				
				  }
				}
				vDistance.clear();
				//cout << "segment++" << endl;
			}
		}

/*		cout << "mapPointsInPixels: " << mapPointsInPixels.size() << endl;
		for(int i=0;i<mapPointsInPixels.size();i++){
			cout << "pixels: (" << mapPointsInPixels[i].first << "," << mapPointsInPixels[i].second << ")" << endl;
		}
*/
/*		cout << "mapPoints: " << mapPoints.size() << endl;
		for(int i=0;i<mapPoints.size();i++){
			cout << "pixels: (" << mapPoints[i].first << "," << mapPoints[i].second << ")" << endl;
		}
*/		cout << "feature: " << endl;
		for(int i=0;i<pointsFilter.size();i++){
			cout << "pixels: (" << pointsFilter[i].first << "," << pointsFilter[i].second << ")" << endl;
		}
		cout << "quads(detection): "<< endl;
		for(int i=0;i<quadsPoints.size();i++){
			for(int j=0;j<4;j++){
				cv::circle(gray,quadsPoints[i][j],5.,cv::Scalar(255));
				cout << quadsPoints[i][j] << ",";
			}
			cout << endl;
		}
		cout << "quads(map): "<< endl;
		for(int i=0;i<quadsMapP.size();i++){
			for(int j=0;j<4;j++){
				cv::circle(gray,quadsMapP[i][j],5.,cv::Scalar(0));
				cout << quadsMapP[i][j] << ",";
			}
			cout << endl;
		}

		cout << "matching !!!END!!!"<< endl;
//!=============================================================================================
/*! Step3: Pose Estimation
*  pnp method
*  homography estimate
*/
		if(imgPts.size()>=4){
			PoseEstimation(objPts,imgPts,2);
		}

		cv::imshow("gray", gray);
		cv::waitKey(1);	
	}

	double time = ((double)cvGetTickCount() - t)/((double)cvGetTickFrequency()*1000.);

	cout << "running time: " << time << std::endl;
}
//! to load map from file s
void iposInfo::mapLoading(string s)
{
	ifstream infile;

	infile.open(s.c_str());
	assert(infile.is_open());

	char buffer[100];
	
	bool odd = false;

	std::pair<float,float> point;

	while(!infile.eof())
	{
		infile >> buffer;
		
		if(!odd){
			point.first = atof(buffer);
			odd = true;
		}
		else{
			point.second = atof(buffer);
			odd = false;
			map.push_back(point);
		}
	}

	infile.close();
}
//! project map into the image
void iposInfo::mapProjection(Matrix3d mmHomography)
{
	//cout << "size " << map.size() << endl;
	//cout << "mapPoints: " << endl;
	mapPoints.clear();
	for(std::vector<std::pair<float,float>>::iterator it=map.begin();it!=map.end();it++){
		Vector3d worldCoord(it->first,it->second,1.);
		Vector3d pixelCoord = mmHomography*worldCoord;
		pixelCoord = pixelCoord/pixelCoord(2);
	
		if(pixelCoord(0) > 0. && pixelCoord(0) < 640. && pixelCoord(1) > 0. && pixelCoord(1) < 480.){

			std::pair<float,float> point;
			point.first = pixelCoord(0);
			point.second = pixelCoord(1);

			mapPoints.push_back(point);

			//cout << "pixels: (" << pixelCoord(0) << "," << pixelCoord(1) << ")" << endl;
			//cv::circle(gray,cv::Point2f(pixelCoord(0),pixelCoord(1)),5.,cv::Scalar(0));
		}
	}
	//cout << "!!!END!!!" << endl;
}
//! reprojection map features to world plane
std::pair<float,float> iposInfo::mapReprojection(Matrix3d mmHomography,std::pair<float,float>& Ptsrc)
{	
	Vector3d pixelCoord(Ptsrc.first,Ptsrc.second,1.);
	Vector3d worldCoord = mmHomography.inverse()*pixelCoord;
	worldCoord = worldCoord/worldCoord(2);

	std::pair<float,float> Ptdst;
	Ptdst.first = worldCoord(0);
	Ptdst.second = worldCoord(1);
	return Ptdst;
}
//! to sort the map according to the pos of map feature
void iposInfo::mapSorting(float maxCombineDistance)
{
	for(int i =0;i<mapPoints.size();i++){
	  for(int j=i+1;j<mapPoints.size();j++){
		if(mapPoints[i].first > mapPoints[j].first){
			float temp;
			temp = mapPoints[i].first;
			mapPoints[i].first = mapPoints[j].first;
			mapPoints[j].first = temp;
			temp = mapPoints[i].second;
			mapPoints[i].second = mapPoints[j].second;
			mapPoints[j].second = temp;
		}
	  }
	}
	for(int i =0;i<mapPoints.size();i++){
	  int j =i+1;
	  for(j=i+1;j<mapPoints.size();j++){	
		if(fabs(mapPoints[i].first-mapPoints[j].first) > maxCombineDistance){
			break;
		}
	  }
	  for(int n=i;n<j;n++){
	    for(int m=n+1;m<j;m++){
		if(mapPoints[n].second > mapPoints[m].second){
			float temp;
			temp = mapPoints[n].first;
			mapPoints[n].first = mapPoints[m].first;
			mapPoints[m].first = temp;
			temp = mapPoints[n].second;
			mapPoints[n].second = mapPoints[m].second;
			mapPoints[m].second = temp;
		}
	    }
	  }
	  i = j-1;
	}
}
//! merge neibor map features
void iposInfo::mapMerging(float maxCombineDistance)
{
	mapPointsInPixels.clear();
	for(int i=0;i<mapPoints.size();i++){
	  int j=i+1;
	  for(j=i+1;j<mapPoints.size();j++){
		if(fabs(mapPoints[i].first-mapPoints[j].first)<maxCombineDistance&&\
		fabs(mapPoints[i].second-mapPoints[j].second)<maxCombineDistance){		
		}
		else{
		  break;
		}
	  }
	  if(j-i>=1){
	    float xSum=0.,ySum=0.;
	    for(int n=i;n<j;n++){
		xSum+=mapPoints[n].first;
		ySum+=mapPoints[n].second;
	    }
	    std::pair<float,float> pt;
	    pt.first=xSum/(j-i);
	    pt.second=ySum/(j-i);
	    mapPointsInPixels.push_back(pt);
	  }
	  i = j-1;
	}
}
//! connect map features to quads 
/*!      {  0 2  }
*        {  1 3  } <quads>
*/
vector<vector<std::pair<float,float>>> iposInfo::mapQuad(float maxCombineDistance)
{
	//vector<vector<float>> vvDist;
	vector<float> vdx;
	vector<float> vdy;
	vector<float> vdist;
	vector<float> vPara;
	vector<float> vParaOth;	
	vector<vector<std::pair<float,float>>> vhQuads;
	vector<vector<std::pair<float,float>>> vQuads;
	vector<std::pair<float,float>> hQuads(3);
	vector<std::pair<float,float>> Quads(4);

	for(int i=0;i<mapPointsInPixels.size();i++){
	  for(int j=i+1;j<mapPointsInPixels.size();j++){
		float dx = mapPointsInPixels[i].first-mapPointsInPixels[j].first;
		float dy = mapPointsInPixels[i].second-mapPointsInPixels[j].second;
		float dist = sqrt(dx*dx+dy*dy);
		vdx.push_back(dx/dist);
		vdy.push_back(dy/dist);
		vdist.push_back(dist);
	  }
	
	  for(int n=0;n<vdist.size();n++){
	    for(int m=n+1;m<vdist.size();m++){
		float dist = fabs(vdist[m]-vdist[n]);
		float angle = fabs(vdx[m]*vdx[n]+vdy[m]*vdy[n]);
		vPara.push_back(dist);
		vParaOth.push_back(angle);
	    }
	    if(vPara.size()>0){
		auto smallest = std::min_element(std::begin(vPara),std::end(vPara));
		//cout << "small" << *smallest << endl;
		int pos = std::distance(std::begin(vPara), smallest);
		if(*smallest < 10.0 && vParaOth[pos] < 0.1){
			
			hQuads[0] = mapPointsInPixels[i];
			hQuads[1] = mapPointsInPixels[i+n+1];
			hQuads[2] = mapPointsInPixels[i+n+1+pos+1];
			vhQuads.push_back(hQuads);
		}
		vPara.clear();
		vParaOth.clear();
	    }
	  }
	  vdist.clear();
	  vdx.clear();
	  vdy.clear();
	}
	for(int i=0;i<vhQuads.size();i++){
	  float Ex = vhQuads[i][1].first + vhQuads[i][2].first - vhQuads[i][0].first;
	  float Ey = vhQuads[i][1].second + vhQuads[i][2].second - vhQuads[i][0].second;
	  for(int j=0;j<mapPointsInPixels.size();j++){
		if(fabs(mapPointsInPixels[j].first-Ex)<maxCombineDistance &&\
		fabs(mapPointsInPixels[j].second-Ey)<maxCombineDistance){
			Quads[0] = vhQuads[i][0];
			Quads[1] = vhQuads[i][1];
			Quads[2] = vhQuads[i][2];
			Quads[3] = mapPointsInPixels[j];
			vQuads.push_back(Quads);
			break;
		}
	  }
	}
	return vQuads;
}
//! merge neibor features
void iposInfo::featureMerging(float maxCombineDistance)
{
	pointsFilter.clear();
	for(int i=0;i<points.size();i++){
	  int j=i+1;
	  for(j=i+1;j<points.size();j++){
		if(fabs(points[i].first-points[j].first)<maxCombineDistance&&\
		fabs(points[i].second-points[j].second)<maxCombineDistance){		
		}
		else{
		  break;
		}
	  }
	  if(j-i>=1){
	    float xSum=0,ySum=0.;
	    for(int n=i;n<j;n++){
		xSum+=points[n].first;
		ySum+=points[n].second;
	    }
	    std::pair<float,float> pt;
	    pt.first=xSum/(j-i);
	    pt.second=ySum/(j-i);
	    pointsFilter.push_back(pt);
	  }
	  i = j-1;
	}
}
//! connect features to quads
vector<vector<std::pair<float,float>>> iposInfo::featureQuad(float maxCombineDistance)
{
	//vector<vector<float>> vvDist;
	vector<float> vdx;
	vector<float> vdy;
	vector<float> vdist;
	vector<float> vPara;
	vector<float> vParaOth;
		
	vector<vector<std::pair<float,float>>> vhQuads;
	vector<vector<std::pair<float,float>>> vQuads;
	vector<std::pair<float,float>> hQuads(3);
	vector<std::pair<float,float>> Quads(4);

	for(int i=0;i<pointsFilter.size();i++){
	  for(int j=i+1;j<pointsFilter.size();j++){
		float dx = pointsFilter[i].first-pointsFilter[j].first;
		float dy = pointsFilter[i].second-pointsFilter[j].second;
		float dist = sqrt(dx*dx+dy*dy);
		vdx.push_back(dx/dist);
		vdy.push_back(dy/dist);
		vdist.push_back(dist);
	  }
	  for(int n=0;n<vdist.size();n++){
	    for(int m=n+1;m<vdist.size();m++){
		float dist = fabs(vdist[m]-vdist[n]);
		float angle = fabs(vdx[m]*vdx[n]+vdy[m]*vdy[n]);
		vPara.push_back(dist);
		vParaOth.push_back(angle);
	    }
	    if(vPara.size()>0){
		auto smallest = std::min_element(std::begin(vPara),std::end(vPara));
		int pos = std::distance(std::begin(vPara), smallest);
		cout << "angle: " << vParaOth[pos] << endl;
		cout << "small: " << *smallest << endl;
		if(*smallest < 20.0 && vParaOth[pos] < 0.15){
			
			hQuads[0] = pointsFilter[i];
			hQuads[1] = pointsFilter[i+n+1];
			hQuads[2] = pointsFilter[i+n+1+pos+1];
			vhQuads.push_back(hQuads);
		}
	    }
	    vPara.clear();
	    vParaOth.clear();
	  }
	  vdist.clear();
	  vdx.clear();
	  vdy.clear();
	}

	for(int i=0;i<vhQuads.size();i++){
	  float Ex = vhQuads[i][1].first + vhQuads[i][2].first - vhQuads[i][0].first;
	  float Ey = vhQuads[i][1].second + vhQuads[i][2].second - vhQuads[i][0].second;
	  for(int j=0;j<pointsFilter.size();j++){
		if(fabs(pointsFilter[j].first-Ex)<maxCombineDistance &&\
		fabs(pointsFilter[j].second-Ey)<maxCombineDistance){
			Quads[0] = vhQuads[i][0];
			Quads[1] = vhQuads[i][1];
			Quads[2] = vhQuads[i][2];
			Quads[3] = pointsFilter[j];
			vQuads.push_back(Quads);
			break;
		}
	  }
	}
	return vQuads;
}

//! to sort the features according to the pos
void iposInfo::featursSorting(float maxCombineDistance)
{
	for(int i =0;i<points.size();i++){
	  for(int j=i+1;j<points.size();j++){
		if(points[i].first > points[j].first){
			float temp;
			temp = points[i].first;
			points[i].first = points[j].first;
			points[j].first = temp;
			temp = points[i].second;
			points[i].second = points[j].second;
			points[j].second = temp;
		}
	  }
	}
	for(int i =0;i<points.size();i++){
	  int j=i+1;
	  for(j=i+1;j<points.size();j++){	
		if(fabs(points[i].first-points[j].first) >= maxCombineDistance){
			break;
		}
	  }
	  for(int n=i;n<j;n++){
	    for(int m=n+1;m<j;m++){
		if(points[n].second > points[m].second){
			float temp;
			temp = points[n].first;
			points[n].first = points[m].first;
			points[m].first = temp;
			temp = points[n].second;
			points[n].second = points[m].second;
			points[m].second = temp;
		}
	    }
	  }
	  i = j-1;
	}
}
void iposInfo::PoseEstimation(std::vector<cv::Point3f> objPts,std::vector<cv::Point2f> imgPts,int method)
{
	cv::Mat rvec, tvec;

	float fx = mmCamera(0,0);
	float px = mmCamera(0,2);
	float fy = mmCamera(1,1);
	float py = mmCamera(1,2);

	cv::Matx33f cameraMatrix(fx, 0, px,
				 0, fy, py,
				 0,  0,  1);
	if(method==1){	
		cv::Vec4f distParam(0,0,0,0); // all 0?

		cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);

		cv::Matx33d r;
		cv::Rodrigues(rvec, r);

		mmWPosInv << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

		mmWPos = mmWPosInv.transpose();

		mvTPos << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
		mvTPos = -mmWPos*mvTPos;

	}
	else if(method == 2){
		cv::Point2f tempPoint;
		vector<cv::Point2f> objPoints;

		for(int i=0;i<objPts.size();i++){
			tempPoint.x = objPts[i].x;
			tempPoint.y = objPts[i].y;
			objPoints.push_back(tempPoint);
		}

		cv::Matx33f H;  

		H = findHomography(objPoints,imgPts);
		H = cameraMatrix.inv()*H;

		Vector3d r1(H(0,0),H(1,0),H(2,0));
		Vector3d r2(H(0,1),H(1,1),H(2,1));

		double m1 = sqrt(r1.dot(r1));
		double m2 = sqrt(r2.dot(r2));

		r1 /= m1;
		r2 /= m2;

		Vector3d r3 = r1.cross(r2);

		mmWPosInv << r1(0,0), r2(0,0), r3(0,0), r1(1,0), r2(1,0), r3(1,0), r1(2,0), r2(2,0), r3(2,0);

		mmWPos = mmWPosInv.transpose();
	
		mvTPos << 2*H(0,2)/(m1+m2), 2*H(1,2)/(m1+m2), 2*H(2,2)/(m1+m2);

		mvTPos = -mmWPos*mvTPos;
	}
}
