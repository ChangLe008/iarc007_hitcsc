#include "iarc007/mjh_pos/find_line.h"
#include "iarc007/mjh_pos/threshold_image.h"

bool FindLine(Mat ImageSource,  vector<Vec4i> &_OutFitLine, int Flag, int PixelThreshold, int ThinTime = 0)
{
	Mat ImageRed;
	Mat ImageWhite;
	Mat ImageGreen;
	
	ImageSource.copyTo(ImageRed);
	ImageSource.copyTo(ImageWhite);
	ImageSource.copyTo(ImageGreen);
	
	Mat ImageRed1;
	Mat ImageWhite1;
	Mat ImageGreen1;
	
	ImageSource.copyTo(ImageRed1);
	ImageSource.copyTo(ImageWhite1);
	ImageSource.copyTo(ImageGreen1);
	
	Mat ImageBinary = Mat::zeros(ImageRows, ImageCols, CV_8UC1);
	
	if(Flag == FindWhiteLine)
	{
		ThresholdWhiteLine(ImageSource, ImageBinary, ThinTime);
	}
	else if(Flag == FindRedLine)
	{
		ThresholdRedLine(ImageSource, ImageBinary);
	}
	else if(Flag == FindGreenLine)
	{
		ThresholdGreenLine(ImageSource, ImageBinary);
	}
	vector<Vec4i> Lines;
	HoughLinesP( ImageBinary, Lines, 1, CV_PI/180, 80, 30, 50);
	if(Lines.size() >= 1)
	{
		vector<Vec4i> OutFitLine;
		int ClassOfLine[Lines.size()];
		bool LineFlag[Lines.size()];
		
		for(int i = 0; i < Lines.size(); i++)
		{
			LineFlag[i] = false;
		}
		
		float AngleInPicThreshold = 15; 
		int LineClassNumber = -1;	
		AngleInPicThreshold = AngleInPicThreshold /180.0 * CV_PI;
		
		for(int i = 0; i < Lines.size(); i++) //直线分类
		{
			if(LineFlag[i])
			{
				continue;
			}
			LineFlag[i] = true;
			LineClassNumber ++;
			ClassOfLine[i] = LineClassNumber;
			float x0_i = Lines[i][0];
			float y0_i = Lines[i][1];
			float x1_i = Lines[i][2];
			float y1_i = Lines[i][3];			
			
			float A = -(y1_i - y0_i);
			float B = (x1_i - x0_i);
			float C = (y1_i - y0_i) * x0_i - (x1_i - x0_i) * y0_i;
			float Den = sqrt(A * A + B * B);
			float Angle_i = atan2(y1_i - y0_i, x1_i - x0_i);
			
			if(Angle_i > CV_PI / 2)
			{
				Angle_i -= CV_PI;
			}
			if(Angle_i <= - CV_PI / 2)
			{
				Angle_i += CV_PI;
			}
			
			for(int j = i + 1; j < Lines.size(); j++)
			{
				float x0_j = Lines[j][0];
				float y0_j = Lines[j][1];
				float x1_j = Lines[j][2];
				float y1_j = Lines[j][3];	
				
				float Angle_j = atan2(y1_j - y0_j, x1_j - x0_j);
				if(Angle_j > CV_PI / 2)
				{
					Angle_j -= CV_PI;
				}
				if(Angle_j <= - CV_PI / 2)
				{
					Angle_j += CV_PI;
				}
				bool AngleFlag = fabs(Angle_j - Angle_i) < AngleInPicThreshold;
				AngleFlag = AngleFlag  | (CV_PI - fabs(Angle_j - Angle_i)) < AngleInPicThreshold;
				if(AngleFlag)
				{
					float MiddleX = (x0_j + x1_j)/2;
					float MiddleY = (y0_j + y1_j)/2;
					float Length = fabs(A * MiddleX + B * MiddleY + C) / Den;
					
					if(Length < PixelThreshold)
					{
						LineFlag[j] = true;
						ClassOfLine[j] = LineClassNumber;
					}
				}
			}
		}
		if(Flag == FindWhiteLine)
		{
			DrawLine(ImageWhite1, Lines);
			PutClass(ImageWhite1, Lines, ClassOfLine);
			imshow("W1", ImageWhite1);
		}
		if(Flag == FindRedLine)
		{
			DrawLine(ImageRed1, Lines);
			PutClass(ImageRed1, Lines, ClassOfLine);
			imshow("R1", ImageRed1);
		}
		if(Flag == FindGreenLine)
		{
			DrawLine(ImageGreen1, Lines);
			PutClass(ImageGreen1, Lines, ClassOfLine);
			//imshow("G1", ImageGreen1);
		}
		int FitLineClass[ LineClassNumber+1];
		for(int i = 0; i < LineClassNumber+1; i ++)//直线拟合
		{
			FitLineClass[i] = i;
			Vec4f OutFitLine_i;
			vector<Point2f> PointLine_i;
			bool Line_iOnlyOne = true;
			vector<Point2f> PointInLine_i;
			/*for(int j = 0; j < Lines.size(); j++)
			{
				if(ClassOfLine[j] == i)
				{
					Point Point1 = Point(Lines[j][0], Lines[j][1]);
					PointLine_i.push_back(Point1);
					Point Point2 = Point(Lines[j][2], Lines[j][3]);
					PointLine_i.push_back(Point2);
				}
			}*/
			for(int n = 0; n < Lines.size(); n++)
			{
				if(ClassOfLine[n] == i)
				{
					float x0_n = Lines[n][0];
					float y0_n = Lines[n][1];
					PointInLine_i.push_back(Point2f(x0_n, y0_n));
					float x1_n = Lines[n][2];
					float y1_n = Lines[n][3];
					PointInLine_i.push_back(Point2f(x1_n, y1_n));
					float A = -(y1_n - y0_n);
					float B = (x1_n - x0_n);
					float C = (y1_n - y0_n) * x0_n - (x1_n - x0_n) * y0_n;
					float Den = sqrt(A * A + B * B);
					for(int m = n + 1; m < Lines.size(); m++)
					{
						if(ClassOfLine[m] == i)
						{
							float x0_m = Lines[m][0];
							float y0_m = Lines[m][1];
							float x1_m = Lines[m][2];
							float y1_m = Lines[m][3];
							Point2f MiddlePoint;
							MiddlePoint.x = (x0_n + x0_m) / 2.0;
							MiddlePoint.y = (y0_n + y0_m) / 2.0;
							float Length = fabs(A * MiddlePoint.x + B * MiddlePoint.y + C) / Den;
							if(Length > PixelThreshold/4)
							{
								Line_iOnlyOne = false;
								PointLine_i.push_back(MiddlePoint);
					
								MiddlePoint.x = (x0_n + x1_m) / 2.0;
								MiddlePoint.y = (y0_n + y1_m) / 2.0;
								PointLine_i.push_back(MiddlePoint);
								
								MiddlePoint.x = (x1_n+ x0_m) / 2.0;
								MiddlePoint.y = (y1_n + y0_m) / 2.0;
								PointLine_i.push_back(MiddlePoint);
								
								MiddlePoint.x = (x1_n + x1_m) / 2.0;
								MiddlePoint.y = (y1_n + y1_m) / 2.0;
								PointLine_i.push_back(MiddlePoint);
							}
						}
					}

				}
			}
			if(Line_iOnlyOne)
			{
				for(int m = 0; m < Lines.size(); m++)
				{
					if(ClassOfLine[m] == i)
					{
						Point Point1 = Point(Lines[m][0], Lines[m][1]);
						PointLine_i.push_back(Point1);
						Point Point2 = Point(Lines[m][2], Lines[m][3]);
						PointLine_i.push_back(Point2);
					}
				}
			}
			for(int ii = 0; ii < PointLine_i.size(); ii ++)
			{
				circle(ImageWhite,PointLine_i[ii], 5,Scalar(255,255,0), -1); 
			}
			fitLine(PointLine_i, OutFitLine_i, CV_DIST_L12, 0, 0.01, 0.01);
			float FitA = -OutFitLine_i[1];
			float FitB = OutFitLine_i[0];
			float FitC = OutFitLine_i[1] * OutFitLine_i[2] - OutFitLine_i[0]*OutFitLine_i[3];
			//cout << i << ":\t" << FitA << "\t" << FitB << "\t" << FitC << endl;
			Point FitLinePoint0;
			Point FitLinePoint1;
			if(fabs(FitA/FitB) < 0.01)
			{
				float MaxX = 0;
				float MinX = 1000;
				for(int ii = 0; ii < PointInLine_i.size(); ii++)
				{
					MaxX = PointInLine_i[ii].x > MaxX ? PointInLine_i[ii].x:MaxX;
					MinX =  PointInLine_i[ii].x < MinX ? PointInLine_i[ii].x:MinX;
				}
				Point Point0_0 = Point(MinX, OutFitLine_i[3]);
				Point Point0_1 = Point(MaxX, OutFitLine_i[3]);
				FitLinePoint0 = Point0_0;
				FitLinePoint1 = Point0_1;
			}
			else if(fabs(FitB/FitA) < 0.01)
			{
				float MaxY = 0;
				float MinY = 1000;
				for(int ii = 0; ii < PointInLine_i.size(); ii++)
				{
					MaxY = PointInLine_i[ii].y > MaxY ? PointInLine_i[ii].y:MaxY;
					MinY =  PointInLine_i[ii].y < MinY ? PointInLine_i[ii].y:MinY;
				}
				Point Point90_0 = Point(OutFitLine_i[2], MinY);
				Point Point90_1 = Point(OutFitLine_i[2], MaxY);
				FitLinePoint0 = Point90_0;
				FitLinePoint1 = Point90_1;
			}
			else
			{
				float MaxY = 0;
				float MinY = 1000;
				for(int ii = 0; ii < PointInLine_i.size(); ii++)
				{
					float X0 = PointInLine_i[ii].x;
					float Y0 = PointInLine_i[ii].y;
					float Y;
					Y = FitA * FitA * Y0 - FitA * FitB * X0 - FitB * FitC;
					Y = Y /(FitA * FitA + FitB * FitB);
					MaxY = Y > MaxY ? Y : MaxY;
					MinY = Y < MinY ? Y : MinY;
				}
				float MinY_X = -(FitB * MinY + FitC) / FitA;
				float MaxY_X = -(FitB * MaxY + FitC) / FitA;
				MinY_X = MinY_X > ImageCols ? ImageCols : MinY_X;
				MaxY_X = MaxY_X > ImageCols ? ImageCols : MaxY_X;
				MaxY = MaxY < ImageRows ? MaxY : ImageRows;
				MinY = MinY < ImageRows ? MinY : ImageRows;
				Point PointCommon_0 = Point(MinY_X, MinY);
				Point PointCommon_1 = Point(MaxY_X, MaxY);
				FitLinePoint0 = PointCommon_0;
				FitLinePoint1 = PointCommon_1;
			}
			Vec4i FitLine_i;
			FitLine_i[0] = FitLinePoint0.x;
			FitLine_i[1] = FitLinePoint0.y;
			FitLine_i[2] = FitLinePoint1.x;
			FitLine_i[3] = FitLinePoint1.y;	
			
			OutFitLine.push_back(FitLine_i);
		}
		_OutFitLine.assign(OutFitLine.begin(), OutFitLine.end());
		if(Flag == FindWhiteLine)
		{
			//cout << "找到白线" << _OutFitLine.size() << endl;
			DrawLine(ImageWhite, _OutFitLine);
			PutClass(ImageWhite, _OutFitLine, FitLineClass);
			imshow("W", ImageWhite);
		}
		if(Flag == FindRedLine)
		{
			//cout << "Red" << endl;
			DrawLine(ImageRed, _OutFitLine);
			PutClass(ImageRed, _OutFitLine, FitLineClass);
			imshow("R", ImageRed);
		}
		if(Flag == FindGreenLine)
		{
			//cout << "Green" << endl;
			DrawLine(ImageGreen, _OutFitLine);
			PutClass(ImageRed, _OutFitLine, FitLineClass);
			//imshow("G", ImageGreen);
		}
		return true;
	}
	else if(Flag == FindGreenLine)
	{
		//imshow("G1", ImageGreen1);
		//imshow("G", ImageGreen);
	}
	else if(Flag == FindWhiteLine)
	{
		imshow("W1", ImageWhite1);
		imshow("W", ImageWhite);
	}
	else if(Flag == FindRedLine)
	{
		imshow("R1", ImageRed1);
		imshow("R", ImageRed);
	}
	return false;
}
void DrawLine(Mat &Image, vector<Vec4i> Line)
{
	for(int i = 0; i < Line.size(); i++)
	{
		line( Image, Point(Line[i][0], Line[i][1]), Point(Line[i][2], Line[i][3]), Scalar(0,0,255), 3, 8 );
	}
}

void PutClass(Mat& Image, vector<Vec4i> Line, int * ClassOfLine)
{
	for(int i = 0; i < Line.size(); i++)
	{
		char str[100];
		sprintf(str, "%d", ClassOfLine[i]);
		putText(Image, str, Point((Line[i][0]+Line[i][2])/2, (Line[i][1]+Line[i][3]) / 2) ,CV_FONT_HERSHEY_COMPLEX,0.7,Scalar(0,255,0));
		
		float Angle = atan2(Line[i][3] - Line[i][1], Line[i][2] - Line[i][0]) / CV_PI * 180;
		sprintf(str, "%0.2f", Angle);
		putText(Image, str, Point((Line[i][0]+Line[i][2])/2, (Line[i][1]+Line[i][3]) / 2 - 20) ,CV_FONT_HERSHEY_COMPLEX,0.7,Scalar(0,255,0));
	}
}