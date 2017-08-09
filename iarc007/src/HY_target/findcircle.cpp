/*
 * findcircle.cpp
 *
 *  Created on: 2016年8月20日
 *      Author: hou
 */

/*
 * ExtendHoughCircle.cpp
 *
 *  Created on: 2016年8月17日
 *      Author: hou
 */

//----------------------------------------------------------------------------
//   20014.11.26 by Charles

/*	circle_found为找圆结果的结构
	FindCircles为最终对外的接口，其他都是内部函数
//优化处理后打包好的HoughCircles接口，cv核心算法的优化补充在该函数中添加如下：
//1、添加cvCanny的低阈值low_threshold，参数控制可以更灵活
//2、可选择性的手动输入轮廓图代替cv核心算法内自动调用的canny，可以更有效的进行预处理
//3、添加分数过滤，将找出的圆与实际轮廓对比重合率（如果输入了轮廓图则与轮廓图比，否则用算法内部自动生成的轮廓图对比），
//		按照分数高低排序,并过滤掉分数过低的圆
*/
//----------------------------------------------------------------------------

#include "iarc007/hy_target/findcircle.h"



bool SortCircleFound( const circle_found &v1, const circle_found &v2)//注意：本函数的参数的类型一定要与vector中元素的类型一致
{
	return v1.score > v2.score;//降序排列
}

const int STORAGE_SIZE = 1 << 12;

#define hough_cmp_gt(l1,l2) (aux[l1] > aux[l2])

#define cvSobel_Core 5
#define overlap_check_brush 2

enum { XY_SHIFT = 16, XY_ONE = 1 << XY_SHIFT, DRAWING_STORAGE_BLOCK = (1<<12) - 256 };

static CV_IMPLEMENT_QSORT_EX( icvHoughSortDescent32s, int, hough_cmp_gt, const int* )

static void seqToMat(const CvSeq* seq, cv::OutputArray _arr)
{
	if( seq && seq->total > 0 )
	{
		_arr.create(1, seq->total, seq->flags, -1, true);
		cv::Mat arr = _arr.getMat();
		cvCvtSeqToArray(seq, arr.data);
	}
	else
		_arr.release();
}

static void icvFindCirclesGradient( CvMat* img, cv::Mat &contour_img, float dp, float min_dist,int min_radius, int max_radius,
	int low_threshold, int high_threshold,int acc_threshold,CvSeq* circles, int circles_max )
{
	const int SHIFT = 10, ONE = 1 << SHIFT;
	cv::Ptr<CvMat> dx, dy;
	cv::Ptr<CvMat> edges, accum, dist_buf;
	std::vector<int> sort_buf;
	cv::Ptr<CvMemStorage> storage;

	int x, y, i, j, k, center_count, nz_count;
	float min_radius2 = (float)min_radius*min_radius;
	float max_radius2 = (float)max_radius*max_radius;
	int rows, cols, arows, acols;
	int astep, *adata;
	float* ddata;
	CvSeq *nz, *centers;
	float idp, dr;
	CvSeqReader reader;

	edges = cvCreateMat( img->rows, img->cols, CV_8UC1 );
	cvCanny( img, edges, low_threshold, high_threshold, 3 );	//添加cvCanny的低阈值low_threshold，参数控制可以更灵活
	//contour_img=cv::Mat(edges)+0;
	//cv::Mat  edges_mat(edges);
	contour_img=cv::Mat(edges);

	dx = cvCreateMat( img->rows, img->cols, CV_16SC1 );
	dy = cvCreateMat( img->rows, img->cols, CV_16SC1 );
	///////////////////////////////////计算方向导数的Sobel核大小，重要参数////////////////////////////////////////
	cvSobel( img, dx, 1, 0, cvSobel_Core );
	cvSobel( img, dy, 0, 1, cvSobel_Core );

	if( dp < 1.f )
		dp = 1.f;
	idp = 1.f/dp;
	accum = cvCreateMat( cvCeil(img->rows*idp)+2, cvCeil(img->cols*idp)+2, CV_32SC1 );
	cvZero(accum);

	storage = cvCreateMemStorage();
	nz = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );
	centers = cvCreateSeq( CV_32SC1, sizeof(CvSeq), sizeof(int), storage );

	rows = img->rows;
	cols = img->cols;
	arows = accum->rows - 2;
	acols = accum->cols - 2;
	adata = accum->data.i;
	astep = accum->step/sizeof(adata[0]);
	// Accumulate circle evidence for each edge pixel
	for( y = 0; y < rows; y++ )
	{
		const uchar* edges_row = edges->data.ptr + y*edges->step;
		const short* dx_row = (const short*)(dx->data.ptr + y*dx->step);
		const short* dy_row = (const short*)(dy->data.ptr + y*dy->step);

		for( x = 0; x < cols; x++ )
		{
			float vx, vy;
			int sx, sy, x0, y0, x1, y1, r;
			CvPoint pt;

			vx = dx_row[x];
			vy = dy_row[x];

			if( !edges_row[x] || (vx == 0 && vy == 0) )
				continue;

			float mag = sqrt(vx*vx+vy*vy);
			assert( mag >= 1 );
			sx = cvRound((vx*idp)*ONE/mag);
			sy = cvRound((vy*idp)*ONE/mag);

			x0 = cvRound((x*idp)*ONE);
			y0 = cvRound((y*idp)*ONE);
			// Step from min_radius to max_radius in both directions of the gradient
			for(int k1 = 0; k1 < 2; k1++ )
			{
				x1 = x0 + min_radius * sx;
				y1 = y0 + min_radius * sy;

				for( r = min_radius; r <= max_radius; x1 += sx, y1 += sy, r++ )
				{
					int x2 = x1 >> SHIFT, y2 = y1 >> SHIFT;
					if( (unsigned)x2 >= (unsigned)acols ||
						(unsigned)y2 >= (unsigned)arows )
						break;
					adata[y2*astep + x2]++;
				}

				sx = -sx; sy = -sy;
			}

			pt.x = x; pt.y = y;
			cvSeqPush( nz, &pt );
		}
	}

	nz_count = nz->total;
	if( !nz_count )
		return;
	//Find possible circle centers
	for( y = 1; y < arows - 1; y++ )
	{
		for( x = 1; x < acols - 1; x++ )
		{
			int base = y*(acols+2) + x;
			if( adata[base] > acc_threshold &&
				adata[base] > adata[base-1] && adata[base] > adata[base+1] &&
				adata[base] > adata[base-acols-2] && adata[base] > adata[base+acols+2] )
				cvSeqPush(centers, &base);
		}
	}

	center_count = centers->total;
	if( !center_count )
		return;

	sort_buf.resize( MAX(center_count,nz_count) );
	cvCvtSeqToArray( centers, &sort_buf[0] );

	icvHoughSortDescent32s( &sort_buf[0], center_count, adata );
	cvClearSeq( centers );
	cvSeqPushMulti( centers, &sort_buf[0], center_count );

	dist_buf = cvCreateMat( 1, nz_count, CV_32FC1 );
	ddata = dist_buf->data.fl;

	dr = dp;
	min_dist = MAX( min_dist, dp );
	min_dist *= min_dist;
	// For each found possible center
	// Estimate radius and check support
	for( i = 0; i < centers->total; i++ )
	{
		int ofs = *(int*)cvGetSeqElem( centers, i );
		y = ofs/(acols+2);
		x = ofs - (y)*(acols+2);
		//Calculate circle's center in pixels
		float cx = (float)((x + 0.5f)*dp), cy = (float)(( y + 0.5f )*dp);
		float start_dist, dist_sum;
		float r_best = 0;
		int max_count = 0;
		// Check distance with previously detected circles
		for( j = 0; j < circles->total; j++ )
		{
			float* c = (float*)cvGetSeqElem( circles, j );
			if( (c[0] - cx)*(c[0] - cx) + (c[1] - cy)*(c[1] - cy) < min_dist )
				break;
		}

		if( j < circles->total )
			continue;
		// Estimate best radius
		cvStartReadSeq( nz, &reader );
		for( j = k = 0; j < nz_count; j++ )
		{
			CvPoint pt;
			float _dx, _dy, _r2;
			CV_READ_SEQ_ELEM( pt, reader );
			_dx = cx - pt.x; _dy = cy - pt.y;
			_r2 = _dx*_dx + _dy*_dy;
			if(min_radius2 <= _r2 && _r2 <= max_radius2 )
			{
				ddata[k] = _r2;
				sort_buf[k] = k;
				k++;
			}
		}

		int nz_count1 = k, start_idx = nz_count1 - 1;
		if( nz_count1 == 0 )
			continue;
		dist_buf->cols = nz_count1;
		cvPow( dist_buf, dist_buf, 0.5 );
		icvHoughSortDescent32s( &sort_buf[0], nz_count1, (int*)ddata );

		dist_sum = start_dist = ddata[sort_buf[nz_count1-1]];
		for( j = nz_count1 - 2; j >= 0; j-- )
		{
			float d = ddata[sort_buf[j]];

			if( d > max_radius )
				break;

			if( d - start_dist > dr )
			{
				float r_cur = ddata[sort_buf[(j + start_idx)/2]];
				if( (start_idx - j)*r_best >= max_count*r_cur ||
					(r_best < FLT_EPSILON && start_idx - j >= max_count) )
				{
					r_best = r_cur;
					max_count = start_idx - j;
				}
				start_dist = d;
				start_idx = j;
				dist_sum = 0;
			}
			dist_sum += d;
		}
		// Check if the circle has enough support
		if( max_count > acc_threshold )
		{
			float c[3];
			c[0] = cx;
			c[1] = cy;
			c[2] = (float)r_best;
			cvSeqPush( circles, c );
			if( circles->total > circles_max )
				return;
		}
	}
}

CV_IMPL CvSeq* cvFindCircles( CvArr* src_image, cv::Mat &contour_image, void* circle_storage,float dp, int min_dist,
	int low_threshold, int high_threshold,int acc_threshold,int min_radius, int max_radius)
{
	CvSeq* result = 0;

	CvMat stub, *img = (CvMat*)src_image;

	CvMat* mat = 0;
	CvSeq* circles = 0;
	CvSeq circles_header;
	CvSeqBlock circles_block;
	int circles_max = INT_MAX;


	img = cvGetMat( img, &stub );

	if( !CV_IS_MASK_ARR(img))
		CV_Error( CV_StsBadArg, "The source image must be 8-bit, single-channel" );

	if ( contour_image.cols==img->cols && contour_image.rows==img->rows )
	{
		if( contour_image.type()!=CV_8UC1)
			CV_Error( CV_StsBadArg, "The contour image must be 8-bit, single-channel" );
	}

	if( !circle_storage )
		CV_Error( CV_StsNullPtr, "NULL destination" );

	if( dp <= 0 || min_dist <= 0 || low_threshold <= 0 || high_threshold <= 0 || acc_threshold <= 0 )
		CV_Error( CV_StsOutOfRange, "dp, min_dist, canny_threshold and acc_threshold must be all positive numbers" );

	min_radius = MAX( min_radius, 0 );
	if( max_radius <= 0 )
		max_radius = MAX( img->rows, img->cols );
	else if( max_radius <= min_radius )
		max_radius = min_radius + 2;

	if( CV_IS_STORAGE( circle_storage ))
	{
		circles = cvCreateSeq( CV_32FC3, sizeof(CvSeq),
			sizeof(float)*3, (CvMemStorage*)circle_storage );
	}
	else if( CV_IS_MAT( circle_storage ))
	{
		mat = (CvMat*)circle_storage;

		if( !CV_IS_MAT_CONT( mat->type ) || (mat->rows != 1 && mat->cols != 1) ||
			CV_MAT_TYPE(mat->type) != CV_32FC3 )
			CV_Error( CV_StsBadArg,
			"The destination matrix should be continuous and have a single row or a single column" );

		circles = cvMakeSeqHeaderForArray( CV_32FC3, sizeof(CvSeq), sizeof(float)*3,
			mat->data.ptr, mat->rows + mat->cols - 1, &circles_header, &circles_block );
		circles_max = circles->total;
		cvClearSeq( circles );
	}
	else
		CV_Error( CV_StsBadArg, "Destination is not CvMemStorage* nor CvMat*" );

	icvFindCirclesGradient( img, contour_image, (float)dp, (float)min_dist,
		min_radius, max_radius, low_threshold,high_threshold,
		acc_threshold, circles, circles_max );

	if( mat )
	{
		if( mat->cols > mat->rows )
			mat->cols = circles->total;
		else
			mat->rows = circles->total;
	}
	else
		result = circles;

	return result;
}

UINT CheckCircleOverlap( cv::Mat& img, cv::Point center, int radius, UINT &overlap_num, int shift = 0 )
{
	CV_Assert( radius >= 0 && 0 <= shift && shift <= XY_SHIFT );

	cv::Size size = img.size();
	size_t step = img.step;
	int pix_size = (int)img.elemSize();
	uchar* ptr = img.data;
	int err = 0, dx = radius, dy = 0, plus = 1, minus = (radius << 1) - 1;
	int inside = center.x >= radius && center.x < size.width - radius &&
		center.y >= radius && center.y < size.height - radius;
	UINT total_num=0;
	overlap_num=0;

#define ICV_COUNT_POINT( ptr, x )	if( *(ptr + (x)*pix_size) ){overlap_num++;total_num++;}else{total_num++;}

	while( dx >= dy )
	{
		int mask;
		int y11 = center.y - dy, y12 = center.y + dy, y21 = center.y - dx, y22 = center.y + dx;
		int x11 = center.x - dx, x12 = center.x + dx, x21 = center.x - dy, x22 = center.x + dy;

		if( inside )
		{
			uchar *tptr0 = ptr + y11 * step;
			uchar *tptr1 = ptr + y12 * step;

			ICV_COUNT_POINT( tptr0, x11 );
			ICV_COUNT_POINT( tptr1, x11 );
			ICV_COUNT_POINT( tptr0, x12 );
			ICV_COUNT_POINT( tptr1, x12 );

			tptr0 = ptr + y21 * step;
			tptr1 = ptr + y22 * step;

			ICV_COUNT_POINT( tptr0, x21 );
			ICV_COUNT_POINT( tptr1, x21 );
			ICV_COUNT_POINT( tptr0, x22 );
		}
		else if( x11 < size.width && x12 >= 0 && y21 < size.height && y22 >= 0 )
		{
			if( (unsigned)y11 < (unsigned)size.height )
			{
				uchar *tptr = ptr + y11 * step;

				if( x11 >= 0 )
					ICV_COUNT_POINT( tptr, x11 );
				if( x12 < size.width )
					ICV_COUNT_POINT( tptr, x12 );
			}

			if( (unsigned)y12 < (unsigned)size.height )
			{
				uchar *tptr = ptr + y12 * step;

				if( x11 >= 0 )
					ICV_COUNT_POINT( tptr, x11 );
				if( x12 < size.width )
					ICV_COUNT_POINT( tptr, x12 );
			}

			if( x21 < size.width && x22 >= 0 )
			{
				if( (unsigned)y21 < (unsigned)size.height )
				{
					uchar *tptr = ptr + y21 * step;

					if( x21 >= 0 )
						ICV_COUNT_POINT( tptr, x21 );
					if( x22 < size.width )
						ICV_COUNT_POINT( tptr, x22 );
				}

				if( (unsigned)y22 < (unsigned)size.height )
				{
					uchar *tptr = ptr + y22 * step;

					if( x21 >= 0 )
						ICV_COUNT_POINT( tptr, x21 );
					if( x22 < size.width )
						ICV_COUNT_POINT( tptr, x22 );
				}
			}
		}
		dy++;
		err += plus;
		plus += 2;

		mask = (err <= 0) - 1;

		err -= minus & mask;
		dx += mask;
		minus -= mask & 2;
	}

#undef  ICV_COUNT_POINT
	return total_num;
}

void ExtendCV::FindCircles( cv::InputArray _image, cv::vector<circle_found>& _circles,float dp, int min_dist,
	int low_threshold, int high_threshold,int acc_threshold,int minRadius, int maxRadius,
	float minScore, cv::InputArray _contour_image )
{
	cv::Ptr<CvMemStorage> storage = cvCreateMemStorage(STORAGE_SIZE);
	cv::Mat image = _image.getMat();
	if (image.channels()!=1)
	{
		cv::cvtColor(image,image,CV_BGR2GRAY);
	}
	CvMat c_image = image;
	cv::Mat contour_image;
cv::Mat temp_image=_image.getMat();

	cv::vector<cv::Vec3f> circles_all;
	CvSeq* seq = cvFindCircles( &c_image, contour_image, storage, dp, min_dist, low_threshold, high_threshold,
			acc_threshold, minRadius,maxRadius );


	seqToMat(seq, circles_all);
	vector<vector<Point> > contours;
	printf("2\n");
	Canny(image, temp_image, low_threshold, high_threshold, 3);
	//findContours(contour_image,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
	findContours(temp_image,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);

	drawContours(temp_image,contours,-1,cv::Scalar(255),overlap_check_brush);
	//imshow("img",image);
// 	Mat draw_mat=Mat(contour_image.rows,contour_image.cols,CV_8UC1);
	UINT NonZeroCount_circle,NonZeroCount_overlap;
	float overlap_score=0;
	_circles.clear();
	int vector_size=circles_all.size();

	for (int i=0,j=0;i<vector_size;i++)
	{
		NonZeroCount_circle=CheckCircleOverlap(temp_image,
			cv::Point(circles_all[i][0],circles_all[i][1]),circles_all[i][2],NonZeroCount_overlap);

		overlap_score=(float)NonZeroCount_overlap/(float)NonZeroCount_circle;
		if (overlap_score>minScore)
		{
			_circles.push_back(circle_found());
			_circles[j].score=overlap_score;
			_circles[j].circle=circles_all[i];
			j++;	//分开计数，i记录的是比对过的总数，j记录的是比对后可以通过的计数
		}
	}
	std::sort(_circles.begin(),_circles.end(),SortCircleFound);
}




