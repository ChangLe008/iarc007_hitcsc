/*
 * target.h
 *
 *  Created on: 2016年3月13日
 *      Author: exbot
 */
/*
 * target.h
 *
 *  Created on: 2016年3月13日
 *      Author: exbot
 */
#ifndef TARGET_H_
#define TARGET_H_

#include "iarc007/hy_target/DecHOG.h"
#include "iarc007/hy_target/common.h"
#include "iarc007/hy_target/parameter.h"
#include "iarc007/hy_target/Track.h"
#include "iarc007/hy_target/aprilTags.h"
#include "iarc007/hy_target/findcircle.h"
#include "iarc007/hy_target/fun.h"
#include "iarc007/hy_target/L_target.h" 

void target(IplImage* img,CvMat* R,double h,PosPoint* point,int mode,IplImage* dst,float air_pos[],long s_num);

Track_Mode huizhi(float h);
float find_direction(target_description& H_target,CvMat* intrinsic,CvMat* distortion,CvMat* R,float h);

#endif /* TARGET_H_ */
