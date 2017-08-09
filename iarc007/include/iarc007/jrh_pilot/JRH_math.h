/*
 * JRH_math.h
 *
 *  Created on: 2016年3月10日
 *      Author: JRH
 */


#ifndef  JRH_MATH_H_
#define JRH_MATH_H_


#include <opencv/cv.h>
#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include<math.h>

void Quaternion_To_Euler(CvMat *q_att, CvMat *att);
void Euler_To_Matrix(float roll, float pitch, float yaw, CvMat *R);
void From_Field_To_Body(CvMat *R,CvMat *R_initial,CvMat *R_avalible);
float Low_Pass(float h);


#endif 
