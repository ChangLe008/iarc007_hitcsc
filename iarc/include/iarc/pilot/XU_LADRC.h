//#ifndef XU_ADRC_H
//#define XU_ADRC_H
#include <stdlib.h>
#include <stdio.h>
#include <iarc/pilot/leo_math.h>
#include <iarc/pilot/control.h>
#include <string>
#include <Eigen/Core>
#include <iarc/pilot/XU_ESO.h>

class ADRC {
	ESO m_observer;
	float m_kp;
	float m_kd;
	float m_b;

	float Controller(Eigen::Vector3f const & xhat, float y_desired);
public:
	ADRC(float wo, float wc, float b0, float dt);
	virtual ~ADRC() { };
	float Update(float u, float y, float y_desired);
	float saturation(float data,float upper_limit,float lower_limit);
};

//#endif
