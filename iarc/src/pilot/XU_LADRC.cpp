#include <iarc/pilot/XU_LADRC.h>

using namespace Eigen;
//重载构造函数，实例化对象时直接设置LADRC和ESO参数
ADRC::ADRC(float wo, float wc, float b0, float dt) 
: m_observer(wo, b0, dt), m_b(b0) {
	m_kp = wc * wc;
	m_kd = wc + wc;
}

float ADRC::Controller(Vector3f const & xhat, float y_desired) {
	float u0 = m_kp * (y_desired - xhat[0]) - m_kd * xhat[1];
	return (u0 - xhat[2]) / m_b;
}

float ADRC::saturation(float data,float top_limit,float lower_limit)
{
	return data>top_limit?top_limit:data<lower_limit?lower_limit:data;
}

float ADRC::Update(float u, float y, float y_desired) {
	Vector3f xhat = m_observer.Update(u, y);
	return saturation(Controller(xhat, y_desired),0.6,-0.6);
}