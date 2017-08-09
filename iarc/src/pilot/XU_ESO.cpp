#include <iarc/pilot/XU_ESO.h>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;
//重载构造函数，实例化对象时直接设置参数
ESO::ESO(float wo, float b0, float dt) {
	SetParameters(wo, b0, dt);
}

void ESO::SetParameters(float wo, float b0, float dt) {
	Matrix3f A;
	Vector3f B;
	Vector3f L;
	Matrix<float, 1, 3> C;
	Matrix<float, 3, 2> B_obs_ct;
	Matrix3f A_obs_ct;
      
	A << 0, 1, 0, 
	     0, 0, 1, 
	     0, 0, 0;
	
	B << 0, b0, 0;
	
	C << 1, 0, 0;
	
	L << 3*wo, 3*wo*wo, wo*wo*wo;
	
	A_obs_ct = A - L * C;
	B_obs_ct << B, L;

	Discretize(A_obs_ct, B_obs_ct, dt);

}
//离散化
void ESO::Discretize(Matrix3f const & A_obs_ct, 
	Matrix<float, 3, 2> const & B_obs_ct, float dt) {
	Matrix<float, 5, 5> discretization, discretization_exp;

	discretization << A_obs_ct, B_obs_ct, MatrixXf::Zero(2, 5);
	discretization_exp = (discretization * dt).exp();

	m_A_obs_dt = discretization_exp.block<3, 3>(0, 0);
	m_B_obs_dt = discretization_exp.block<3, 2>(0, 3);
}

void ESO::SetState(Eigen::Vector3f const & xhat) {
	m_xhat = xhat;
}
	

Vector3f ESO::Update(float u, float y) {
	Vector2f u_obs;
	u_obs << u, y;
	m_xhat = m_A_obs_dt * m_xhat + m_B_obs_dt * u_obs;
	return m_xhat;
}
