
#ifndef _CLASS_JACOBIAN
#define _CLASS_JACOBIAN

#include "tools/MatrixDefs.h"
#include "tools/MatrixDefsInternal.h"

namespace manip_core
{
class Tree;

class Jacobian {

public:
	 Jacobian(const Tree& tree);
	~Jacobian();

private:
	Jacobian& Jacobian::operator =(const Jacobian&);
	Jacobian(const Jacobian&);

public:
	void  SetJacobian(const matrices::MatrixX& jacobian);
	void  ComputeAll(const Tree& tree); // recomputes jacobian
	void  ComputeAll(); // recomputes everything but the jacobian
	void  ComputeJacobian(const Tree& tree);
	const matrices::MatrixX& GetNullspace();
	const matrices::MatrixX& GetJacobian();
		  matrices::MatrixX  GetJacobianCopy();
	const matrices::MatrixX& GetJacobianInverse();
	const matrices::Matrix3& GetJacobianProduct();
	const matrices::Matrix3& GetJacobianProductInverse();
	void GetEllipsoidAxes(matrices::Vector3& /*u1*/, matrices::Vector3& /*u2*/, matrices::Vector3& /*u3*/);
	void GetEllipsoidAxes(matrices::Vector3& /*u1*/, matrices::Vector3& /*u2*/, matrices::Vector3& /*u3*/, numeric& /*sig1*/, numeric& /*sig2*/, numeric& /*sig3*/);

	void  GetNullspace(const matrices::MatrixX /*pseudoId*/, matrices::MatrixX& /*result*/);

private:
	void ComputeSVD();

private:
	bool computeInverse_;
	bool computeProduct_;
	bool computeProductInverse_;
	bool computeJacSVD_;
	bool computeNullSpace_;
	void Invalidate();

private:
	matrices::Matrix3 jacobianProductInverse_;
	matrices::Matrix3 jacobianProduct_;
	matrices::MatrixX jacobian_;
	matrices::MatrixX jacobianInverse_;
	matrices::MatrixX jacobianInverseNoDls_;
	matrices::MatrixX Identitymin_;
	Eigen::JacobiSVD<matrices::MatrixX> svd_;
	Eigen::JacobiSVD<matrices::MatrixX> svdProduct_;
};
}//namespace manip_core

#endif //_CLASS_JACOBIAN