
#include "skeleton/Jacobian.h"
#include "skeleton/Tree.h"
#include "skeleton/Joint.h"

using namespace manip_core;
using namespace matrices;
using namespace Eigen;

Jacobian::Jacobian(const Tree& tree)
{
	ComputeJacobian(tree);
}

Jacobian::~Jacobian()
{
	//NOTHING
}

void Jacobian::SetJacobian(const MatrixX& jacobian)
{
	jacobian_ = jacobian;
	Invalidate();
}

void Jacobian::Invalidate()
{
	computeInverse_ = true; computeProduct_ = true; computeProductInverse_ = true;
	computeJacSVD_ = true; computeNullSpace_ = true;
}

void Jacobian::ComputeJacobian(const Tree& tree)
{
	Invalidate();
	jacobian_ = MatrixX(3,tree.GetNumJoints()); // à cause de son incrémentation débile
	// Traverse this to find all end effectors
	Vector3 temp;
	Joint* n = tree.root_;
	while (n) 
	{	
		if (n->IsEffector())
		{
			int i = 0;
			// Find all ancestors (they will usually all be joints)
			// Set the corresponding entries in the Jacobian J
			Joint* m = n->pRealparent_;
			int j = tree.GetNumJoints() - 1;
			while (m) {
				temp = m->GetS();			// joint pos.
				temp -= n->GetS();			// -(end effector pos. - joint pos.)
				Vector3 tmp2 = temp.cross(m->GetW());			// cross product with joint rotation axis
				jacobian_.col(j-1) = tmp2;
				m = m->pRealparent_;
				j--;
			}
		}
		n = (n->IsEffector() ? 0 : n->pChild_);
	}
}

void Jacobian::GetEllipsoidAxes(matrices::Vector3& u1, matrices::Vector3& u2, matrices::Vector3& u3)
{
	GetJacobianProduct();
	svdProduct_ = Eigen::JacobiSVD<MatrixX>(jacobianProduct_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	u1 = svdProduct_.matrixU().block(0,0,3,1) / (svdProduct_.singularValues()(0) + 0.000000000000000000000000001);
	u2 = svdProduct_.matrixU().block(0,1,3,1) / (svdProduct_.singularValues()(1) + 0.000000000000000000000000001);
	u3 = svdProduct_.matrixU().block(0,2,3,1) / (svdProduct_.singularValues()(2) + 0.000000000000000000000000001);
	// TODO
}

void Jacobian::GetEllipsoidAxes(matrices::Vector3& u1, matrices::Vector3& u2, matrices::Vector3& u3, numeric& sig1, numeric& sig2, numeric& sig3)
{
	GetJacobianProduct();
	svdProduct_ = Eigen::JacobiSVD<MatrixX>(jacobianProduct_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	u1 = svdProduct_.matrixU().block(0,0,3,1);
	u2 = svdProduct_.matrixU().block(0,1,3,1);
	u3 = svdProduct_.matrixU().block(0,2,3,1);
	sig1 = 1. / (svdProduct_.singularValues()(0) + 0.000000000000000000000000001);
	sig2 = 1. / (svdProduct_.singularValues()(1) + 0.000000000000000000000000001);
	sig3 = 1. / (svdProduct_.singularValues()(2) + 0.000000000000000000000000001);
}


void Jacobian::ComputeAll(const Tree& tree)
{
	ComputeJacobian(tree);
	ComputeAll();
}
void Jacobian::ComputeAll()
{
	ComputeSVD();
	GetJacobianInverse();
	GetJacobianProduct();
	GetJacobianProductInverse();
	GetNullspace();
}

const MatrixX& Jacobian::GetJacobian()
{
	return jacobian_;
}

MatrixX Jacobian::GetJacobianCopy()
{
	return jacobian_;
}
const MatrixX& Jacobian::GetJacobianInverse()
{
	if(computeInverse_)
	{
		computeInverse_ = false;
		jacobianInverse_ = jacobian_;
		PseudoInverseDLS(jacobianInverse_, 1.f); // tmp while figuring out how to chose lambda
	}
	return jacobianInverse_;
}

const MatrixX& Jacobian::GetNullspace()
{
	if(computeNullSpace_)
	{
		computeNullSpace_ = false;
		MatrixX id = MatrixX::Identity(jacobian_.cols(), jacobian_.cols());
		ComputeSVD();
		MatrixX res = MatrixX::Zero(id.rows(), id.cols());
		for(int i =0; i < svd_.matrixV().cols(); ++ i)
		{
			VectorX v = svd_.matrixV().col(i);
			res += v * v.transpose();
		}
		Identitymin_ = id - res;
	}
	return Identitymin_;
}

void Jacobian::GetNullspace(const MatrixX pseudoId, MatrixX& result)
{
		GetNullspace(); // computing inverse jacobian

		MatrixX id = MatrixX::Identity(Identitymin_.rows(), Identitymin_.cols());
		result = pseudoId - (id + Identitymin_);
}

const Matrix3& Jacobian::GetJacobianProduct()
{
	if(computeProduct_)
	{
		computeProduct_ = false;
		jacobianProduct_ = jacobian_ * jacobian_.transpose();
	}
	return jacobianProduct_;
}

const Matrix3& Jacobian::GetJacobianProductInverse()
{
	if(computeProductInverse_)
	{
		computeProductInverse_ = false;
		Eigen::JacobiSVD<Matrix3> svd = Eigen::JacobiSVD<Matrix3>(jacobianProduct_, Eigen::ComputeFullU | Eigen::ComputeFullV);
		PseudoInverseSVDDLS(jacobianProduct_, svd, jacobianProductInverse_);
	}
	return jacobianProductInverse_;
}

void Jacobian::ComputeSVD()
{
	if(computeJacSVD_)
	{
		computeJacSVD_ = false;
		svd_ = Eigen::JacobiSVD<MatrixX>(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	}
}
