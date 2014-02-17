
#include "sampling/Sample.h"
#include "skeleton/Tree.h"
#include "skeleton/Joint.h"
#include "skeleton/Jacobian.h"

using namespace matrices;
using namespace manip_core;


Sample::Sample(const Tree& tree)
{
	Joint * j = tree.root_;
	while(j)
	{
		angles_.push_back(j->GetTheta());
		position_ = j->GetS();
		j = j->pChild_;
	}
	position_ -= tree.GetPosition();
	Jacobian jacob(tree);
	jacobianProd_ = jacob.GetJacobianProduct();
	jacobianProdInverse_ = jacob.GetJacobianProductInverse();
}

Sample::~Sample()
{
	// NOTHING
}

void Sample::LoadIntoTree(Tree& tree) const
{
	Joint * j = tree.root_;
	assert(angles_.size() == tree.GetNumJoints());
	{
		for(Sample::T_Angles::const_iterator it = angles_.begin(); it < angles_.end() && (j != 0); ++it)
		{
			j->SetTheta(*it);
			j = j->pChild_;
		}
		tree.Compute();
	}
}

numeric Sample::forceManipulability (const Vector3& direction) const
{
	numeric r = (direction.transpose()*jacobianProd_*direction);
	return 1/sqrt(r);
}
