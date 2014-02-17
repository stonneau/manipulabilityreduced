
#include "skeleton/Tree.h"
#include "skeleton/Joint.h"


using namespace matrices;
using namespace Eigen;
using namespace manip_core;

Tree::Tree(TREE_ID id)
: sphereRadius_(0)
, id_(id)
{
	// NOTHING
}


Tree::~Tree()
{
	Joint* n = root_;
	Joint* n1 = n;
	while (n)
	{
		n = n->pChild_;
		delete n1;
		n1 = n;
	}
}

void Tree::ToRest()
{
	Joint* j = root_;
	while(j)
	{
		j->ToRest();
		j = j->pChild_;
	}
	Compute();
}

void Tree::InsertRoot(Joint* root)
{
	assert(root_ == 0);
	root_ = root;
	root->r_ = root->attach_;
	assert(!(root->pChild_));
	numJoint_ = 1;
}

void Tree::InsertChild(Joint* parent, Joint* child)
{
	assert(parent);
	parent->pChild_ = child;
	child->pRealparent_ = parent;
	child->r_ = child->attach_ - child->pRealparent_->attach_;
	sphereRadius_ += child->r_.norm();
	assert(!(child->pChild_));
	// updating numJoint
	Joint * j = root_;
	numJoint_ = 0;
	while(j)
	{
		++numJoint_;
		j = j->pChild_;
	}
}

namespace
{
	Joint* SearchJoint(Joint* current, int index)
	{
		if(index ==0) return current;
		return current->pChild_ ? SearchJoint(current->pChild_, --index) : 0;
	}
}

// Get the joint with the index value
Joint* Tree::GetJoint(int index) const
{
	return SearchJoint(root_, index);
}

// Returns the global position of the effector.
const Vector3& Tree::GetEffectorPosition() const
{
	Joint* j = root_;
	while(j->pChild_)
	{
		j = j->pChild_;
	};
	return (j->s_);  
}

namespace
{
	void ComputeTree(Joint* joint)
	{
		if (joint != 0) {
			joint->ComputeS();
			joint->ComputeW();
			ComputeTree(joint->pChild_);
		}
	}

	// Recursively initialize this below the Joint
	void InitTree(Joint* joint)
	{
		if (joint != 0)
		{
			joint->InitJoint();
			InitTree(joint->pChild_);
		}
	}
}

void Tree::Compute(void)
{ 
	ComputeTree(root_); 
}

// Initialize all Joints in the this
void Tree::Init(void)
{
	InitTree(root_);
	ToRest();
	Compute();
}

const Vector3& Tree::GetPosition() const
{
	root_->ComputeS();
	return root_->GetS();
}

Tree* Tree::Clone() const
{
	Tree* res = new Tree(id_);
	Joint* n1 = root_;
	if(n1)
	{
		Joint* n2(n1->pChild_);
		n1 = n1->Clone(); // clones don't have children !
		res->InsertRoot(n1);
		Joint* tmp(0);
		while(n2)
		{
			tmp = n2;
			n2 = n2->Clone();
			res->InsertChild(n1, n2);
			n1 = n2;
			n2 = tmp->pChild_;
		}
	}
	return res;
}


