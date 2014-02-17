
#include "skeleton/Robot.h"

#include "world/World.h"

//#include "IKSolver.h"
//#include "ForceManipulabilityConstraint.h"

using namespace matrices;
using namespace Eigen;

using namespace std;

namespace manip_core
{
struct RobotPImpl
{
	typedef std::vector<matrices::Vector3>	T_Attach;
	typedef T_Attach::iterator IT_Attach;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	RobotPImpl(const Matrix4& transform, Tree* torsoAndHead, Joint* root)
		: torso_(0)
	{
		SetTransform(transform);
		assert(torsoAndHead);
		torsoAndHead->Init();
		torsoAndHead->Compute();
		torso_ = torsoAndHead;
		InitColumnHierarchy(root);
	}

	~RobotPImpl()
	{
		for(Robot::T_TreeIT it = trees_.begin(); it!= trees_.end(); ++it)
		{
			delete(*it);
		}
		if(torso_)
		{
			delete(torso_);
		}
	}

	void InitColumnHierarchy(Joint* root)
	{
		Joint* j = root;
		while(j)
		{
			Robot::T_Tree trees;
			hierarchy_.push_back(trees);
			j = j->pChild_;
		}
	}

	void SetTransform(const Matrix4& transform)
	{
		toWorldCoo_ = transform;
		toRobotCoo_ = transform.inverse();
	}

	void AddTree(Tree* tree, const matrices::Vector3& attach, const unsigned int columnJoint)
	{
		assert(columnJoint < hierarchy_.size());
		hierarchy_.at(columnJoint).push_back(tree);
		trees_.push_back(tree);
		attaches_.push_back(attach);
	}
	
	T_Attach attaches_;
	//IKSolver ikSolver_;
	Matrix4 toRobotCoo_;
	Matrix4 toWorldCoo_;

	Robot::T_Tree trees_;
	Tree* torso_;
	Robot::T_Hierarchy hierarchy_;
};
}

using namespace manip_core;;

Robot::Robot(const Matrix4& transform, Tree* torsoAndHead)
	: pImpl_(new RobotPImpl(transform, torsoAndHead, torsoAndHead->root_))
	, numTrees_(0)
{
	// which constraints are we going to use ?
//	pImpl_->ikSolver_.Register(new ForceManipulabilityConstraint);
}

void Robot::SetPosOri(const matrices::Matrix4& transform)
{
	pImpl_->SetTransform(transform);
}

Robot::~Robot()
{
	// NOTHING
}

const Tree* Robot::GetTorso() const
{
	return pImpl_->torso_;
}

void Robot::AddTree(Tree* tree,  const matrices::Vector3& attach, const unsigned int columnJoint)
{
	assert(tree);
	tree->Init();
	tree->Compute();
	++ numTrees_;
	pImpl_->AddTree(tree, attach, columnJoint);
}

Tree* Robot::GetTree(Tree::TREE_ID id) const
{
	return id >= numTrees_ ? 0 : pImpl_->trees_[id];
}



void Robot::Reset()
{
	pImpl_->toWorldCoo_.block(0,3,3,1) = pImpl_->toWorldCoo_.block(0,3,3,1) - pImpl_->toWorldCoo_.block(0,3,3,1);
	Rest();
}

const matrices::Matrix4& Robot::ToWorldCoordinates() const
{
	return pImpl_->toWorldCoo_;
}

const matrices::Matrix4& Robot::ToRobotCoordinates() const
{
	return pImpl_->toRobotCoo_;
}

void Robot::Translate(const matrices::Vector3& direction)
{
	pImpl_->toWorldCoo_.block(0,3,3,1) = pImpl_->toWorldCoo_.block(0,3,3,1) + direction;
	pImpl_->toRobotCoo_.block(0,3,3,1) = pImpl_->toRobotCoo_.block(0,3,3,1) - direction;
}

Robot::T_Tree& Robot::GetTrees() const
{
	return pImpl_->trees_;
}

const Robot::T_Hierarchy& Robot::GetHierarchy() const
{
	return pImpl_->hierarchy_;
}

Robot* Robot::Clone() const
{
	Robot* res = new Robot(pImpl_->toWorldCoo_, pImpl_->torso_->Clone());
	unsigned int anchor = 0;
	for(T_Hierarchy::const_iterator it0 = pImpl_->hierarchy_.begin(); it0!= pImpl_->hierarchy_.end(); ++it0)
	{
		for(T_TreeCIT it = it0->begin(); it!= it0->end(); ++it)
		{
			res->AddTree((*it)->Clone(), (pImpl_->attaches_[(*it)->id_]), anchor);
		}
		++anchor;
	}
	return res;
}

void Robot::Rest()
{
	for(T_TreeIT it = pImpl_->trees_.begin(); it!= pImpl_->trees_.end(); ++it)
	{
		//(*it)->UnLockTarget();
		(*it)->ToRest();
	}
}

matrices::Vector3 Robot::MoveTo(const Robot* robot)
{
	Vector3 res = robot->pImpl_->toWorldCoo_.block(0,3,3,1) - pImpl_->toWorldCoo_.block(0,3,3,1);
	pImpl_->SetTransform(robot->pImpl_->toWorldCoo_);
	return res;
}

matrices::Vector3 Robot::MoveTo(const matrices::Vector3& position)
{
	matrices::Vector3 old = pImpl_->toWorldCoo_.block(0,3,3,1);
	pImpl_->toWorldCoo_.block(0,3,3,1) = position;
	pImpl_->toRobotCoo_ = pImpl_->toWorldCoo_.inverse();
	return position - old;
}
