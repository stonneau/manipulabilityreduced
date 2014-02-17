	
#include "world/World.h"
#include "ObstacleVisitor_ABC.h"

#include "world/Intersection.h"
#include "CollisionHandlerDefault.h"
//#include "CollisionHandlerBullet.h"
#include "world/Obstacle.h"
#include "skeleton/Tree.h"
#include "skeleton/Robot.h"

#include <vector>
using namespace std;

//TODO : alignement error with obstacle vector ... hence the ugly stuff

namespace manip_core
{
struct WorldPImpl
{
	WorldPImpl(const World& world)
		: instantiated_(false)
		//, collisionHandler_(/*world*/)
		, collisionHandler_(world)
	{
		//NOTHING
	}

	~WorldPImpl()
	{
		for(T_ObstacleIT it = obstacles_.begin(); it!= obstacles_.end(); ++it)
		{
			delete(*it);
		}
	}

	Intersection intersection_;

	typedef vector<Obstacle*> T_Obstacle;
	typedef T_Obstacle::iterator T_ObstacleIT;
	typedef T_Obstacle::const_iterator T_ObstacleCIT;
	T_Obstacle obstacles_;
	CollisionHandlerDefault collisionHandler_;
	//CollisionHandlerBullet collisionHandler_;
	bool instantiated_;
};
} // namespace manip_core;
using namespace matrices;
using namespace Eigen;
using namespace manip_core;

World::World()
	: pImpl_(new WorldPImpl(*this))
{
	//NOTHING
}

World::~World()
{
	//NOTHING
}

void World::Instantiate(bool activateCollision)
{
	if(activateCollision)
	{
		pImpl_->collisionHandler_.Instantiate();
		pImpl_->instantiated_ = true;
	}
}

void World::AddObstacle(Obstacle* obstacle)
{
	assert(obstacle);
	assert(!(pImpl_->instantiated_));
	pImpl_->obstacles_.push_back(obstacle);
	pImpl_->collisionHandler_.AddObstacle(obstacle);
}

void World::Accept(ObstacleVisitor_ABC& visitor) const
{
	//assert(pImpl_->instantiated_);
	for(WorldPImpl::T_ObstacleIT it = pImpl_->obstacles_.begin(); it!= pImpl_->obstacles_.end(); ++it)
	{
		visitor.Visit(*(*it));
	}
}

bool World::GetTarget(const Robot& robot, const Tree& tree, const Vector3& direction, Vector3& target) const
{
	//TODO : pattern patron pour choisir méthode de sélection 
	//phase 1 : le premier qui intersecte ...
	assert(pImpl_->instantiated_);
	for(WorldPImpl::T_ObstacleIT it = pImpl_->obstacles_.begin(); it!= pImpl_->obstacles_.end(); ++it)
	{
		if(pImpl_->intersection_.Intersect(robot, tree, *(*it), target))
		{
			return true;
		}
	}
	return false;
}

bool World::GetClosestTarget(const Robot& robot, const Tree& tree, const Vector3& from, Vector3& target) const
{
	assert(pImpl_->instantiated_);
	numeric minDistance = 10000;
	Vector3 currentTarget;
	bool found = false;
	// get closest obstacle
	for(WorldPImpl::T_ObstacleIT it = pImpl_->obstacles_.begin(); it!= pImpl_->obstacles_.end(); ++it)
	{
		if(pImpl_->intersection_.IntersectClosest(robot, tree, from, (*(*it)), currentTarget))
		{
			numeric currentDistance = (currentTarget-from).norm();
			if(currentDistance < minDistance)
			{
				target = currentTarget;
				minDistance = currentDistance;
				found = true;
			}
		}
	}
	return found;
}

bool World::IsColliding(const Robot& robot, const Tree& tree) const
{
	return pImpl_->instantiated_ && pImpl_->collisionHandler_.IsColliding(robot, tree);
}


bool World::IsSoftColliding(const Robot& robot, const Tree& tree) const
{
	return pImpl_->instantiated_ && pImpl_->collisionHandler_.IsSoftColliding(robot, tree);
}


bool World::IsReachable(const Robot& robot, const Tree& tree, const Vector3& target) const
{
	//assert(pImpl_->instantiated_);
	return pImpl_->intersection_.Intersect(tree, (matrix4TimesVect3(robot.ToRobotCoordinates(), target)));
}

bool World::IsReachable(const Robot& robot, const Tree& tree, const Obstacle& obstacle) const
{
	Vector3  treePos = tree.GetPosition(); int room = ( tree.GetPosition()(1) > 0 ) ? 1 : - 1;
	const matrices::Matrix4& transform( robot.ToRobotCoordinates());
	numeric p1robPos = matrices::matrix4TimesVect3( transform, obstacle.GetP1())(1) + 0.05 * room;
	numeric p3robPos = matrices::matrix4TimesVect3( transform, obstacle.GetP3())(1) + 0.05 * room;
	if(! (p1robPos * room >= 0 || p3robPos * room >= 0))
	{
		return false;
	}
	//assert(pImpl_->instantiated_);
	return pImpl_->intersection_.Intersect(robot, tree, obstacle);
}
