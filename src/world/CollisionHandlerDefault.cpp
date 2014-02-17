
#include "world/CollisionHandlerDefault.h"
#include "world/ObstacleVisitor_ABC.h"
#include "world/Obstacle.h"
#include "skeleton/Robot.h"
#include "skeleton/Tree.h"
#include "skeleton/Joint.h"


#include <vector>
// Collision handler based on
// ozCollide 1.1.1 library

using namespace std;
using namespace manip_core;

struct ReachableObstaclesContainerCollide : public ObstacleVisitor_ABC
{
	ReachableObstaclesContainerCollide(const World& world, const Tree& tree, const Robot& robot)
		: ObstacleVisitor_ABC()
		, world_(world)
		, tree_ (tree)
		, robot_(robot)
	{
		// NOTHING
	}

	~ReachableObstaclesContainerCollide()
	{
		// NOTHING
	}

	virtual void Visit(const Obstacle& obstacle)
	{
		if(world_.IsReachable(robot_, tree_, obstacle))
			obstacles_.push_back(&obstacle);
	}

	typedef std::vector<const Obstacle*> T_Obstacles;
	typedef T_Obstacles::const_iterator T_ObstaclesCIT;
	typedef T_Obstacles::iterator		T_ObstaclesIT;

	T_Obstacles obstacles_;
	const World& world_;
	const Tree&  tree_ ;
	const Robot& robot_;
};

CollisionHandlerDefault::CollisionHandlerDefault(const World& world)
	: CollisionHandler_ABC()
	, world_(world)
{
	// NOTHING
}

CollisionHandlerDefault::~CollisionHandlerDefault()
{
	// NOTHING
}

void CollisionHandlerDefault::AddObstacle(const Obstacle* obstacle)
{
	// NOTHING
}

void CollisionHandlerDefault::Instantiate()
{
	// NOTHING
}

namespace
{
	typedef std::vector<matrices::Vector3,Eigen::aligned_allocator<matrices::Vector3>> T_Point;
	T_Point TreeToSegments(const Robot& robot, const Tree& tree)
	{
		matrices::Matrix4 mat(robot.ToWorldCoordinates());
		Joint* j = tree.GetRoot();
		T_Point res;
		matrices::Vector3 zero(0,0,0);
		while(j)
		{
			if(j->GetR() != zero)
			{
				res.push_back(matrices::matrix4TimesVect3(mat, j->ComputeS()));
			}
			j = j->pChild_;
		}
		return res;
	}
}

bool CollisionHandlerDefault::IsColliding(const Robot& robot, const Tree& tree)
{
	ReachableObstaclesContainerCollide rc(world_, tree, robot);
	world_.Accept(rc);
	T_Point points = TreeToSegments(robot, tree);
	if(points.size() >= 2)
	{
		for(ReachableObstaclesContainerCollide::T_ObstaclesCIT it = rc.obstacles_.begin(); it!= rc.obstacles_.end(); ++it)
		{
			T_Point::const_iterator ptIt = points.begin();;
			T_Point::const_iterator ptIt2 = points.begin(); ++ptIt2;
			for(;ptIt2 != points.end(); ++ptIt, ++ptIt2)
			{
				if(intersection_.Intersect(*ptIt, *ptIt2, **it))
				{
					return true;
				}
			}
		}
	}
	return false;
}

bool CollisionHandlerDefault::IsSoftColliding(const Robot& robot, const Tree& tree)
{
	ReachableObstaclesContainerCollide rc(world_, tree, robot);
	world_.Accept(rc);
	T_Point points = TreeToSegments(robot, tree);
	if(points.size() >= 2)
	{
		if(points.size() >= 3)
		{
			matrices::Vector3 pt2 = points[points.size()-1];
			points.pop_back();
			matrices::Vector3 pt1 = points[points.size()-1];
			points.push_back(pt1 + (pt2 - pt1) * 0.9);
		}
		for(ReachableObstaclesContainerCollide::T_ObstaclesCIT it = rc.obstacles_.begin(); it!= rc.obstacles_.end(); ++it)
		{
			T_Point::const_iterator ptIt = points.begin();;
			T_Point::const_iterator ptIt2 = points.begin(); ++ptIt2;
			for(;ptIt2 != points.end(); ++ptIt, ++ptIt2)
			{
				if(intersection_.Intersect(*ptIt, *ptIt2, **it))
				{
					return true;
				}
			}
		}
	}
	return false;
}

// TESTS

//using namespace matrices;
//#include "MatrixDefs.h"
//#include "kinematic/TreeFactory.h"
//
//int main(int argc, char *argv[])
//{
//	Vector3 p1(-1,1,1.f);
//	Vector3 p2(1,1,1.f);
//	Vector3 p3(1,-1,1.f);
//	Vector3 p4(-1,-1,1.f);
//	Obstacle* obs = new Obstacle(p1,p2,p3,p4); // first test
//
//	Vector3 p11(-1,1,2.2f);
//	Vector3 p12(1,1,2.2f);
//	Vector3 p13(1,-1,2.2f);
//	Vector3 p14(-1,-1,2.2f);
//
//	Obstacle* obs1 = new Obstacle(p11,p12,p13,p14); // third test
//
//	CollisionHandler cHandler;
//	cHandler.AddObstacle(obs);
//	cHandler.AddObstacle(obs1);
//	cHandler.Instantiate();
//
//	factories::TreeFactory factory;
//	matrices::Matrix4 m(matrices::Matrix4::Identity());
//
//	Tree * testTree = factory.CreateTree(manip_core::enums::robot::QuadrupedLegRight, Vector3(0.0, 0.0, 1.8), 0);
//	
//	//checking collision with first obstacle
//	Robot robot(m, testTree);
//	assert(cHandler.IsColliding(robot, *testTree));
//
//	m = matrices::Rotx4(1.57);
//	Tree * testTree2 = factory.CreateTree(manip_core::enums::robot::QuadrupedLegRight, Vector3(0.0, 0.0, 1.8), 0);
//	Robot robot2(m, testTree2);
//	assert(!cHandler.IsColliding(robot2, *testTree2));
//
//	
//	m = matrices::Roty4(3.14) + matrices::Translate(Vector3(0,0,3.6));
//	Tree * testTree3 = factory.CreateTree(manip_core::enums::robot::QuadrupedLegRight, Vector3(0.0, 0.0, 1.8), 0);
//	Robot robot3(m, testTree3);
//	assert(cHandler.IsColliding(robot3, *testTree3));
//	return 0;
//}
