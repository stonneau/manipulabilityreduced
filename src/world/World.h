
#ifndef _CLASS_WORLD
#define _CLASS_WORLD

#include "tools/MatrixDefs.h"

#include <memory>

namespace manip_core
{
struct WorldPImpl;

class Obstacle;
class Tree;
class Robot;
class ObstacleVisitor_ABC;

class World {

public:
	 World();
	~World();

	 void Instantiate(bool /*activateCollision*/);
	 void AddObstacle		(Obstacle* /*obstacle*/);
	 bool IsReachable		(const Robot& /*robot*/, const Tree& /*tree*/, const matrices::Vector3& /*target*/) const;
	 bool IsReachable		(const Robot& /*robot*/, const Tree& /*tree*/, const Obstacle& /*obstacle*/) const;
	 bool GetTarget			(const Robot& /*robot*/, const Tree& /*tree*/, const matrices::Vector3& /*direction*/, matrices::Vector3& /*target*/) const;
	 bool GetClosestTarget	(const Robot& /*robot*/, const Tree& /*tree*/, const matrices::Vector3& /*from*/, matrices::Vector3& /*target*/) const;
	 bool IsColliding		(const Robot& /*robot*/, const Tree& /*tree*/) const;
	 bool IsSoftColliding	(const Robot& /*robot*/, const Tree& /*tree*/) const;
	 void Accept(ObstacleVisitor_ABC& /*visitor*/) const;

private:
	std::auto_ptr<WorldPImpl> pImpl_;
};
} // namespace manip_core
#endif //_CLASS_WORLD