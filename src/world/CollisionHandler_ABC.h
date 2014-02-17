
#ifndef _CLASS_COLLISION_HANDLER_ABC
#define _CLASS_COLLISION_HANDLER_ABC

namespace manip_core
{
class Tree;
class Robot;
class Obstacle;

class CollisionHandler_ABC
{

public:
	 explicit CollisionHandler_ABC();
			 ~CollisionHandler_ABC();

//helper
public:

//request
public:
	virtual void AddObstacle(const Obstacle* /*obstacle*/) = 0;
	virtual bool IsColliding(const Robot& /*robot*/, const Tree& /*tree*/) = 0;
	virtual bool IsSoftColliding(const Robot& robot, const Tree& tree) {return IsColliding(robot, tree);}
	virtual void Instantiate() = 0;
	
};
} // namespace manoip_core
#endif //_CLASS_COLLISION_HANDLER_ABC