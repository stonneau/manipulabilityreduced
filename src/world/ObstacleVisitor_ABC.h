
#ifndef _CLASS_OBSTACLEVISITOR_ABC
#define _CLASS_OBSTACLEVISITOR_ABC

namespace manip_core
{
class Obstacle;

class ObstacleVisitor_ABC {

public:
	 ObstacleVisitor_ABC();
	~ObstacleVisitor_ABC();

public:
	virtual void Visit(const Obstacle& /*obstacle*/) = 0;	
private:
};
}
#endif //_CLASS_OBSTACLEVISITOR_ABC