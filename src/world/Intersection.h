
#ifndef _CLASS_INTERSECTION
#define _CLASS_INTERSECTION

#include "tools/MatrixDefs.h"

namespace manip_core
{
class Tree;
class Obstacle;
class Robot;

class Intersection {

public:

public:
	 Intersection();
	~Intersection();

public:
	bool Intersect(const Robot& /*robot*/, const Tree& /*tree*/, const Obstacle& /*obstacle*/) const; // Obstacle reachable in theory ?
	bool Intersect(const Robot& /*robot*/, const Tree& /*tree*/, const Obstacle& /*obstacle*/, matrices::Vector3& /*anInterPoint*/) const; // compute intersection
	bool Intersect(const matrices::Vector3& /*from*/, const Obstacle& /*obstacle*/, matrices::Vector3& /*anInterPoint*/) const; // compute intersection
	bool Intersect(const matrices::Vector3& /*A*/, const matrices::Vector3& /*B*/, const Obstacle& /*obstacle*/) const; // compute intersection
	bool Intersect(const matrices::Vector3& /*A*/, const matrices::Vector3& /*B*/, const Obstacle& /*obstacle*/, matrices::Vector3& /*intersection*/) const; // compute intersection
	bool IntersectPlane(const matrices::Vector3& /*A*/, const matrices::Vector3& /*B*/, const Obstacle& /*plane*/, matrices::Vector3& /*result*/) const; // compute PLANE intersection
	bool IntersectPlane(const Tree& /*tree*/, const Obstacle& /*plane*/, matrices::Vector3& /*result*/) const; // compute PLANE intersection
	bool Intersect(const Tree& /*tree*/, const matrices::Vector3& /*target*/) const;
	bool IntersectClosest(const Robot& robot, const Tree& tree, const matrices::Vector3& from, const Obstacle& /*obstacle*/, matrices::Vector3& target) const;
	bool IntersectSegments(const matrices::Vector2& /*A*/, const matrices::Vector2& /*B*/, const matrices::Vector2& /*C*/, const matrices::Vector2& /*D*/, matrices::Vector3& /*intersectionPoint*/) const;

private:
	// do segments AB et CD intersect ?
	numeric determinant(const matrices::Vector2& /*A*/, const matrices::Vector2& /*B*/, const matrices::Vector2& /*C*/) const;
};
} // namespace manip_core
#endif //_CLASS_INTERSECTION