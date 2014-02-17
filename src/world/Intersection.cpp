
#include "world/Intersection.h"

#include "world/Obstacle.h"

#include "skeleton/Tree.h"
#include "skeleton/Robot.h"

#include <math.h>

using namespace manip_core;
using namespace matrices;
using namespace Eigen;

Intersection::Intersection()
{
	// NOTHING
}

Intersection::~Intersection()
{
	// NOTHING
}

numeric Intersection::determinant(const Vector2& A, const Vector2& B, const Vector2& C) const
{
	// xA * yB + yA * xC  + xB * yC - xA * yC - yA * xB - yB * xC
	return (A.x() * B.y() + A.y() * C.x()  + B.x() * C.y() - A.x() * C.y() - A.y() * B.x() - B.y() * C.x());
}

void lineEq(const Vector2& A, const Vector2& B, numeric& a, numeric& b)
{
	a = (A.y() - B.y()) / (A.x() - B.x());
	b = A.y() - a * A.x();
}



bool Intersection::IntersectSegments(const Vector2& A, const Vector2& B, const Vector2& C, const Vector2& D, Vector3& intersectionPoint) const
{
	// http://www.ilemaths.net/forum-sujet-30008.html#msg138234
	// improvement : bentley-ottmann Algorithm
	
	// C and D on each side of A B 
	if ((determinant(A, B, C) * determinant(A, B, D) < 0) && (determinant(C, D, A) * determinant(C, D, B) < 0))
	{
		numeric xInt, yInt;
		const float treshold = 0.001f;
		numeric a, b, c, d;
		// we need to handle case where equation of type x = a
		if(abs(A.x() - B.x()) < treshold) // since intersection, AB and CD not // so this can happen just once
		{
			lineEq(C, D, c, d);
			xInt = A.x();
			yInt = c * xInt + d;
		}
		else if(abs(C.x() - D.x()) < treshold)
		{
			// line AB y = ax + b
			lineEq(A, B, a, b);
			xInt = C.x();
			yInt = a * xInt + b;
		}
		else
		{
			lineEq(A, B, a, b);
			lineEq(C, D, c, d);
			xInt = (d - b) / (a - c);
			yInt = c * xInt + d;
		}
		intersectionPoint(0) = xInt;
		intersectionPoint(1) = yInt ;
		intersectionPoint(2) = 0 ;
		return true;
	}
	return false;
}

bool Intersection::Intersect(const Tree& tree, const Vector3& target) const
{
	//TODO : first implemen only checks distance...
	numeric bRad = tree.GetBoundaryRadius(); //TODO reduction factor
	return (target - tree.GetPosition()).norm() < bRad;
}

//TODO : affiner intersection avec joint limits
bool Intersection::Intersect(const Robot& robot, const Tree& tree, const Obstacle& obstacle, Vector3& anInterPoint) const
{
	
	//only considering obstacles in front of us = x positive in robot coordinates
	/*
	if((matrix4TimesVect3(robot.ToRobotCoordinates(), obstacle.Center()).x() < 0))
	{
		return false;
	}*/

	//applying robot transformation for tree
	Vector3 treePositionWorld = matrix4TimesVect3(robot.ToWorldCoordinates(), tree.GetPosition());
	
	// get center of the circle 
	Vector3 center(0,0,0);
	numeric distance = obstacle.Distance(treePositionWorld, center);
	Vector3 cv = treePositionWorld - center;
	Vector3 ov = obstacle.Center() - center;
	
	numeric w, h;
	w = obstacle.GetW(); h = obstacle.GetH();
	// http://homeomath.imingo.net/sphere2.htm
	// use this ? http://stackoverflow.com/a/402010
	numeric bRad = tree.GetBoundaryRadius();
	if (distance < bRad) // obstacle plan in range
	{
		// go into rectangle plan to check what we want
		//float circleRadius = sqrt(bRad * bRad -  obstacle.GetD() * obstacle.GetD());
		numeric circleRadius = sqrt(bRad * bRad -  distance * distance);
		Vector3 centerR = matrix4TimesVect3(obstacle.BasisInv(), center);

		numeric xc, yc;
		xc = centerR(0); yc = centerR(1);
		if((xc >= 0 && xc <= w) && (yc >= 0 && yc <= h)) // is center inside obstacle ? -> take center
		{
			anInterPoint = center;
			return true;
		}
		//
		numeric mRad = circleRadius * 0.8f;
		if((xc + mRad > 0 && xc - mRad < w) && (yc + mRad > 0 && yc - mRad < h)) // on a border -> intersects one side
		{
			Vector2 center2(xc, yc);
			Vector2 centerRectangle((w / 2), (h / 2));
			Vector3 intersectionPoint;
			// Comparing each segment
			//ABCD clockwise from upperLeft
			Vector2 A(0,h);
			Vector2 B(w,h);
			Vector2 C(w,0);
			Vector2 D(0,0);

			bool ok = true;
			if(! IntersectSegments(center2, centerRectangle, A, B, intersectionPoint))
			{
				if(! IntersectSegments(center2, centerRectangle, A, D, intersectionPoint))
				{
					if(! IntersectSegments(center2, centerRectangle, B, C, intersectionPoint))
					{
						ok = IntersectSegments(center2, centerRectangle, C, D, intersectionPoint);
					}
				}
			}
			if(ok)
			{
				// eq de la droite entre les 2 centres
				anInterPoint = matrix4TimesVect3(obstacle.Basis(), intersectionPoint);
				return true;
			}
		}

	}
	return false;
}

bool Intersection::Intersect(const Vector3& from, const Obstacle& obstacle, Vector3& anInterPoint) const
{
	// get center of the circle 
	Vector3 center(0,0,0);
	numeric distance = obstacle.Distance(from, center);
	Vector3 cv = from - center;
	Vector3 ov = obstacle.Center() - center;
	
	numeric w, h;
	w = obstacle.GetW(); h = obstacle.GetH();
	// http://homeomath.imingo.net/sphere2.htm
	// use this ? http://stackoverflow.com/a/402010
	// go into rectangle plan to check what we want
	Vector3 centerR = matrix4TimesVect3(obstacle.BasisInv(), center);

	numeric xc, yc;
	xc = centerR(0); yc = centerR(1);
	if((xc >= 0 && xc <= w) && (yc >= 0 && yc <= h)) // is center inside obstacle ? -> take center
	{
		anInterPoint = center;
		return true;
	}
	//
	Vector2 center2(xc, yc);
	Vector2 centerRectangle((w / 2), (h / 2));
	Vector3 intersectionPoint;
	// Comparing each segment
	//ABCD clockwise from upperLeft
	Vector2 A(0,h);
	Vector2 B(w,h);
	Vector2 C(w,0);
	Vector2 D(0,0);

	bool ok = true;
	if(! IntersectSegments(center2, centerRectangle, A, B, intersectionPoint))
	{
		if(! IntersectSegments(center2, centerRectangle, A, D, intersectionPoint))
		{
			if(! IntersectSegments(center2, centerRectangle, B, C, intersectionPoint))
			{
				ok = IntersectSegments(center2, centerRectangle, C, D, intersectionPoint);
			}
		}
	}
	if(ok)
	{
		// eq de la droite entre les 2 centres
		anInterPoint = matrix4TimesVect3(obstacle.Basis(), intersectionPoint);
		return true;
	}
	return false;
}

bool Intersection::IntersectClosest(const Robot& robot, const Tree& tree, const Vector3& from, const Obstacle& obstacle, Vector3& anInterPoint) const
{
	//applying robot transformation for tree
	Vector3 treePositionWorld = matrix4TimesVect3(robot.ToWorldCoordinates(), tree.GetPosition());
	
	// get center of the circle 
	Vector3 center(0,0,0);
	numeric distance = obstacle.Distance(treePositionWorld, center);
	Vector3 cv = treePositionWorld - center;
	Vector3 ov = obstacle.Center() - center;
	
	numeric w, h;
	w = obstacle.GetW(); h = obstacle.GetH();
	// http://homeomath.imingo.net/sphere2.htm
	// use this ? http://stackoverflow.com/a/402010
	numeric bRad = tree.GetBoundaryRadius();
	if (distance < bRad) // obstacle plan in range
	{
		// go into rectangle plan to check what we want
		//float circleRadius = sqrt(bRad * bRad -  obstacle.GetD() * obstacle.GetD());
		numeric circleRadius = sqrt(bRad * bRad -  distance * distance);
		Vector3 centerR = matrix4TimesVect3(obstacle.BasisInv(), center);

		// take obstacle point that is the closest to from
		Vector3 objective;
		Intersect(from, obstacle, objective);
		Vector3 objectiveR = matrix4TimesVect3(obstacle.BasisInv(), objective);

		numeric xc, yc;
		xc = centerR(0); yc = centerR(1);
		Vector3 objDirection = (objectiveR - centerR);
		if(objDirection.norm() < circleRadius) // is objectiveR inside circle ? -> take objective
		{
			anInterPoint = objective;
			return true;
		}
		//
		numeric mRad = circleRadius * 0.8f;
		if((xc + mRad > 0 && xc - mRad < w) && (yc + mRad > 0 && yc - mRad < h)) // on a border -> intersects one side
		{
			Vector2 center2(xc, yc);
			Vector2 centerRectangle((w / 2), (h / 2));
			Vector3 intersectionPoint;
			// Comparing each segment
			//ABCD clockwise from upperLeft
			Vector2 A(0,h);
			Vector2 B(w,h);
			Vector2 C(w,0);
			Vector2 D(0,0);

			bool ok = true;
			if(! IntersectSegments(center2, centerRectangle, A, B, intersectionPoint))
			{
				if(! IntersectSegments(center2, centerRectangle, A, D, intersectionPoint))
				{
					if(! IntersectSegments(center2, centerRectangle, B, C, intersectionPoint))
					{
						ok = IntersectSegments(center2, centerRectangle, C, D, intersectionPoint);
					}
				}
			}
			if(ok)
			{
				// eq de la droite entre les 2 centres
				objDirection.normalize();
				anInterPoint = matrix4TimesVect3(obstacle.Basis(), centerR + objDirection * mRad );
				return true;
			}
		}

	}
	return false;
}


bool Intersection::Intersect(const Robot& robot, const Tree& tree, const Obstacle& obstacle) const
{
	//only considering obstacles in front of us = x positive in robot coordinates
	/*if((matrix4TimesVect3(robot.ToRobotCoordinates(), obstacle.Center()).x() < 0))
	{
		return false;
	}*/

	//applying robot transformation for tree
	Vector3 treePositionWorld = matrix4TimesVect3(robot.ToWorldCoordinates(), tree.GetPosition());
	
	// get center of the circle 
	Vector3 center(0,0,0);
	Vector3 centerR = matrix4TimesVect3(obstacle.BasisInv(), center);
	numeric distance = obstacle.Distance(treePositionWorld, center);
	Vector3 cv = treePositionWorld - center;
	Vector3 ov = obstacle.Center() - center;
	
	numeric w, h;
	w = obstacle.GetW(); h = obstacle.GetH();
	numeric bRad = tree.GetBoundaryRadius();
	if (distance < bRad) // obstacle plan in range
	{
		numeric circleRadius = sqrt(bRad * bRad -  distance * distance);
		Vector3 centerR = matrix4TimesVect3(obstacle.BasisInv(), center);

		numeric xc, yc;
		xc = centerR(0); yc = centerR(1);
		if((xc >= 0 && xc <= w) && (yc >= 0 && yc <= h)) // is center inside obstacle ? -> take center
		{
			return true;
		}
		numeric mRad = circleRadius * 0.8f;
		if((xc + mRad > 0 && xc - mRad < w) && (yc + mRad > 0 && yc - mRad < h)) // on a border -> intersects one side
		{
			return true;
		}
	}
	return false;
}

namespace
{
	Vector3 ComputeTriangleIntersection (const Vector3& p0, const Vector3& p1, const Vector3& p2, const matrices::Vector3& a, const matrices::Vector3& b)
	{
		matrices::Matrix3 mat (matrices::Matrix3::Zero());
		mat.col(0) = a - b;
		mat.col(1) = p1 - p0;
		mat.col(2) = p2 - p0;
		matrices::Matrix3 inv = mat.inverse(); // this is ok, we know for sure mat is not singular
		return inv * (a - p0);
	}

	bool CheckInsideTriangle(const Vector3& res)
	{
		numeric u = res(1);
		numeric v = res(2);
		return u >= 0 && v >= 0 && u <=1 && v <= 1 && (u + v ) <= 1;
	}
}

//http://en.wikipedia.org/wiki/Line-plane_intersection
bool Intersection::Intersect(const matrices::Vector3& a, const matrices::Vector3& b, const Obstacle& obstacle) const
{
	Vector3 p0 = obstacle.GetP1();
	Vector3 p1 = obstacle.GetP2();
	Vector3 p2 = obstacle.GetP3();
	Vector3 p3 = obstacle.GetP4();

	// We use the parametric approach
	Vector3 directingVector = b - a;

	// make sure we don't have singularities
	// parallel...
	if(directingVector.dot(obstacle.n_) == 0)
	{
		// and included
		if((obstacle.GetP1() - a) .dot(obstacle.n_) == 0)
		{
			return true;
		}
		// and outside
		else
		{
			return false;
		}
	}

	Vector3 res = ComputeTriangleIntersection(p0, p1, p2, a, b);
	if( res(0) >= 0 && res(0) <= 1 ) // t E [0,1] ?
	{
		if(CheckInsideTriangle(res))
		{
			return true;
		}
		else
		{
			return (!obstacle.onlyThreePoints_) && CheckInsideTriangle(ComputeTriangleIntersection(p0, p2, p3, a, b));
		}
	}
	return false;
}


//http://en.wikipedia.org/wiki/Line-plane_intersection
bool Intersection::Intersect(const matrices::Vector3& a, const matrices::Vector3& b, const Obstacle& obstacle, matrices::Vector3& intersectionPoint) const
{
	Vector3 p0 = obstacle.GetP1();
	Vector3 p1 = obstacle.GetP2();
	Vector3 p2 = obstacle.GetP3();
	Vector3 p3 = obstacle.GetP4();

	// We use the parametric approach
	Vector3 directingVector = b - a;

	// make sure we don't have singularities
	// parallel...
	if(directingVector.dot(obstacle.n_) == 0)
	{
		// and included
		if((obstacle.GetP1() - a) .dot(obstacle.n_) == 0)
		{
			intersectionPoint = a + ( b - a ) / 2 ;// midpoint
			return true;
		}
		// and outside
		else
		{
			return false;
		}
	}

	Vector3 res = ComputeTriangleIntersection(p0, p1, p2, a, b);
	if( res(0) >= 0 && res(0) <= 1 ) // t E [0,1] ?
	{
		intersectionPoint = a + ( b - a ) * res(0) ; // intersection point  a + (b-a) * t
		if(CheckInsideTriangle(res))
		{
			return true;
		}
		else
		{
			return CheckInsideTriangle(ComputeTriangleIntersection(p0, p2, p3, a, b));
		}
	}
	return false;
}

//http://en.wikipedia.org/wiki/Line-plane_intersection
bool Intersection::IntersectPlane(const matrices::Vector3& a, const matrices::Vector3& b, const Obstacle& plane, matrices::Vector3& result) const
{
	Vector3 p0 = plane.GetP1();
	Vector3 p1 = plane.GetP2();
	Vector3 p2 = plane.GetP3();
	Vector3 p3 = plane.GetP4();

	// We use the parametric approach
	Vector3 directingVector = b - a;

	// make sure we don't have singularities
	// parallel...
	if(directingVector.dot(plane.n_) == 0)
	{
		// and included
		if((plane.GetP1() - a) .dot(plane.n_) == 0)
		{
			// convention; infinity of points, return a
			result = a;
			return true;
		}
		// and outside
		else
		{
			return false;
		}
	}

	Vector3 res = ComputeTriangleIntersection(p0, p1, p2, a, b);
	if( res(0) >= 0 && res(0) <= 1 ) // t E [0,1] => intersection with plane
	{
		result = a + directingVector * res(0);
		return true;
	}
	return false;
}


//http://en.wikipedia.org/wiki/Line-plane_intersection
// plane in t's coordinate
bool Intersection::IntersectPlane(const Tree& t, const Obstacle& plane, matrices::Vector3& result) const
{
	Vector3 a, b;
	Joint* j = t.GetRoot();
	b = t.GetPosition();
	bool intersect(false);
	while(j)
	{
		a = b;
		b = j->GetS(); // assume tree computed
		if(a!=b)
		{
			intersect = intersect || IntersectPlane(a, b, plane, result); // returned deepest point in the plane
		}
		j = j->pChild_;
	}
	return intersect;
}
