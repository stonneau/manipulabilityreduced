
#include "skeleton/Joint.h"
#include "tools/Pi.h"
#include "tools/MatrixDefs.h"

#include <math.h>

using namespace matrices;
using namespace manip_core;

Joint::Joint(const Vector3& attach, const Vector3& iRotAxis, numeric minTheta, numeric maxTheta, numeric restAngle)
	: r_			(0, 0, 0)
	, attach_		(attach) 
	, v_			(iRotAxis) // Rotation axis when joints at zero angles
	, theta_		(0)
	, pRealparent_	(0)
	, restAngle_	(restAngle)
	, minTheta_		(minTheta)
	, maxTheta_		(maxTheta)
	, pChild_		(0)
{
	// NOTHING
}

Joint::Joint(const Vector3& attach, const Vector3& iRotAxis, numeric restAngle)
	: r_			(0, 0, 0)
	, attach_		(attach) 
	, v_			(iRotAxis) // Rotation axis when joints at zero angles
	, theta_		(0)
	, pRealparent_	(0)
	, restAngle_	(restAngle)
	, minTheta_		(-Pi)
	, maxTheta_		(Pi)
	, pChild_		(0)
{
	// NOTHING
}

Joint::~Joint()
{
	//NOTHING
}

numeric Joint::AddToTheta(const numeric delta)
{ 
	numeric newTheta = theta_ + delta;
	while (newTheta > 360 * DegreesToRadians )
	{
		newTheta -= 360 * DegreesToRadians;
	}
	while (newTheta < -360 * DegreesToRadians )
	{
		newTheta += 360 * DegreesToRadians;
	}
	if(newTheta < 0 * DegreesToRadians)
	{
		newTheta = (360 * DegreesToRadians + newTheta);
	}
	theta_ = newTheta;
	return 0.f;
}

// Compute the global position of a single Joint
const matrices::Vector3& Joint::ComputeS(void)
{
	Joint* y = this->pRealparent_;
	Joint* w = this;
	s_ = r_;							// Initialize to local (relative) position
	while (y) {
		Rotate(y->v_, s_, y->theta_);
		y = y->pRealparent_;
		w = w->pRealparent_;
		s_ += w->r_;
	}
	return s_;
}

// Compute the global rotation axis of a single Joint
void Joint::ComputeW(void)
{
	Joint* y = this->pRealparent_;
	w_ = v_;							// Initialize to local rotation axis
	while (y) {
		Rotate(y->v_, w_, y->theta_);
		y = y->pRealparent_;
	}
}

void Joint::InitJoint()
{
	theta_ = restAngle_;
}

void Joint::ToRest()
{
	theta_ = restAngle_;
}

Joint* Joint::Clone() const
{
	Joint* res = new Joint(attach_, v_, minTheta_, maxTheta_, restAngle_);
	res->theta_ = theta_;
	return res;
}

