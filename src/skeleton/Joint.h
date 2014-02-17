
#ifndef _CLASS_JOINT
#define _CLASS_JOINT

#include "tools/MatrixDefs.h"
#include "Exports.h"

namespace manip_core
{
class MANIPCORE_API Joint
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	friend class Tree;
	friend class Jacobian;

public:
	Joint(const matrices::Vector3& /*attach*/, const matrices::Vector3& /*iRotAxis*/, numeric restAngle=0.);
	Joint(const matrices::Vector3& /*attach*/, const matrices::Vector3& /*iRotAxis*/, numeric /*minTheta*/, const numeric /*maxTheta*/, const  numeric restAngle=0.);

	~Joint();

private:
	Joint& Joint::operator =(const Joint&);
	Joint(const Joint&);
	
public:
	void InitJoint();
	
	const numeric& GetTheta() const { return theta_; }
	const matrices::Vector3& GetS() const { return s_; }
	const matrices::Vector3& GetR() const { return r_; }
	const matrices::Vector3& GetW() const { return w_; }
	const matrices::Vector3& GetRotationAxis() const { return v_; }
	bool IsEffector() const { return pChild_ == 0; }

	const matrices::Vector3& ComputeS(void);
	void ComputeW(void);
	numeric AddToTheta(const numeric delta);
	void SetTheta(numeric newTheta) { theta_ = newTheta; }
	void ToRest();

	Joint* Clone() const;

public:
	Joint* pRealparent_;		// pointer to real parent
	Joint* pChild_;				// pointer to child
	
public:
	const numeric minTheta_;	// lower limit of joint angle
	const numeric maxTheta_;	// upper limit of joint angle
	const numeric restAngle_;	// rest position angle
	const matrices::Vector3 v_;	// rotation axis

private:
	matrices::Vector3 attach_;	// attachment point
	matrices::Vector3 r_;		// relative position vector
	numeric theta_;				// joint angle (radian)
	matrices::Vector3 s_;		// GLobal Position
	matrices::Vector3 w_;		// Global rotation axis
};
} // namespace manip_core
#endif // _CLASS_JOINT
