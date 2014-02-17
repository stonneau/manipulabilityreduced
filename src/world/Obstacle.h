
#ifndef _CLASS_OBSTACLE
#define _CLASS_OBSTACLE

#include "tools/MatrixDefs.h"

//#include "Rennes1\SpatialDataStructure\Selectors\Triangle3D.h"
#include <vector>
namespace manip_core
{
class Obstacle {

public:
	typedef std::vector<Obstacle>		T_Obstacle;
	typedef T_Obstacle::const_iterator	CIT_Obstacle;

public:
	//make it clockwise from upper left
	 Obstacle(const matrices::Vector3& /*p1*/, const matrices::Vector3& /*p2*/, const matrices::Vector3& /*p3*/, bool donttouch = false);
	 Obstacle(const matrices::Vector3& /*p1*/, const matrices::Vector3& /*p2*/, const matrices::Vector3& /*p3*/, const matrices::Vector3& /*p4*/, bool donttouch = false);
	~Obstacle();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// Minimal distance between the plan described by the obstacle and a point, that is, the distance btw point and its orthonormal projection on the plan
	numeric Distance(const matrices::Vector3& /*point*/, matrices::Vector3& /*getCoordinates*/) const; // get coordinates of the projection
	bool IsAbove(const matrices::Vector3& /*point*/) const; // true if a point is above the obstacle, considering the normal
	//bool  ContainsPlanar(const matrices::Vector3& /*point*/) const; // point in the plan expressed in local coordinates	

	const matrices::Vector3 ProjectUp(const matrices::Vector3& /*point*/) const; // projects a points onto obstacle plan and rises it up a little
	const matrices::Vector3& Center() const { return center_; }
	const matrices::Matrix4& Basis   () const { return basis_; }
	const matrices::Matrix4& BasisInv() const { return basisInverse_; }

	numeric GetD() const { return d_; }
	numeric GetW() const { return w_; }
	numeric GetH() const { return h_; }
	numeric GetA() const { return a_; }
	numeric GetB() const { return b_; }
	numeric GetC() const { return c_; }

	const matrices::Vector3& GetP1() const{return p1_;}
	const matrices::Vector3& GetP2() const{return p2_;}
	const matrices::Vector3& GetP3() const{return p3_;}
	const matrices::Vector3& GetP4() const{return p4_;}
	const matrices::Vector3& GetCenter() {return center_;}

	const matrices::Vector3 u_;
	const matrices::Vector3 v_;
	const matrices::Vector3 n_; // normal vector

	const bool donttouch_;
	const bool onlyThreePoints_;

private:
	void InitObstacle();
	const matrices::Vector3 p1_;
	const matrices::Vector3 p2_;
	const matrices::Vector3 p3_;
	const matrices::Vector3 p4_;
		  matrices::Vector3 center_;

	matrices::Matrix4 basis_; // transformation matrix to world basis (on p4)
	matrices::Matrix4 basisInverse_; // transformation matrix to rectangle basis (on p4)

	numeric a_;
	numeric b_;
	numeric c_;
	numeric d_;
	numeric norm_;
	numeric normsquare_;
	numeric w_;
	numeric h_;
};
}

#endif //_CLASS_OBSTACLE