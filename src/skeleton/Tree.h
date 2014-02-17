
#ifndef _CLASS_TREE
#define _CLASS_TREE

#include "Exports.h"
#include "skeleton/Joint.h"
#include "tools/MatrixDefs.h"

namespace manip_core
{
class MANIPCORE_API Tree {
	friend class Robot;
	friend class Jacobian;
	friend class Sample;

public:
	typedef int TREE_ID;

public:
	explicit Tree(TREE_ID /*id*/);
	~Tree();

private:
	Tree& Tree::operator =(const Tree&);
	Tree(const Tree&);
	
public:	

	void InsertRoot(Joint*);
	void InsertChild(Joint* parent, Joint* child);

	// Accessors based on node numerics
	Joint* GetJoint(int) const;
	Joint* GetRoot() const {return root_;}
	int GetNumJoints() const {return numJoint_;}
	Joint* GetEffector() const;
	const matrices::Vector3& GetPosition() const;
	const numeric GetBoundaryRadius() const { return sphereRadius_; }
	const matrices::Vector3& GetEffectorPosition() const;

	// in contact
	void Compute();
	void Init();
	void ToRest();
	Tree* Clone() const;

public:
	const TREE_ID id_;

private:
	Joint* root_;
	numeric sphereRadius_;
	int numJoint_;
};
}//namespace manip_core

#endif //_CLASS_TREE