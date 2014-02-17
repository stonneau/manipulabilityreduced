
#ifndef _CLASS_ROBOT
#define _CLASS_ROBOT


#include "Exports.h"

#include "skeleton/Joint.h"
#include "skeleton/Tree.h"

#include "tools/MatrixDefs.h"


#include <memory>
#include <vector>

namespace manip_core
{
	struct RobotPImpl;
	class RobotVisitor_ABC;
	class World;

class MANIPCORE_API Robot
{
public:
	typedef std::vector<Tree*>		T_Tree;
	typedef T_Tree::iterator		T_TreeIT;
	typedef T_Tree::const_iterator	T_TreeCIT;
	typedef std::vector<T_Tree>		T_Hierarchy;

public:
	explicit Robot(const matrices::Matrix4& /*transform*/, Tree* /*torsoAndHead*/); //basis switch
	~Robot();

//configure
public:
	void AddTree (Tree* /*tree*/, const matrices::Vector3& /*attach*/, const unsigned int columnJoint = 0);

//helpers
public:
	void  Accept(RobotVisitor_ABC& /*visitor*/) const;
	const matrices::Matrix4& ToWorldCoordinates() const;
	const matrices::Matrix4& ToRobotCoordinates() const;
	Tree* GetTree(Tree::TREE_ID /*id*/) const;
	const Tree* GetTorso() const;
	T_Tree& GetTrees() const;
	const T_Hierarchy& GetHierarchy() const;
	matrices::Vector3 ComputeCom() const;
	matrices::Vector3 ComputeTargetCom() const;
	
//action
public:
	matrices::Vector3 MoveTo(const Robot* /*robot*/); 
	matrices::Vector3 MoveTo(const matrices::Vector3& /*position*/); 
	void Rest(); 
	void Reset(); 
	void Translate(const matrices::Vector3& /*direction*/); 
	void SetPosOri(const matrices::Matrix4& /*transform*/); 

	Robot* Clone() const;

private:
	std::auto_ptr<RobotPImpl> pImpl_;
	int numTrees_;
};
}//namespace manip_core

#endif //_CLASS_ROBOT