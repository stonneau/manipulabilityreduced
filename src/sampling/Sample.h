
#ifndef _CLASS_SAMPLE
#define _CLASS_SAMPLE

#include <vector>
#include "tools/MatrixDefs.h"

#include "Exports.h"

namespace manip_core
{
class Tree;

class Sample {

public:
	typedef std::vector<numeric> T_Angles;

public:
	Sample(const Tree& /*tree*/);
	~Sample();

private:
	Sample& Sample::operator =(const Sample&);
	/*Sample(const Sample&)*/;

public:
	const T_Angles& AngleValues() const {return angles_;}
	const matrices::Vector3& GetPosition() const {return position_;}

public:
	void LoadIntoTree(Tree& /*tree*/) const;
	numeric forceManipulability (const matrices::Vector3& /*direction*/) const ;

private:
	matrices::Matrix3 jacobianProd_;
	matrices::Matrix3 jacobianProdInverse_;
	T_Angles angles_;
	matrices::Vector3 position_;

private:
};
} // namespace manip_core
#endif //_CLASS_SAMPLE