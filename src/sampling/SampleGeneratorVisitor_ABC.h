
#ifndef _CLASS_SAMPLEGENERATORVISITOR_ABC
#define _CLASS_SAMPLEGENERATORVISITOR_ABC


namespace manip_core
{
class Sample;
class Tree;
class Robot;
class Obstacle;

class SampleGeneratorVisitor_ABC {

public:
	 SampleGeneratorVisitor_ABC();
	~SampleGeneratorVisitor_ABC();

public:
	// TODO tree must be const
	virtual void Visit(const Robot& /*robot*/, /*const*/Tree& /*tree*/, Sample& /*sample*/);	
	virtual void Visit(const Robot& /*robot*/, /*const*/Tree& /*tree*/, Sample& /*sample*/, const Obstacle& /*obstacle*/);	
private:
};
} // namespace manip_core
#endif //_CLASS_SAMPLEGENERATORVISITOR_ABC