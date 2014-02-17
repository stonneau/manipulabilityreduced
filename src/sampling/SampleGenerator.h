
#ifndef _CLASS_SAMPLEGENERATOR
#define _CLASS_SAMPLEGENERATOR

#include <memory>

namespace manip_core
{
class Sample;
class Robot;
class Tree;
class Obstacle;

class SampleGeneratorVisitor_ABC;
class Filter_ABC;
struct PImpl;

class SampleGenerator {

private:
	 SampleGenerator();
	~SampleGenerator();

	SampleGenerator(const SampleGenerator&); //Pas définie pour ne pas avoir a gérer les copies d'auto_ptr
	SampleGenerator& operator = (const SampleGenerator&); //idem

public:
	void GenerateSamples(Tree& /*tree*/, int nbSamples = 1);
	void GenerateSamples(Robot& /*robot*/, int nbSamples = 1);
	void Request(const Robot& /*robot*/, /*const*/ Tree& /*tree*/, SampleGeneratorVisitor_ABC* /*visitor*/, const Filter_ABC& /*filter*/) const;
	void Request(const Robot& /*robot*/, /*const*/ Tree& /*tree*/, SampleGeneratorVisitor_ABC* /*visitor*/, const Filter_ABC& /*filter*/, const Obstacle&   /*obstacle*/) const;
	void Request(const Robot& /*robot*/, /*const*/ Tree& /*tree*/, SampleGeneratorVisitor_ABC* /*visitor*/) const;
	void Request(const Robot& /*robot*/, /*const*/ Tree& /*tree*/, SampleGeneratorVisitor_ABC* /*visitor*/, const Filter_ABC& /*filter*/, Sample& /*sample*/, const Obstacle& /*obstacle*/) const;

private:
	static SampleGenerator* instance;
	
	std::auto_ptr<PImpl> pImpl_;

public:
	static SampleGenerator* GetInstance()
	{
		if(!instance)
		{
			instance = new SampleGenerator();
		}
		return instance;
	}
};
} // namespace manip_core
#endif //_CLASS_SAMPLEGENERATOR