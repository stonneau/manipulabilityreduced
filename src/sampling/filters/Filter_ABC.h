
#ifndef _CLASS_FILTER_ABC
#define _CLASS_FILTER_ABC

namespace manip_core
{
class Sample;
class Obstacle;

class Filter_ABC {

friend class SampleGenerator;

public:
	 Filter_ABC();
	~Filter_ABC();

protected:
	virtual bool ApplyFilter(const Sample& /*sample*/) const  = 0; // this can change
};
} // namespace manip_core

#endif //_CLASS_FILTER_ABC