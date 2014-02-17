
#include "FilterDistance.h"

#include "skeleton/Tree.h"
#include "Sampling/Sample.h"
#include "world/Obstacle.h"

using namespace matrices;
using namespace Eigen;
using namespace manip_core;

FilterDistance::FilterDistance(numeric treshold, const Tree& tree, const Vector3& target)
	: Filter_ABC()
	, treshold_(treshold)
	, target_(target)
	, treePos_(tree.GetPosition())
{
	// NOTHING
}

FilterDistance::~FilterDistance()
{
	// NOTHING
}

bool FilterDistance::ApplyFilter(const Sample& sample) const
{
	// check that sample position is inside obstacle radius and not too far from its plan
	return ((sample.GetPosition() + treePos_) - target_).norm() < treshold_;
}

