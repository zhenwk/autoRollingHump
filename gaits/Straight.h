/// \file Straight.h
/// \brief The Straight "gait"

#ifndef STRAIGHT_H
#define STRAIGHT_H

#include "Gait.h"

/// \brief This gait just creates a straight line.
class Straight:public Gait{
	public:
	/// Gait constructor sets the number of modules.
	// TODO: should this move to the parent class?
	Straight(int numModulesIn) {
		numModules = numModulesIn;
	};

	std::vector<double> getAngles(double time) {
		std::vector<double> angles(numModules,0.0);
		return angles;
	};
};

#endif
