/// \file Gait.h
/// \brief The Base gait class

#ifndef GAIT_H
#define GAIT_H

#include <vector>
// For child gait convenience:
#include <math.h>
#include <stdio.h>

enum GaitDirection {GAIT_DIR_NONE, GAIT_DIR_LEFT, GAIT_DIR_RIGHT, GAIT_DIR_FORWARD, GAIT_DIR_BACKWARD};

class Gait {
	protected:
	// Length of snake.  Accessible to children gaits.
	int numModules;
	// The constructor should only be called by the children.
	Gait() {};

	private:
	// The copy constructor is private -- it should not be
	// called by anything.
	Gait(Gait const&);

	public:
	// This is the function that all of the gaits should override 
	virtual std::vector<double> getAngles(double time) = 0;
	// Is this gait a transition?  By default, no:
	virtual bool isTransition() {return false;};
};

#endif
