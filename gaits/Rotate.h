/// \file Rotate.h
/// \brief A turn in place gait, using sidewind as a base gait.

#ifndef ROTATE_H
#define ROTATE_H

#include "Gait.h"
#include "Sidewind.h"

/// \brief Rotateing motion.
class Rotate:public Gait{
	private:
	// The direction is simply +/- 1:
	GaitDirection direction;

	public:
	/// Gait constructor sets the number of modules and the direction of motion.
	// TODO: should part or all of this move to the parent class?
	Rotate(int numModulesIn, GaitDirection directionIn) {
		numModules = numModulesIn;
		direction = directionIn;
		if (directionIn != GAIT_DIR_LEFT && directionIn != GAIT_DIR_RIGHT) {
			fprintf(stderr,"Invalid direction selected.  Setting LEFT\n");
			direction = GAIT_DIR_LEFT;
		}
	};

	std::vector<double> getAngles(double phase) {
		// Get the angles from a temporary incarnation of sidewinding:
		std::vector<double> angles =
			Sidewind(numModules,direction).getAngles(0.5*phase);

		// Apply the transformation to turn sidewinding into rotating:
		for (int i=0;i<numModules/2;i++) {
			angles[numModules-1-i] = angles[i];
		}

		return angles;
	};
};

#endif
