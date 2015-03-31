/// \file Sidewind.h
/// \brief The Sidewind gait

#ifndef SIDEWIND_H
#define SIDEWIND_H

#include "Gait.h"

/// \brief Sidewinding motion.
class Sidewind:public Gait{
	private:
	// The direction is simply +/- 1:
	int dir;

	public:
	/// Gait constructor sets the number of modules and the direction of motion.
	// TODO: should part or all of this move to the parent class?
	Sidewind(int numModulesIn, GaitDirection directionIn) {
		numModules = numModulesIn;
		if (directionIn == GAIT_DIR_LEFT) {
			dir = +1;
		} else if (directionIn == GAIT_DIR_RIGHT) {
			dir = -1;
		} else {
			fprintf(stderr,"Invalid direction selected.  Setting LEFT\n");
			dir = +1;
		}
	};

	std::vector<double> getAngles(double phase) {
		std::vector<double> angles(numModules,0.0);
		// Spatial frequency of the wave:
		double spat_freq = 0.08;
		// Temporal phase offset between horizontal and vertical waves.
		double TPO = 3.0/8.0;

		// Sidewind equation:

		// First, do horizontal modules
		double amplitude = 0.38*M_PI_2;
		for (int i = 0; i < numModules; i+=2) {
			angles[i] = amplitude * sin(2.*M_PI*(dir*phase + i * spat_freq)); 
		}

		// Then, vertical modules
		amplitude = 0.49*M_PI_2;
		for (int i = 1; i < numModules; i+=2) {
			angles[i] = amplitude * sin(2.*M_PI*(dir*phase + TPO + i * spat_freq)); 
		}

		return angles;
	};
};

#endif
