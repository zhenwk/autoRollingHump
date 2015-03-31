/// \file Slither.h
/// \brief The Slither gait

#ifndef SLITHER_H
#define SLITHER_H

#include "Gait.h"

/// \brief Slithering motion.
class Slither:public Gait{
	private:
	// The direction is simply +/- 1:
	int dir;

	public:
	/// Gait constructor sets the number of modules and the direction of motion.
	// TODO: should part or all of this move to the parent class?
	Slither(int numModulesIn, GaitDirection directionIn) {
		numModules = numModulesIn;
		if (directionIn == GAIT_DIR_FORWARD) {
			dir = +1;
		} else if (directionIn == GAIT_DIR_BACKWARD) {
			dir = -1;
		} else {
			fprintf(stderr,"Invalid direction selected.  Setting FORWARD\n");
			dir = +1;
		}
	};

	std::vector<double> getAngles(double phase) {
		// Adjust the speed:
		phase = .6*phase;

		std::vector<double> angles(numModules,0.0);
		// Amount the amplitude changes per module.
		//  (tail amp = 0.55, head amp = 0.15)
		double damplitude_dmodule = 0.4*M_PI_2/(numModules-1);
		double head_amplitude = 0.15*M_PI_2;

		// Slither equation:

		// First, do horizontal modules

		// Spatial frequency of the wave:
		double spat_freq = 1.0/8.0;
		// Amplitude
		double amplitude = head_amplitude;
		for (int i = 0; i < numModules; i+=2) {
			angles[i] = amplitude * sin(2.*M_PI*(dir*phase*2 + i * spat_freq)); 
			amplitude += damplitude_dmodule;
		}

		// Then, vertical modules
		spat_freq = 1.0/14.0;
		amplitude = head_amplitude;
		for (int i = 1; i < numModules; i+=2) {
			angles[i] = amplitude * sin(2.*M_PI*(dir*phase + i * spat_freq)); 
			amplitude += damplitude_dmodule;
		}

		return angles;
	};
};

#endif
