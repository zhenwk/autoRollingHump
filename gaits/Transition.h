/// \file Transition.h
/// \brief A transition between a start and end gait; uses a sinusoidal weighted
///        sum of the angles from each individual gait.

#ifndef TRANSITION_H
#define TRANSITION_H

#include "Gait.h"

/// \brief Smoothly transitions between a starting and ending gait for a given
///        length of time.  The transition begins at the time given by the first
///        getAngles call, and the completion can be checked via the 'complete'
///        function.
class Transition:public Gait{
	private:
	/// Length (in seconds) of the transition.
	double length;
	/// Start and end gaits.
	Gait* start;
	Gait* end;
	/// When getAngles is first called, the time is assumed to be when the
	/// transition starts.  These variables keep track of that information.
	bool hasTransitionBegun;
	double transitionStartTime;

	public:
	/// Constructor sets start and end gaits and other private variables.
	Transition(Gait* startIn, Gait* endIn, double lengthIn) {
		// Initialize this variable for logic in other functions.
		hasTransitionBegun = false;
		// Set start and end gaits.
		start = startIn;
		end = endIn;
		length = lengthIn;
	};

	/// This gait is a transition.
	bool isTransition() {
		return true;
	};

	// Return the starting gait.
	Gait* getStartGait() {
		return start;
	}

	// Return the ending gait.
	Gait* getEndGait() {
		return end;
	}

	/// Returns 1 if the transition is complete, zero otherwise.
	bool complete(double t) {
		// First check if transition has started
		if (!hasTransitionBegun) {
			return 0;
		}
		// Now check if transition has completed.
		if (t >= transitionStartTime + length) {
			return 1;
		} else {
			return 0;
		}
	};

	/// Weighted sum of the angles of the two gaits.  Uses the minimum number
	/// joints returned by the two gaits.
	std::vector<double> getAngles(double time) {
		// If this is the first call of this transition, set the starting time.
		if (!hasTransitionBegun) {
			hasTransitionBegun = true;
			transitionStartTime = time;
		}

		// Get the angles from the constituent gaits.
		std::vector<double> angles1 = start->getAngles(time);
		std::vector<double> angles2 = end->getAngles(time);
		int numAngles = std::min(angles1.size(),angles2.size());

		// Find the weight for the weighted sum.  This is the proportion of the
		// END gait, not the start gait.
		double innerPhase;
		if (length == 0.0) {
			innerPhase = 1.0;
		} else {
			innerPhase = std::min((time-transitionStartTime)/length,1.0);
		}
		double weight = sin(innerPhase*M_PI-M_PI_2)/2.0+0.5;

		// Create a vector to return, and fill it with zeros.
		std::vector<double> res(numAngles,0.0);

		// Now fill it with the weighted sum of the other gaits.
		for (int i = 0; i < numAngles; i++) {
			res[i] = angles1[i]*(1.0-weight) + angles2[i]*(weight);
		}

		return res;
	};
};

#endif
