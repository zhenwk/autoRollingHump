#ifndef GAIT_HANDLER_H
#define GAIT_HANDLER_H

#include "gaits/Gait.h"

// To ensure that we get the enum for radius.
#include "gaits/ConicalSidewind.h"

class SnakeSim;

enum GaitName {StraightNone, SidewindLeft, SidewindRight, SlitherForward, SlitherBackward, RotateLeft, RotateRight, ConicalSidewindLeft, ConicalSidewindRight};

struct TransitionCommand {
	// The gait to transition to
	GaitName gait;
	// The length (in seconds) the transition should take
	double transitionLength;
	// For SidewindHelix*, this controls the radius.
	// NOTE: I don't yet know the units
	Radius radius;
};

class GaitHandler {
	public:
		/// Constructor; assumes a 'straight' gait.
		GaitHandler(int _numJoints);

		/// Deconstructor; frees memory
		~GaitHandler();

		/// Reset; resets to a straight snake (optionally, of a given length)
		void reset();
		void reset(int _numJoints);

		/// Runs the gait on the given simulator object for a given amount of
		/// time.
		void runGait(SnakeSim* sim, double delta);

		/// Transitions to a new gait with the given parameters.
		void setTransition(TransitionCommand trans);

	private:
		/// Pointer to the current gait.
		Gait* currentGait;

		/// Number of joints in the snake
		int numJoints;

		/// Internal gait time
		double gaitTime;

		// An internal function to free the memory used by the current gaits.
		void clearGaits();
};

#endif
