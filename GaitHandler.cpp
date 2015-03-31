#include "GaitHandler.h"

// List of gaits that are actually used...
#include "gaits/Straight.h"
#include "gaits/Sidewind.h"
#include "gaits/Slither.h"
#include "gaits/Rotate.h"
#include "gaits/ConicalSidewind.h"
#include "gaits/Transition.h"

#include "SnakeSim.h"

/// Constructor.  This sets the default gait.
GaitHandler::GaitHandler(int _numJoints) {
	numJoints = _numJoints;
	gaitTime = 0;
	currentGait = (Gait*) new Straight(numJoints);
}

/// Destructor.  Frees memory and does cleanup.
GaitHandler::~GaitHandler() {
	// Free memory from the gaits
	clearGaits();
}

/// Resets to a straight gait with no transition
void GaitHandler::reset() {
	gaitTime = 0;
	clearGaits();
	currentGait = (Gait*) new Straight(numJoints);
}

/// Resets to a straight gait with no transition and the given number of joints
void GaitHandler::reset(int _numJoints) {
	numJoints = _numJoints;
	reset();
}

/// \brief Run current gait for the given length of time. This function is also
///        responsible for ending transitions.
void GaitHandler::runGait(SnakeSim* sim, double delta) {
	for (double t=0; t<delta; t+=TIMESTEP) {
		// First, end any current transition that is complete
		if (currentGait->isTransition() &&
		    ((Transition*) currentGait)->complete(gaitTime)) {
			Gait* tmp = ((Transition*) currentGait)->getEndGait();
			// Delete starting gait and transition to free memory, as they are
			// no longer used.
			delete(((Transition*) currentGait)->getStartGait());
			delete(currentGait);
			currentGait = tmp;
		}
		sim->setDesiredAngles(currentGait->getAngles(gaitTime));
		sim->simulateForTime(TIMESTEP);
		gaitTime += TIMESTEP;
	}
}

/// \brief Sets the robot to begin running a given gait, using specified
///        parameters.
void GaitHandler::setTransition(TransitionCommand trans) {
	Gait* nextGait;
	switch(trans.gait) {
		case (StraightNone):
			nextGait = new Straight(numJoints);
			break;
		case (SidewindLeft):
			nextGait = new Sidewind(numJoints,GAIT_DIR_LEFT);
			break;
		case (SidewindRight):
			nextGait = new Sidewind(numJoints,GAIT_DIR_RIGHT);
			break;
		case (SlitherForward):
			nextGait = new Slither(numJoints,GAIT_DIR_FORWARD);
			break;
		case (SlitherBackward):
			nextGait = new Slither(numJoints,GAIT_DIR_BACKWARD);
			break;
		case (RotateLeft):
			nextGait = new Rotate(numJoints,GAIT_DIR_LEFT);
			break;
		case (RotateRight):
			nextGait = new Rotate(numJoints,GAIT_DIR_RIGHT);
			break;
		case (ConicalSidewindLeft):
			nextGait = new ConicalSidewind(numJoints,GAIT_DIR_LEFT,trans.radius);
			break;
		case (ConicalSidewindRight):
			nextGait = new ConicalSidewind(numJoints,GAIT_DIR_RIGHT,trans.radius);
			break;
	}
	currentGait = new Transition(currentGait,nextGait,trans.transitionLength);
}

////////////////////////////////////////////////////////////////////////////////
/// Private Functions
////////////////////////////////////////////////////////////////////////////////

/// \brief Frees the memory used by the current gaits, and sets the current gait
/// pointer to NULL.
void GaitHandler::clearGaits() {
	// Delete any remaining gait pointers, cycling through all active
	// transitions first.
	while(currentGait->isTransition()) {
		Gait* tmp = ((Transition*) currentGait)->getEndGait();
		delete(((Transition*) currentGait)->getStartGait());
		delete(currentGait);
		currentGait = tmp;
	}
	delete(currentGait);
	currentGait = NULL;
}
