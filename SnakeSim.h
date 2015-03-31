/// \file SnakeSim.h

#if !defined(_sim_snake_h_)
#define _sim_snake_h_

// For the double precision ODE installation; remove if you use the single
// precision install.
#define dDOUBLE
//#define dSINGLE

#include <vector>
#include <ode/ode.h>
#include <GL/glut.h>
// Pose, position, and orientation classes
#include "Pose3d.h"

// Other classes
//#include "Obstacle.h"

// Define constants for the world.
#define GRAVITY -9.81
#define CFM 1e-5
#define MAX_CORRECTING_VEL .115
#define SURFACE_LAYER 0.1
#define MU 7//0.2
#define MU2 MU
#define BOUNCE 0.1
#define BOUNCE_VEL 0.1
#define SOFT_CFM 0.0002
#define MAX_CONTACTS 8

// Simulation constants
#define TIMESTEP .0005
//0.00005
// Define constants for the robot modules
#define LENGTH 0.003
//0.055
//(0.072/3.0)
#define RADIUS 0.026
//(0.036/3.0)
#define MASS 0.163

// How high you must place the module above the ground to overcome initial
// stiction reliably.
#define CREATEOFFSET .04

// Possible camera positions:
#define CAMERA_HEAD 1
#define CAMERA_FIXED 2

////////////////////////////////////////////////////////////////////////////////
// Visualization-specific constants.
////////////////////////////////////////////////////////////////////////////////
#ifdef VISUALIZATION
#include <GL/glx.h>
#include <X11/Xatom.h>
#define LIGHTX (1.0f)
#define LIGHTY (0.4f)
// Ground color:
// (.5, .5, .3 gives a grassy color)
// (.8, .72, .62 gives a light brown - bisque 3)
// (.55, .49, .42 gives a darker brown - bisque 4)
#define GROUND_R (0.55f) 
#define GROUND_G (0.49f)
#define GROUND_B (0.42f)
#define SHADOW_INTENSITY (0.65f)
// Once every 'FRAMERATE' frames, a screenshot is saved:
#define FRAMERATE 1
#endif


////////////////////////////////////////////////////////////////////////////////
// End visualization-specific constants.
////////////////////////////////////////////////////////////////////////////////

struct simRobotSnake {
	std::vector<dJointID> joints;      // Joints

	std::vector<dBodyID> body;         // modules bodies and geometries
	std::vector<dGeomID> geom;         //

    std::vector<std::vector<int> > sensors; // A matrix representing the sensors with each row for each module
};

struct simObstacle {
	std::vector<dJointID> joints;
	std::vector<dBodyID> body;	//obstacle body and geometry
	std::vector<dGeomID> geom;
};

struct simWheel {
	std::vector<dJointID> joints;
	std::vector<dBodyID> body;
	std::vector<dGeomID> geom;
};
class SnakeSim {
public:
	// Constructor; the "visualize" flag controls whether or not openGL is
	// used to visualize the snake.  "joints" represents the number of angles
	// that can be controlled on the snake (aka, number of modules minus one).
	SnakeSim(bool visualize, bool screenshots, int joints);
	// Deconstructor
	~SnakeSim();

	// Reset the snake to its starting configuration -- straight line at 0,0,0
	void resetRobot(int numJointsIn);

	// Functions to get the position and orientation of the robot (or parts of
	// the robot).
	Point3d getPosition();
	Point3d getPosition(int module);
	Orientation3d getOrientation();
	Orientation3d getOrientation(int module);
	Pose3d getPose();
	Pose3d getPose(int module);

	// Functions to set the position, orientation, and internal joint angles
	// of the entire robot.
	void setState(Pose3d worldPose, std::vector<double> jointAngles);
	void setAngles(std::vector<double> jointAngles);
	void setPosition(Point3d worldPosition);
	void setOrientation(Orientation3d worldOrientation);

	// Does nothing unless visualization is compiled in.
	void setCameraType(int);

	// Set the angle commands for the snake.
	void setDesiredAngles(std::vector<double> commands);

	// Set the snake's center of mass, mean-orientation pose to the given
	// world frame location.  Keeps the current internal shape of the robot.
	void setPose(Pose3d worldPose);

	// Sets the simulation to run for delta seconds.  Note that delta is rounded
	// down to the closest multiple of TIMESTEP.
	void simulateForTime(double delta);

	// Create a new obstacle in the envionment!
	void createObstacle(double x, double y, double z, double sideLength);
	void drawObstacle();
	void drawWheel();

    // Simulate the output of sensors according to the contact joints
    void sensing();
    std::vector<std::vector<int> > getSensorData();

	//for debug:from Alex's
	double newSideLength;
	//dGeomID newBox[1];
	//dBodyID newBody[1];
	dMass newM;
	dMass rotorM;
	dMass tailM;
	double ROTOR_R;
	double ROTOR_L;
private:
	// Copy-constructor: not implemented.
	SnakeSim(SnakeSim const&);
	void operator=(SnakeSim const&);

	// The number of joints in the snake robot.
	int numJoints;

	// The current simulation time, in seconds:
	double simulationTime;

	std::vector<double> desiredAngleCommands;

	// Functions to create the robot body, geometries, and joints.  Only called
	// internally during construction of the simulation.
	void createModule(int index, const dMatrix3 & rotate, dReal position,
	                  dReal mass, dReal length);
	void createJoint(int jointIndex, int direction, dReal position);
	void createRobot();
	void createWheel();
	// Internal variable to store the world ID.
	dWorldID world;
	// Internal variable to store the collision space ID.
	dSpaceID collisionSpace;
	// Internal variable to store the collision contact points
	dJointGroupID collisionContactGroup;

    // Store the contact points of each module
    std::vector<std::vector<dContact> > modContactPnts;
    // Store the contact joints of each module
    std::vector<std::vector<dJointID> > modContactJoints;
    // Store the feedback of the contact joints
    std::vector<std::vector<dJointFeedback> > modContactJointFb;

	// Nead to make this a static method to be able to be used as a callback.
	static void nearCallback(void*, dGeomID, dGeomID);

	// The representation of the robot
	simRobotSnake robot;
	
	// Obstacle
	simObstacle obstacle;
	//Obstacle obby;
	simWheel wheel;
	////////////////////////////////////////////////////////////////////////////
	// Visualization-specific member variables and functions.
	////////////////////////////////////////////////////////////////////////////
	#ifdef VISUALIZATION

	// A variable that controls whether or not to visualize the simulation
	bool enableVisualization;
	// A variable that controls whether or not to visualize the simulation
	bool enableScreenshots;
  	// x-y-z camera position
	float cameraPosition[3];
	// Defines and sets whether or not the camera position is on the robot's
	// head.
	bool cameraHead;

	// Camera orientation. The three values are Pan, Tilt and Roll.
	// A (0,0,0) value means the camera is point to the positive direction of
	// of the x axis and it is parallel to the ground.
	float cameraOrientation[3];

	// X11 display info variables.
	Display *display;
	Display *ssDisplay;
	XVisualInfo *visual;
	XVisualInfo *ssVisual;
	Colormap colormap;
	Atom wm_protocols_atom;
	Atom wm_delete_window_atom;

	// window and openGL
	Window win;
	Pixmap pmap;
	GLXPixmap screenshot_win;
	int width, height;
	GLXContext glx_context;
	GLXContext glx_screenshot_context;
	unsigned char *image;
	char *imageName;
	int imageNumber;
	int imageSkipNumber;

	// Setup GL window
	void setupVisualization();
	// Setup GL objects for screenshots
	void setupScreenshots();
	// Draw the current frame to a file.
	void takeScreenshot();
	// Draw a single frame
	void drawFrame();
	// Draw the robot.
	void drawRobot();
	// Draw the ground.
	void drawGround();
	// Draw the sky.
	void drawSky(float view_xyz[3]);
	
	//for debug
	//Alex's Vars
	//dGeomID newBox[1];
	//dBodyID newBody[1];
	//dMass newM;
	//double newSideLength;

	#endif
	////////////////////////////////////////////////////////////////////////////
	// End visualization-specific member variables and functions.
	////////////////////////////////////////////////////////////////////////////
};

#endif // _sim_snake_h_
