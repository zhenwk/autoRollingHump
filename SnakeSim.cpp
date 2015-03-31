/// \file SnakeSim.cpp
/// \brief Implementation code for snake simulator.
///
/// Required ubuntu packages:
///   build-essential
///   libode1
///   libode-dev
///   libgl1-mesa-dev  (only when enabling visualization)
///   libglu1-mesa-dev  (only when enabling visualization)
///   mesa-common-dev  (only when enabling visualization)
///   libfreeimage  (only when enabling visualization/screenshots)

#include "virtualChassis.cpp"
#include "SnakeSim.h"
//#include "Obstacle.h"
#include <assert.h>
#include <FreeImage.h>
#include <GL/glut.h>
#include <cmath>
#include <iostream>
// Constructor; the "visualize" flag controls whether or not openGL is
// used to visualize the snake.
SnakeSim::SnakeSim(bool visualize, bool screenshots, int joints) {
	// Check whether visualization is desired and compiled in
if (visualize) {
		#ifdef VISUALIZATION
		printf("Enabling visualization\n");
		enableVisualization = true;
		#else
		printf("Not compiled with opengl.  Disabling visualization\n");
		#endif
	} else {
		#ifdef VISUALIZATION
		enableVisualization = false;
		#endif
	}
	// Check whether screenshots are desired and compiled in
	if (screenshots) {
		#ifdef VISUALIZATION
		printf("Enabling screenshots\n");
		enableScreenshots = true;
		#else
		printf("Not compiled with opengl.  Disabling screenshots\n");
		#endif
	} else {
		#ifdef VISUALIZATION
		enableScreenshots = false;
		#endif
	}
	#ifdef VISUALIZATION
	cameraHead = false;
	#endif

	// Initialize ODE
	dInitODE2(0);
	// Created the virtual world
	world = dWorldCreate();
	// Set the gravity and CFM parameters.
	dWorldSetGravity(world,0,0,GRAVITY);
	dWorldSetCFM(world,CFM);
	// Set auto-disabled flag to reduce resource need of idle objects.
	dWorldSetAutoDisableFlag(world,1);
	// Set other ODE parameters (see the ODE documentation for details)
	dWorldSetContactMaxCorrectingVel (world,MAX_CORRECTING_VEL);
	dWorldSetContactSurfaceLayer (world,SURFACE_LAYER);
	// Create the collision space. It contains all the geometries that should
	// be checked for collisions.
	collisionSpace = dHashSpaceCreate (0);
	// Create the contact group for storing the collision contact points
	collisionContactGroup = dJointGroupCreate (0);

	// Create a plane that acts as the ground. It is given by the equation:
	// a*x + b*y + c*z = d, where (a,b,c) is the unit vector normal
	// to the plane. In this simulation, the ground is located at (0,0,1). 
	// That is, the z=0 plane.
	dCreatePlane (collisionSpace,0,0,1,0);

	// Must have at least two joints in the robot!
	assert(joints>=2);
	// Create the robot
	numJoints = joints;
	createRobot();
	double obs_length=2.0*RADIUS;
	createObstacle(-1.9f, 0.20f, 1.0*RADIUS, 2.0*RADIUS);//rotor radius= 10* RADIUS
	//createObstacle(-1.9f, -0.20f, 1.0*RADIUS, 2.0*RADIUS);
	//createObstacle(-1.9f, 0.0f, 1.0*RADIUS, 2.0*RADIUS);
    
    // Initialize sensing related variables
    
    modContactPnts.resize(numJoints+1);
    modContactJoints.resize(numJoints+1);
    modContactJointFb.resize(numJoints+1);


	// Set the simulation time to be zero.
	simulationTime = 0;

	desiredAngleCommands = std::vector<double>(numJoints,0);

	// If we have enabled visualization, initialize the camera setup.
	#ifdef VISUALIZATION
	//createWheel();//chao's testing code
	cameraPosition[0] = -3.540;
	cameraPosition[1] = 0;
	cameraPosition[2] = 1.34;
	cameraOrientation[0] = 0.0f;
	cameraOrientation[1] = -15.0f;
	cameraOrientation[2] = 0.0f;

	// Set video size:
	width = 640;
	height = 480;
	// Uncomment for high quality video rather than fast video
	//width = 1280;
	//height = 960;
	if (enableVisualization) {
		// Create Main Window
		setupVisualization();
		//dsSetViewpoint(cameraPosition, cameraOrientation);
	}
	if (enableScreenshots) {
		// Create screenshot screen, window, pixmap, etc.
		setupScreenshots();
	}

	// If we only have one or the other, set the display here:
	if (enableVisualization && !enableScreenshots) {
		glXMakeCurrent(display,win,glx_context);
	} else if (enableScreenshots && !enableVisualization) {
		glXMakeCurrent(ssDisplay,screenshot_win,glx_screenshot_context);
	}
	#endif
}

// Destructor.
SnakeSim::~SnakeSim() {
	// Destroy the contact group
	dJointGroupDestroy (collisionContactGroup);

	// Destroy the collision space
	dSpaceDestroy (collisionSpace);

	// Destroy the world!!! 
	dWorldDestroy (world);

	#ifdef VISUALIZATION
	// close openGL stuff
	// TODO: there is probably more that should be done here, because there is
	// a memory leak at the end in the openGL libraries.
	if (enableVisualization) {
		glXDestroyContext(display, glx_context);
		XDestroyWindow(display, win);
		XSync(display, 0);
		XCloseDisplay(display);
		display = 0;
		win = 0;
		glx_context = 0;
	}
	if (enableScreenshots) {
		glXDestroyContext(ssDisplay, glx_screenshot_context);
		glXDestroyPixmap(ssDisplay, screenshot_win);
		XSync(ssDisplay, 0);
		XCloseDisplay(ssDisplay);
		ssDisplay = 0;
		screenshot_win = 0;
		glx_screenshot_context = 0;
		free(image);
		free(imageName);
	}
	#endif
}

/// \brief Sets the type of the camera; currently either a snake-head camera
///    or one fixed to the world frame.
void SnakeSim::setCameraType(int cameraPosition) {
	#ifdef VISUALIZATION
	if (cameraPosition == CAMERA_HEAD) {
		cameraHead = true;
	} else {
		cameraHead = false;
	}
	#endif
}

/// \brief Helper function to create robot modules
void SnakeSim::createModule(int index, const dMatrix3 & rotate, dReal position,dReal mass, dReal length) {
	dMass m;

	// Create the body.
	robot.body[index] = dBodyCreate(world);
	// Set it's position
	dBodySetPosition(robot.body[index], 0.0, position, RADIUS+CREATEOFFSET);
	dBodySetRotation(robot.body[index],rotate);

	// Set the mass
	dMassSetCapsuleTotal(&m,mass,3,RADIUS,length);
	dBodySetMass(robot.body[index],&m);

	// Create the geometry and associate it to the body
	robot.geom[index] = dCreateCapsule(collisionSpace,RADIUS,length);
	dGeomSetBody(robot.geom[index],robot.body[index]);
}

/// \brief Helper function to create joints between robot modules
void SnakeSim::createJoint(int jointIndex, int direction, dReal position) {
	// Create the joint
	robot.joints[jointIndex] = dJointCreateHinge(world,0);
	// Attach the two bodies
	dJointAttach(robot.joints[jointIndex],
	             robot.body[jointIndex],
	             robot.body[jointIndex+1]);
	// Even joints up and down, odd joints left and right
	dJointSetHingeAxis(robot.joints[jointIndex], (direction+1)%2, 0, direction);
	// Set the joint position
	dJointSetHingeAnchor(robot.joints[jointIndex], 0,
	                     position, RADIUS+CREATEOFFSET);
	// Set joint parameters: max torque and initial angular velocity.
	// Currently torque limiting is disabled, but could be re-enabled.  It was
	// disabled because initial stiction required high torques to overcome.
	// dJointSetHingeParam(joint, dParamFMax, TORQUE);
	dJointSetHingeParam(robot.joints[jointIndex], dParamVel, 0.0);
}

/// Create the bodies, geometries, and joints for the snake robot; places these
/// at 0,0,0
void SnakeSim::createRobot() {

	// A rotation matrix for for rotation of pi/2 about x-axis.
	dMatrix3 rotatePi2;
	dRFromAxisAndAngle(rotatePi2,1,0,0,M_PI/2);

	// Resize the vectors that describe the robot
	robot.body.resize(numJoints+1);
	robot.geom.resize(numJoints+1);
	robot.joints.resize(numJoints);
    robot.sensors.resize(numJoints+1);
    for (int i = 0; i <= numJoints; i++)
    {
        // Each module has 8 sensors
        robot.sensors[i].resize(8);
    };
    
	// Create the head.  It is a capsule of length LENGTH/2 
	createModule(0, rotatePi2, 0, MASS/2, LENGTH/2);

	// Create the other modules; they have length LENGTH
	for (int i=1; i<=numJoints-1; i++) {
		createModule(i, rotatePi2, (LENGTH+2*RADIUS)*(i), MASS, LENGTH);
	}

	// Create the tail.  Also length LENGTH/2
	createModule(numJoints, rotatePi2, (LENGTH+2*RADIUS)*(numJoints),
	             MASS/2, LENGTH/2);

	// Create the head to first module joint:
	createJoint(0, 0, LENGTH/2+RADIUS);

	// Create the joints between the modules
	for (int i=1; i<=numJoints-2; i++) {
		createJoint(i, i%2, (LENGTH/2+RADIUS) + (LENGTH+2*RADIUS)*(i));
	}

	// Create the last module to tail joint:
	createJoint(numJoints-1, numJoints%2, (LENGTH/2+RADIUS) + (LENGTH+2*RADIUS)*(numJoints-1));

	Point3d pos;
	pos.x = -1.7;
	pos.y = 0;
	pos.z = 0.03;
	//debug change pos.z to 0.2
	setPosition(pos);
}

/// \brief Callback function for ODEs collision checking.
void SnakeSim::nearCallback(void *data, dGeomID o1, dGeomID o2) {
	SnakeSim* me = (SnakeSim*) data;
	int i;

	//-- Get the body's ID
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	//-- If they are connected by a joint, no colision detection is done. Finish
	if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) {
		return;
	}

	//-- Configure the properties for the contact points
	dContact contact[MAX_CONTACTS];
	for (i=0; i<MAX_CONTACTS; i++) {
		contact[i].surface.mode = dContactBounce | dContactSoftCFM;
		contact[i].surface.mu = MU;
		contact[i].surface.mu2 = MU2;
		contact[i].surface.bounce = BOUNCE;
		contact[i].surface.bounce_vel = BOUNCE_VEL;
		contact[i].surface.soft_cfm = SOFT_CFM;
	}

	//-- Get the contact points
	int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom, sizeof(dContact));


	//-- If there are at least one contact point...
	if (numc!=0) {
		//-- For every contact point a joint should be created
		for (i=0; i<numc; i++) {
			//-- Create the joint and add it to the contact group
			dJointID c = dJointCreateContact (me->world,
			                                  me->collisionContactGroup,
			                                  &contact[i]); 

			//-- Set the articulation between the two bodies
			dJointAttach (c,b1,b2);

            // Check if either of the geom is a snake module
            // if yes, store the contact info
            for (int j = 0; j <= me->numJoints; j++)
            {
                if (o1 == me->robot.geom[j])
                {
                    me->modContactPnts[j].push_back(contact[i]);
                    me->modContactJoints[j].push_back(c);
                    //dJointFeedback jointFb;
                    //me->modContactJointFb[j].push_back(jointFb);
                    //int fbCnt = me->modContactJointFb[j].size()-1;
                    //dJointSetFeedback(c, &((me->modContactJointFb)[j][fbCnt]));
                }

                if (o2 == me->robot.geom[j])
                {
                    me->modContactPnts[j].push_back(contact[i]);
                    me->modContactJoints[j].push_back(c);
                    //dJointFeedback jointFb;
                    //me->modContactJointFb[j].push_back(jointFb);
                    //int fbCnt = me->modContactJointFb[j].size()-1;
                    //dJointSetFeedback(c, &((me->modContactJointFb)[j][fbCnt]));      
                }
            }

		}
	}
}

/// \brief Set the output of the sensors according to the contact info
void SnakeSim::sensing()
{
    // Clear the sensor output for the last time step
    for (int i = 0; i <= numJoints; i++)
    {
        robot.sensors[i].assign(8, 0);
    }

    
    // Iterate over all the snake modules
    for (int i = 0; i <= numJoints; i++)
    {
        // Iterate over the contact points of the current snake module
        for (int j = 0; j < modContactPnts[i].size(); j++)
        {
            // Transform representation of the nomal vector from
            // world coordinate to the body coordinate
            dVector3 normalBody;
            dBodyVectorFromWorld(robot.body[i], 
                                 modContactPnts[i][j].geom.normal[0],
                                 modContactPnts[i][j].geom.normal[1],
                                 modContactPnts[i][j].geom.normal[2],
                                 normalBody);
            
            // The angle of the projection of the normal vector onto 
            // the xy plane of the body frame
            float angle = atan2(-normalBody[1], -normalBody[0]);

            // Map the angle to [0 2pi] from [-pi pi]
            angle = angle < 0 ?  angle + 2.0f*M_PI : angle;

            // Determine which sensor should be triggered
            int sensorIdx = floor(angle / (M_PI/4.0f)+0.5);
            sensorIdx = sensorIdx < 8 ? sensorIdx : 0;

            // Modify the sensor output accordingly;
            robot.sensors[i][sensorIdx] = 1;
            
        }
    }

    // Clear the data of last iteration
    for (int i = 0; i <= numJoints; i++)
    {
        modContactPnts[i].clear();
        modContactJoints[i].clear();
        modContactJointFb[i].clear();
    }

    // For debug only, output the sensor data
    printf("Sensor Output: \n");
    for (int i = 0; i < robot.sensors.size(); i++)
    {
        printf("Module %d: ", i);
        for (int j = 0; j < robot.sensors[i].size(); j++)
        {
            printf("%d ", robot.sensors[i][j]);
        }
        printf("\n");
    }
    printf("\n");

}

std::vector<std::vector<int> > SnakeSim::getSensorData()
{
    return robot.sensors;
}
    
/// \brief Set the desired angles as goals for the low-level P controller.
void SnakeSim::setDesiredAngles(std::vector<double> commands) {
	if (commands.size() == numJoints) {
		desiredAngleCommands = commands;
	} else {
		fprintf(stderr,"Wrong number of angles in commanded angle vector!");
	}
}

/// \brief Sets the simulation to run for delta seconds, rounded down to next
///        TIMESTEP.
void SnakeSim::simulateForTime(double delta) {
	
	// Run timesteps here...
	for (double t=0.0;t<delta;t+=TIMESTEP) {
		// Basic P control.  For each joint:
		for (int joint=0;joint<numJoints;joint++) {
			// Get difference between desired and actual joint angles:
		    dReal error = dJointGetHingeAngle(robot.joints[joint]) -
			              desiredAngleCommands[joint];
			dJointAddHingeTorque(robot.joints[joint],-8*error);
		}
		

		

		// Collision detection.  If two or more objects are about to collide,
		// the given function is called.
		dSpaceCollide(collisionSpace,this,&nearCallback);

		// Perform a simulation step, updating all objects.
		dWorldStep(world,TIMESTEP);

        	// Generate the sensor output
        	sensing();

		// Remove the contacting points
		dJointGroupEmpty(collisionContactGroup);

		// Render the robot.
		#ifdef VISUALIZATION
		if (enableVisualization) {
			// If screenshots are also enabled, make sure the
			// right glx context/window are current
			if (enableScreenshots) {
				glXMakeCurrent(display, win, glx_context);
			}
			drawFrame();
			glXSwapBuffers(display,win);
			XSync(display,0);
		}
		if (enableScreenshots && imageSkipNumber++ == FRAMERATE) {
			// If visualization is also enabled, make sure the
			// right glx context/window are current
			if (enableVisualization) {
				glXMakeCurrent(ssDisplay, screenshot_win, glx_screenshot_context);
			}
			takeScreenshot();
			imageSkipNumber = 0;
		}
		#endif

		// Increment simulation time:
		simulationTime += TIMESTEP;
	}
	//print out error of last joint
	std::cout << std::endl;
}

/// \brief Gets the center of mass position of a single module on the robot.
Point3d SnakeSim::getPosition(int module) {
	Point3d res;
	if (module >= robot.body.size()) {
		fprintf(stderr,"In getPosition(int module): Invalid module (%d) selected\n",module);
		return res;
	}
	const dReal * pos = dBodyGetPosition(robot.body[module]);
	res.x = pos[0];
	res.y = pos[1];
	res.z = pos[2];
	return res;
}

/// \brief Gets the center of mass position of the robot.
Point3d SnakeSim::getPosition() {
	// Note: calls getPose, because it just calls same VC code.
	return (getPose()).location;
}

/// \brief Gets a quaternion of the orientation of a single module on the robot.
Orientation3d SnakeSim::getOrientation(int module) {
	Orientation3d res;
	if (module >= robot.body.size()) {
		fprintf(stderr,"In getOrientation(int module): Invalid module (%d) selected\n",module);
		return res;
	}
	const dReal * quat = dBodyGetQuaternion(robot.body[module]);
	res.a = quat[0];res.b = quat[1];res.c = quat[2];res.d = quat[3];
	return res;
}

/// \brief Gets a quaternion of the orientation of the entire robot. Currently
///        does a dumb average/normalize for orientation which is incorrect.
Orientation3d SnakeSim::getOrientation() {
	// Note: calls getPose, because it just calls same VC code.
	return (getPose()).orientation;
}

/// \brief Gets the center of a link position and orientation of the robot.
///        Returns ((0,0,0),(0,0,0,0)) for an invalid module.
///        The orientation returned is a quaternion.
Pose3d SnakeSim::getPose(int module) {
	Pose3d pose;
	pose.location = getPosition(module);
	pose.orientation = getOrientation(module);
	return pose;	
}

/// \brief Gets the center of mass, mean orientation pose of the robot.
/// \todo Use virtual chassis coordinates.
Pose3d SnakeSim::getPose() {
	Pose3d avgPose;

	std::vector<Point3d> moduleCenters(robot.body.size());

	double VCmatrix[16];

	dMatrix3 rot;
	dQuaternion quat;

	// Get each module position and add it to the array
	for (int i = 0; i<robot.body.size(); i++) {
		moduleCenters[i] = getPosition(i);
	}

	virtualChassis(VCmatrix, moduleCenters);

	// Pull out position elements:
	avgPose.location.x = VCmatrix[12];
	avgPose.location.y = VCmatrix[13];
	avgPose.location.z = VCmatrix[14];

	// Build rotation matrix (VCmatrix is column-major, dMatrix3 is row-major)
	rot[0] = (dReal) VCmatrix[0];
	rot[1] = (dReal) VCmatrix[4];
	rot[2] = (dReal) VCmatrix[8];
	rot[4] = (dReal) VCmatrix[1];
	rot[5] = (dReal) VCmatrix[5];
	rot[6] = (dReal) VCmatrix[9];
	rot[8] = (dReal) VCmatrix[2];
	rot[9] = (dReal) VCmatrix[6];
	rot[10] = (dReal) VCmatrix[10];

	// Convert to quaternion
	dRtoQ(rot, quat);

	// Save quaternion
	avgPose.orientation.a = quat[0];
	avgPose.orientation.b = quat[1];
	avgPose.orientation.c = quat[2];
	avgPose.orientation.d = quat[3];

	// Return result
	return avgPose;
}

/// \brief A function to set the world position of the robot, keeping the
///        orientation and internal shape unchanged.  Velocity (angular and
///        linear) is zeroed when this function is called.
void SnakeSim::setPosition(Point3d worldPosition) {
	// Get the position which must be added to the current position.
	Point3d deltaPosition = worldPosition - getPosition();
	
	// Add that change to each module.
	for (int i = 0; i < robot.body.size(); i++) {
		const dReal * oldPos = dBodyGetPosition(robot.body[i]);
		dBodySetPosition(robot.body[i],oldPos[0]+deltaPosition.x,
		                               oldPos[1]+deltaPosition.y,
		                               oldPos[2]+deltaPosition.z);
		dBodySetLinearVel(robot.body[i],0,0,0);
		dBodySetAngularVel(robot.body[i],0,0,0);
	}
}

/// \brief A function to set the world orientation of the robot, keeping the
///        position and internal shape unchanged.  Velocity (angular and linear)
///        is zeroed when this function is called.
/// \todo Finish this function so that it also sets orientation.
void SnakeSim::setOrientation(Orientation3d worldOrientation) {
	fprintf(stderr,"Warning: setOrientation(Orientation3d) function incomplete.  Does not set orientation.\n");
	for (int i = 0; i < robot.body.size(); i++) {
		dBodySetLinearVel(robot.body[i],0,0,0);
		dBodySetAngularVel(robot.body[i],0,0,0);
	}
}

/// \brief A function to set the world pose of the robot.
void SnakeSim::setPose(Pose3d worldPose) {
	setPosition(worldPose.location);
	setOrientation(worldPose.orientation);
}

/// \brief A function to set the internal joint angles of the robot, keeping the
///        overall orientation and position unchanged. 
///        The length of the joint angle vector should match the number of
///        joints currently in the snake.
/// \todo Implement this function
void SnakeSim::setAngles(std::vector<double> jointAngles) {
	fprintf(stderr,"Warning: setAngles(vector<double> jointAngles) function not yet implemented.\n");
}

/// \brief Set the snake's center of mass, mean-orientation pose to the given
///        world frame location, in a particular configuration of joint angles.
///        The length of the joint angle vector should match the number of
///        joints currently in the snake.
void SnakeSim::setState(Pose3d worldPose, std::vector<double> jointAngles) {
	setPose(worldPose);
	setAngles(jointAngles);
}

/// \brief This function destroys all parts of the robot and rebuilds it at the
///        initial position.  It also allows one to change the number of modules
///        contained in the robot.
/// \param numJointsIn The number of joints that the new robot should contain.
void SnakeSim::resetRobot(int numJointsIn) {
	numJoints = numJointsIn;
	
	for (int i = 0; i < robot.body.size(); i++) {
		dBodyDestroy(robot.body[i]);
	}
	for (int i = 0; i < robot.geom.size(); i++) {
		dGeomDestroy(robot.geom[i]);
	}
	for (int i = 0; i < robot.joints.size(); i++) {
		dJointDestroy(robot.joints[i]);
	}
	// Reinitialize the robot:
	createRobot();

	// Set the simulation time to be zero.
	simulationTime = 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Visualization-specific code
////////////////////////////////////////////////////////////////////////////////
#ifdef VISUALIZATION
void setColor(float r, float g, float b, float alpha) {
	GLfloat light_ambient[4],light_diffuse[4],light_specular[4];
	light_ambient[0] = r*0.3f;
	light_ambient[1] = g*0.3f;
	light_ambient[2] = b*0.3f;
	light_ambient[3] = alpha;
	light_diffuse[0] = r*0.7f;
	light_diffuse[1] = g*0.7f;
	light_diffuse[2] = b*0.7f;
	light_diffuse[3] = alpha;
	light_specular[0] = r*0.2f;
	light_specular[1] = g*0.2f;
	light_specular[2] = b*0.2f;
	light_specular[3] = alpha;
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, light_ambient);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, light_diffuse);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, light_specular);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 5.0f);
}

void SnakeSim::setupVisualization() {
	display = XOpenDisplay(NULL);
	if (!display) {
		printf("Can't open X11 display!");
		assert(0);
	}

	// get GL visual
	static int attribListDblBuf[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE,16,GLX_RED_SIZE,4, GLX_GREEN_SIZE,4, GLX_BLUE_SIZE,4, None};
	static int attribList[] = {GLX_RGBA, GLX_DEPTH_SIZE,16,GLX_RED_SIZE,4, GLX_GREEN_SIZE,4, GLX_BLUE_SIZE,4, None};
	visual = glXChooseVisual (display,DefaultScreen(display),attribListDblBuf);
	if (!visual) visual = glXChooseVisual (display,DefaultScreen(display),attribList);
	if (!visual) {
		printf("no good X11 visual found for OpenGL");
		assert(0);
	}
	// create colormap
	colormap = XCreateColormap (display,RootWindow(display,DefaultScreen(display)),
	                            visual->visual,AllocNone);

	// initialize variables
	win = 0;
	glx_context = 0;

	// create the window
	XSetWindowAttributes attributes;
	attributes.background_pixel = BlackPixel(display,DefaultScreen(display));
	
	attributes.colormap = colormap;
	attributes.event_mask = ButtonPressMask | ButtonReleaseMask |
	                        KeyPressMask | KeyReleaseMask |
	                        ButtonMotionMask | PointerMotionHintMask |
	                        StructureNotifyMask;
	win = XCreateWindow (display, RootWindow(display,DefaultScreen(display)), 50, 50,
	                     width, height, 0, visual->depth, InputOutput,
	                     visual->visual,
	                     CWBackPixel | CWColormap | CWEventMask,
	                     &attributes);

	// associate a GLX context with the window
	glx_context = glXCreateContext (display,visual,0,GL_TRUE);
	if (!glx_context) {
		printf("can't make an OpenGL context");
		assert(0);
	}

	// set the window title
	XTextProperty window_name;
	window_name.value = (unsigned char *) "Simulation";
	window_name.encoding = XA_STRING;
	window_name.format = 8;
	window_name.nitems = strlen((char *) window_name.value);
	XSetWMName(display, win, &window_name);

	// participate in the window manager 'delete yourself' protocol
	wm_protocols_atom = XInternAtom (display,"WM_PROTOCOLS",False);
	wm_delete_window_atom = XInternAtom (display,"WM_DELETE_WINDOW",False);
	if (XSetWMProtocols (display,win,&wm_delete_window_atom,1)==0) {
		printf("XSetWMProtocols() call failed");
		assert(0);
	}

	// pop up the window
	XMapWindow (display,win);
	XSync (display,win);
}

void SnakeSim::setupScreenshots() {
	ssDisplay = XOpenDisplay(NULL);
	if (!ssDisplay) {
		printf("Can't open X11 display!");
		assert(0);
	}

	if (!glXQueryExtension(ssDisplay, NULL, NULL)) {
		printf("X server has no GLX extension!");
		assert(0);
	}

	static int attribListDblBuf[] = {GLX_DOUBLEBUFFER, GLX_RGBA, GLX_DEPTH_SIZE,16,GLX_RED_SIZE,1, GLX_GREEN_SIZE,1, GLX_BLUE_SIZE,1, None};
	static int attribList[] = {GLX_RGBA, GLX_DEPTH_SIZE,16,GLX_RED_SIZE,1, GLX_GREEN_SIZE,1, GLX_BLUE_SIZE,1, None};
	ssVisual = glXChooseVisual (ssDisplay,DefaultScreen(ssDisplay),(int *)attribListDblBuf);

	if (!ssVisual) ssVisual = glXChooseVisual (ssDisplay,DefaultScreen(ssDisplay),(int *)attribList);
	if (!ssVisual) {
		printf("no good X11 visual found for OpenGL");
		assert(0);
	}

	screenshot_win = 0;

	glx_screenshot_context = 0;
	image = NULL;

	// Also associate a GLX context for saving screenshots.
	glx_screenshot_context = glXCreateContext (ssDisplay,ssVisual,NULL,0);
	if (!glx_screenshot_context) {
		printf("can't make an OpenGL context for taking screenshots\n");
		assert(0);
	}

	// and the window to print screenshots from
	pmap = XCreatePixmap(ssDisplay, RootWindow(ssDisplay, ssVisual->screen),
	                     width, height, ssVisual->depth);
	screenshot_win = glXCreateGLXPixmap(ssDisplay, ssVisual, pmap);

	// Allocate memory for the pixels to be stored in when readPixels is called
	image = (unsigned char *)malloc(sizeof(unsigned char)*width*height*3);
	if (!image) {
		printf("Unable to allocate image buffer\n");
		assert(0);
	}

	// Allocate memory for the file name, and initialize numbering:
	imageNumber = 0;
	imageSkipNumber = 0;
	// The length must be the length of the image filename + 1 for the null
	// character.
	imageName = (char *)malloc(sizeof(char)*22);
}

// Draw the current frame to a buffer, and save it to a file.
void SnakeSim::takeScreenshot() {
	drawFrame();
	glXSwapBuffers(ssDisplay,screenshot_win);
	XSync(ssDisplay,0);
	glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, image);
	FIBITMAP* bmp = FreeImage_ConvertFromRawBits(image, width, height, 3*width, 24, 0x0000FF, 0xFF0000, 0x00FF00, false);
	sprintf(imageName,"frames/fr%08d.jpg",imageNumber);
	imageNumber++;
	FreeImage_Save(FIF_JPEG, bmp, imageName, 0);
	delete bmp;
}

// Drawing helper function
static void setCamera (float x, float y, float z, float h, float p, float r) {
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();
	glRotatef (90, 0,0,1);
	glRotatef (90, 0,1,0);
	glRotatef (r, 1,0,0);
	glRotatef (p, 0,1,0);
	glRotatef (-h, 0,0,1);
	glTranslatef (-x,-y,-z);
}

// Drawing helper function
static void setCameraMatrix (float x, float y, float z, const dReal* R) {
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();
	glRotatef (180, 1,0,0);

	GLfloat glR[16];
	glR[0] = (float) R[0];glR[1] = (float) R[1];
	glR[2] = (float) R[2];glR[3] = (float) 0.0;

	glR[4] = (float) R[4];glR[5] = (float) R[5];
	glR[6] = (float) R[6];glR[7] = (float) 0.0;

	glR[8] = (float) R[8];glR[9] = (float) R[9];
	glR[10] = (float) R[10];glR[11] = (float) 0.0;

	glR[12] = (float) 0.0;glR[13] = (float) 0.0;
	glR[14] = (float) 0.0;glR[15] = (float) 1.0;
	glMultMatrixf(glR);
	glTranslatef (-x,-y,-z);
}

void SnakeSim::drawSky(float view_xyz[3]) {
	//float view_xyz[3] = {0.0f,0.0f,0.0f};
	glDisable (GL_LIGHTING);
	glDisable (GL_TEXTURE_2D);
	glColor3f (0,0.5,1.0);

	// make sure sky depth is as far back as possible
	glShadeModel (GL_FLAT);
	glEnable (GL_DEPTH_TEST);
	glDepthFunc (GL_LEQUAL);
	glDepthRange (1,1);

	const float ssize = 1000.0f;
	static float offset = 0.0f;

	const float sky_scale = 1.0f/4.0f;
	const float sky_height = 1.0f;
	float x = ssize*sky_scale;
	float z = view_xyz[2] + sky_height;

	glBegin (GL_QUADS);
	glNormal3f (0,0,-1);
	glTexCoord2f (-x+offset,-x+offset);
	glVertex3f (-ssize+view_xyz[0],-ssize+view_xyz[1],z);
	glTexCoord2f (-x+offset,x+offset);
	glVertex3f (-ssize+view_xyz[0],ssize+view_xyz[1],z);
	glTexCoord2f (x+offset,x+offset);
	glVertex3f (ssize+view_xyz[0],ssize+view_xyz[1],z);
	glTexCoord2f (x+offset,-x+offset);
	glVertex3f (ssize+view_xyz[0],-ssize+view_xyz[1],z);
	glEnd();

	offset = offset + 0.002f;
	if (offset > 1) offset -= 1;

	glDepthFunc (GL_LESS);
	glDepthRange (0,1);
}

void SnakeSim::drawGround() {
	glDisable (GL_LIGHTING);
	glShadeModel (GL_FLAT);
	glEnable (GL_DEPTH_TEST);
	glDepthFunc (GL_LESS);
	// glDepthRange (1,1);

	glDisable (GL_TEXTURE_2D);
	glColor3f (GROUND_R,GROUND_G,GROUND_B);

	const float gsize = 100.0f;
	const float offset = 0; // -0.001f; ... polygon offsetting doesn't work well
	const float ground_ofsx = 0.5;
	const float ground_ofsy = 0.5;
	const float ground_scale = 1.0f/1.0f;

	glBegin (GL_QUADS);
	glNormal3f (0,0,1);
	glTexCoord2f (-gsize*ground_scale + ground_ofsx,
				-gsize*ground_scale + ground_ofsy);
	glVertex3f (-gsize,-gsize,offset);
	glTexCoord2f (gsize*ground_scale + ground_ofsx,
				-gsize*ground_scale + ground_ofsy);
	glVertex3f (gsize,-gsize,offset);
	glTexCoord2f (gsize*ground_scale + ground_ofsx,
				gsize*ground_scale + ground_ofsy);
	glVertex3f (gsize,gsize,offset);
	glTexCoord2f (-gsize*ground_scale + ground_ofsx,
				gsize*ground_scale + ground_ofsy);
	glVertex3f (-gsize,gsize,offset);
	glEnd();

	// setup stuff
	glEnable (GL_LIGHTING);
	glDisable (GL_TEXTURE_2D);
	glShadeModel (GL_FLAT);
	glEnable (GL_DEPTH_TEST);
	glDepthFunc (GL_LESS);

	// draw the pyramid grid
	for (int i=-10; i<=10; i++) {
		for (int j=-10; j<=10; j++) {
			glPushMatrix();
			glTranslatef ((float)i,(float)j,(float)0);
			if (i==1 && j==0) setColor (1,0,0,1);
			else if (i==0 && j==1) setColor (0,0,1,1);
			else setColor (1,1,0,1);
			const float k = 0.03f;
			glBegin (GL_TRIANGLE_FAN);
			glNormal3f (0,-1,1);
			glVertex3f (0,0,k);
			glVertex3f (-k,-k,0);
			glVertex3f ( k,-k,0);
			glNormal3f (1,0,1);
			glVertex3f ( k, k,0);
			glNormal3f (0,1,1);
			glVertex3f (-k, k,0);
			glNormal3f (-1,0,1);
			glVertex3f (-k,-k,0);
			glEnd();
			glPopMatrix();
		}
	}
}

void SnakeSim::drawFrame() {
	// setup stuff
	glEnable (GL_LIGHTING);
	glEnable (GL_LIGHT0);
	glDisable (GL_TEXTURE_2D);
	glDisable (GL_TEXTURE_GEN_S);
	glDisable (GL_TEXTURE_GEN_T);
	glShadeModel (GL_FLAT);
	glEnable (GL_DEPTH_TEST);
	glDepthFunc (GL_LESS);
	glEnable (GL_CULL_FACE);
	glCullFace (GL_BACK);
	glFrontFace (GL_CCW);

	// setup viewport
	glViewport(0,0,width,height);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity();
	const float vnear = 0.1f;
	const float vfar = 100.0f;
	const float k = 0.8f;     // view scale, 1 = +/- 45 degrees
	if (width >= height) {
		float k2 = float(height)/float(width);
		glFrustum (-vnear*k,vnear*k,-vnear*k*k2,vnear*k*k2,vnear,vfar);
	}
	else {
		float k2 = float(width)/float(height);
		glFrustum (-vnear*k*k2,vnear*k*k2,-vnear*k,vnear*k,vnear,vfar);
	}

	// setup lights. it makes a difference whether this is done in the
	// GL_PROJECTION matrix mode (lights are scene relative) or the
	// GL_MODELVIEW matrix mode (lights are camera relative, bad!).
	static GLfloat light_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
	static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
	static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	glLightfv (GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv (GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv (GL_LIGHT0, GL_SPECULAR, light_specular);
	glColor3f (1.0, 1.0, 1.0);

	// clear the window
	glClearColor (0.5,0.5,0.5,0);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// go to GL_MODELVIEW matrix mode and set the camera
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();

	// snapshot camera position (in MS Windows it is changed by the GUI thread)
	// TODO: probably don't need this in linux!
	float xyz[3]; // TODO: this is needed for the sky...why?
	float hpr[3];
	memcpy(xyz, cameraPosition,    sizeof(float)*3);
	memcpy(hpr, cameraOrientation, sizeof(float)*3);

	// Set the camera to the head's position and rotation:
	if (cameraHead) {
		const dReal * pos = dBodyGetPosition(robot.body[0]);
		const dReal * rot = dBodyGetRotation(robot.body[0]);
		setCameraMatrix ((float) pos[0], (float) pos[1], (float) pos[2], rot);
	// Just do a fixed camera position:
	} else {
		setCamera (xyz[0],xyz[1],xyz[2],
		           hpr[0],hpr[1],hpr[2]);
	}

	// set the light position (for some reason we have to do this in model view.
	static GLfloat light_position[] = { LIGHTX, LIGHTY, 1.0, 0.0 };
	glLightfv (GL_LIGHT0, GL_POSITION, light_position);

	// draw the background (ground, sky, etc)
	drawSky (xyz);
	drawGround();

	// leave openGL in a known state - flat shaded white, no textures
	glEnable (GL_LIGHTING);
	glDisable (GL_TEXTURE_2D);
	glShadeModel (GL_FLAT);
	glEnable (GL_DEPTH_TEST);
	glDepthFunc (GL_LESS);
	glColor3f (1,1,1);
	//setColor (1,1,1,1);

	// draw the rest of the objects. set drawing state first.
	/*color[0] = 1;
	color[1] = 1;
	color[2] = 1;
	color[3] = 1;
	tnum = 0;*/
	drawRobot();
	drawObstacle();
	//drawWheel();//chao's testing code
	glFlush();
}

void setupDrawingMode(float color[4]) {
	glEnable (GL_LIGHTING);
	glDisable (GL_TEXTURE_2D);
	setColor (color[0],color[1],color[2],color[3]);

	if (color[3] < 1) {
		glEnable (GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	} else {
		glDisable (GL_BLEND);
	}
}

void setTransform (const float pos[3], const float R[12]) {
	GLfloat matrix[16];
	matrix[0]=R[0];
	matrix[1]=R[4];
	matrix[2]=R[8];
	matrix[3]=0;
	matrix[4]=R[1];
	matrix[5]=R[5];
	matrix[6]=R[9];
	matrix[7]=0;
	matrix[8]=R[2];
	matrix[9]=R[6];
	matrix[10]=R[10];
	matrix[11]=0;
	matrix[12]=pos[0];
	matrix[13]=pos[1];
	matrix[14]=pos[2];
	matrix[15]=1;
	glPushMatrix();
	glMultMatrixf(matrix);
}

// Drawing helper function
void drawCapsule (float l, float r) {
	int capped_cylinder_quality = 3;
	int i,j;
	float tmp,nx,ny,nz,start_nx,start_ny,a,ca,sa;
	// number of sides to the cylinder (divisible by 4):
	const int n = capped_cylinder_quality*4;

	l *= 0.5;
	a = float(M_PI*2.0)/float(n);
	sa = (float) sin(a);
	ca = (float) cos(a);

	// draw cylinder body
	ny=1; nz=0;				 // normal vector = (0,ny,nz)
	glBegin (GL_TRIANGLE_STRIP);
	for (i=0; i<=n; i++) {
		glNormal3d (ny,nz,0);
		glVertex3d (ny*r,nz*r,l);
		glNormal3d (ny,nz,0);
		glVertex3d (ny*r,nz*r,-l);
		// rotate ny,nz
		tmp = ca*ny - sa*nz;
		nz = sa*ny + ca*nz;
		ny = tmp;
	}
	glEnd();

	// draw first cylinder cap
	start_nx = 0;
	start_ny = 1;
	for (j=0; j<(n/4); j++) {
		// get start_n2 = rotated start_n
		float start_nx2 =	ca*start_nx + sa*start_ny;
		float start_ny2 = -sa*start_nx + ca*start_ny;
		// get n=start_n and n2=start_n2
		nx = start_nx; ny = start_ny; nz = 0;
		float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
		glBegin (GL_TRIANGLE_STRIP);
		for (i=0; i<=n; i++) {
			glNormal3d (ny2,nz2,nx2);
			glVertex3d (ny2*r,nz2*r,l+nx2*r);
			glNormal3d (ny,nz,nx);
			glVertex3d (ny*r,nz*r,l+nx*r);
			// rotate n,n2
			tmp = ca*ny - sa*nz;
			nz = sa*ny + ca*nz;
			ny = tmp;
			tmp = ca*ny2- sa*nz2;
			nz2 = sa*ny2 + ca*nz2;
			ny2 = tmp;
		}
		glEnd();
		start_nx = start_nx2;
		start_ny = start_ny2;
	}
	// draw second cylinder cap
	start_nx = 0;
	start_ny = 1;
	for (j=0; j<(n/4); j++) {
		// get start_n2 = rotated start_n
		float start_nx2 = ca*start_nx - sa*start_ny;
		float start_ny2 = sa*start_nx + ca*start_ny;
		// get n=start_n and n2=start_n2
		nx = start_nx; ny = start_ny; nz = 0;
		float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
		glBegin (GL_TRIANGLE_STRIP);
		for (i=0; i<=n; i++) {
			glNormal3d (ny,nz,nx);
			glVertex3d (ny*r,nz*r,-l+nx*r);
			glNormal3d (ny2,nz2,nx2);
			glVertex3d (ny2*r,nz2*r,-l+nx2*r);
			// rotate n,n2
			tmp = ca*ny - sa*nz;
			nz = sa*ny + ca*nz;
			ny = tmp;
			tmp = ca*ny2- sa*nz2;
			nz2 = sa*ny2 + ca*nz2;
			ny2 = tmp;
		}
		glEnd();
		start_nx = start_nx2;
		start_ny = start_ny2;
	}
}

// Drawing helper function
void drawGeom(dGeomID g,float color[4]) {
	const dReal *pos;
	const dReal *rot;

	pos = dGeomGetPosition(g);
	rot = dGeomGetRotation(g);
	//printf("rotor position%f: ",pos[1]);
	setupDrawingMode(color);
	glShadeModel(GL_SMOOTH);
	#ifdef dDOUBLE
		int i; float pos2[3], rot2[12];
		for (i=0; i<3; i++) pos2[i]=(float)pos[i];
		for (i=0; i<12; i++) rot2[i]=(float)rot[i];
		setTransform(pos2,rot2);
	#else
		setTransform(pos,rot);
	#endif
	drawCapsule(LENGTH,RADIUS);
	glPopMatrix();

	// FOR SHADOWS:
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glColor3f(GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
	           GROUND_B*SHADOW_INTENSITY);
	glDepthRange(0,0.9999);

	GLfloat matrix[16];
	for (int i=0; i<16; i++) matrix[i] = 0;
	matrix[0]=1;
	matrix[5]=1;
	matrix[8]=-LIGHTX;
	matrix[9]=-LIGHTY;
	matrix[15]=1;
	glPushMatrix();
	glMultMatrixf (matrix);

	#ifdef dDOUBLE
		setTransform(pos2,rot2);
	#else
		setTransform(pos,rot);
	#endif
	drawCapsule(LENGTH,RADIUS);
	glPopMatrix();
	glPopMatrix();
	glDepthRange (0,1);
	// END SHADOW CODE
}



// Drawing helper function
void SnakeSim::drawRobot() {
	int G=0;
	for (int i=0;i<numJoints+1;i++) {
		float color[4] = {1,float(i)/numJoints,0,1};
		drawGeom(robot.geom[i],color);
	}
}

void SnakeSim::createWheel(){
	wheel.body.clear();
	wheel.geom.clear();
	// A rotation matrix for for rotation of pi/2 about x-axis.
	dMatrix3 rotate;
	dRFromAxisAndAngle(rotate,0,1,0,M_PI/2);
	//set up the rotor
	// Create the body.
	wheel.body.push_back(dBodyCreate(world));//rotor of the wheel
	ROTOR_R= 10.0*RADIUS;
	ROTOR_L= 20.0*RADIUS;
	// Set it's position
	dBodySetPosition(wheel.body[0], 0.0, 0.0, 1.0*ROTOR_R);
	dBodySetRotation(wheel.body[0],rotate);
	
	// Set the mass
	dMassSetCapsuleTotal(&rotorM,1,3,ROTOR_R,ROTOR_L);
	dMassAdjust(&rotorM, 1);
	dBodySetMass(wheel.body[0],&rotorM);
	
	// Create the geometry and associate it to the body
	wheel.geom.push_back(dCreateCapsule(collisionSpace,ROTOR_R,ROTOR_L));
	dGeomSetBody(wheel.geom[0],wheel.body[0]);

	double tail_radius=0.2*ROTOR_R, tail_length= 10*ROTOR_L;
	dRFromAxisAndAngle(rotate,1,0,0,M_PI/2);
	//set up the tail
	wheel.body.push_back(dBodyCreate(world));
	// Set it's position
	dBodySetPosition(wheel.body[1], 0.0, tail_radius+ROTOR_R+0.5* tail_length+0.01, ROTOR_R);
	dBodySetRotation(wheel.body[1],rotate);
	// Set the mass
	dMassSetCapsuleTotal(&tailM, .2,3,tail_radius,tail_length);
	dMassAdjust(&tailM, .2);
	dBodySetMass(wheel.body[1],&tailM);
	// Create the geometry and associate it to the body
	wheel.geom.push_back(dCreateCapsule(collisionSpace,tail_radius,tail_length));
	dGeomSetBody(wheel.geom[1],wheel.body[1]);
	
	//create joint between the rotor and tail
	wheel.joints.push_back(dJointCreateHinge(world,0));
	// Attach the two bodies
	dJointAttach(wheel.joints[0],wheel.body[0], wheel.body[1]);
	// Even joints up and down, odd joints left and right
	dJointSetHingeAxis(wheel.joints[0], 1, 0, 0);
	// Set the joint position
	dJointSetHingeAnchor(wheel.joints[0], 0.0, 0.0, ROTOR_R);
	// Set joint parameters: max torque and initial angular velocity.
	// Currently torque limiting is disabled, but could be re-enabled.  It was
	// disabled because initial stiction required high torques to overcome.
	// dJointSetHingeParam(joint, dParamFMax, TORQUE);
	dJointSetHingeParam(wheel.joints[0], dParamVel, 0.0);
}
#endif
////////////////////////////////////////////////////////////////////////////////
/// End visualization-specific code
////////////////////////////////////////////////////////////////////////////////

// Alex's Code {:D}
void SnakeSim::createObstacle(double x, double y, double z, double sideLength)
{
	//add a new body to obstacle
	int obs_num=obstacle.body.size();//the current obstacle number
	obstacle.body.push_back(dBodyCreate(world));
	dBodySetPosition(obstacle.body[obs_num], x, y, z);//put the box on the ground
	dMassSetBox(&newM,100, sideLength, sideLength, sideLength);
	dMassAdjust(&newM, 100);
	dBodySetMass(obstacle.body[obs_num], &newM);
	obstacle.geom.push_back(dCreateBox(collisionSpace, sideLength, sideLength, sideLength));
	dGeomSetBody(obstacle.geom[obs_num], obstacle.body[obs_num]);

	//create joint to fix the obstacle on the environment
	obstacle.joints.push_back(dJointCreateHinge(world,0));
	// Attach the two bodies
	dJointAttach(obstacle.joints[obs_num],
	             obstacle.body[obs_num],0);//attach to the world
	// Even joints up and down, odd joints left and right
	dJointSetHingeAxis(obstacle.joints[obs_num], 0, 0, 1);//direction does not matter
	// Set the joint position
	dJointSetHingeAnchor(obstacle.joints[obs_num], x, y, 0);//hinge on the ground
	// Set joint parameters: max torque and initial angular velocity.
	// Currently torque limiting is disabled, but could be re-enabled.  It was
	// disabled because initial stiction required high torques to overcome.
	// dJointSetHingeParam(joint, dParamFMax, TORQUE);
	dJointSetHingeParam(obstacle.joints[obs_num], dParamVel, 0.0);
	dReal error = dJointGetHingeAngle(obstacle.joints[obs_num]);
	dJointAddHingeTorque(obstacle.joints[obs_num],-100*error);

}
/*void SnakeSim::drawWheel(){
	float color[4] = {1,0,0,1};
	//drawCapsule(wheel.geom[0],color);
	//drawGeom(wheel.geom[1],color);
	const dReal *pos;
	const dReal *rot;

	pos = dGeomGetPosition(wheel.geom[0]);
	rot = dGeomGetRotation(wheel.geom[0]);
	//printf("rotor position%f: ",pos[1]);
	setupDrawingMode(color);
	glShadeModel(GL_SMOOTH);
	#ifdef dDOUBLE
		int i; float pos2[3], rot2[12];
		for (i=0; i<3; i++) pos2[i]=(float)pos[i];
		for (i=0; i<12; i++) rot2[i]=(float)rot[i];
		setTransform(pos2,rot2);
	#else
		setTransform(pos,rot);
	#endif
	drawCapsule(ROTOR_L, ROTOR_R);
	glPopMatrix();

	// FOR SHADOWS:
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glColor3f(GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
	           GROUND_B*SHADOW_INTENSITY);
	glDepthRange(0,0.9999);

	GLfloat matrix[16];
	for (int i=0; i<16; i++) matrix[i] = 0;
	matrix[0]=1;
	matrix[5]=1;
	matrix[8]=-LIGHTX;
	matrix[9]=-LIGHTY;
	matrix[15]=1;
	glPushMatrix();
	glMultMatrixf (matrix);

	#ifdef dDOUBLE
		setTransform(pos2,rot2);
	#else
		setTransform(pos,rot);
	#endif
	drawCapsule(ROTOR_L,ROTOR_R);
	glPopMatrix();
	glPopMatrix();
	glDepthRange (0,1);
	// END SHADOW CODE
	
	//draw tail
	pos = dGeomGetPosition(wheel.geom[1]);
	rot = dGeomGetRotation(wheel.geom[1]);
	//printf("rotor position%f: ",pos[1]);
	setupDrawingMode(color);
	glShadeModel(GL_SMOOTH);
	#ifdef dDOUBLE
		for (i=0; i<3; i++) pos2[i]=(float)pos[i];
		for (i=0; i<12; i++) rot2[i]=(float)rot[i];
		setTransform(pos2,rot2);
	#else
		setTransform(pos,rot);
	#endif
	drawCapsule(10*ROTOR_L, 0.2*ROTOR_R);
	glPopMatrix();

	// FOR SHADOWS:
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glColor3f(GROUND_R*SHADOW_INTENSITY,GROUND_G*SHADOW_INTENSITY,
	           GROUND_B*SHADOW_INTENSITY);
	glDepthRange(0,0.9999);

	for (int i=0; i<16; i++) matrix[i] = 0;
	matrix[0]=1;
	matrix[5]=1;
	matrix[8]=-LIGHTX;
	matrix[9]=-LIGHTY;
	matrix[15]=1;
	glPushMatrix();
	glMultMatrixf (matrix);

	#ifdef dDOUBLE
		setTransform(pos2,rot2);
	#else
		setTransform(pos,rot);
	#endif
	drawCapsule(2*ROTOR_L,0.2*ROTOR_R);
	glPopMatrix();
	glPopMatrix();
	glDepthRange (0,1);
	// END SHADOW CODE
}*/

void SnakeSim::drawObstacle()
{
	for (int Idx = 0; Idx<obstacle.geom.size(); Idx++)
	{	
		dVector3 v3;
	
		const dReal *pos;
		pos = dGeomGetPosition(obstacle.geom[Idx]);
		//for debug
		//printf("block position %f, %f, %f\n",pos[0],pos[1],pos[2]);
		float x = pos[0]-pos[2];
		float y = pos[1]-pos[2];
		float z = pos[2]-pos[2];
	
		dGeomBoxGetLengths(obstacle.geom[Idx], v3);

		float sideLength = pos[2]*2;//v3[0];

		float color[4] = {1, 0, 0, 1};
		setupDrawingMode(color);
		glBegin (GL_QUADS);
		glVertex3d (x, y, z);
		glVertex3d (x + sideLength, y, z);
		glVertex3d (x + sideLength, y + sideLength, z);
		glVertex3d (x, y + sideLength, z);
		glEnd();

		float color1[4] = {0, 1, 0, 1};
		setupDrawingMode(color1);
		glVertex3d (x, y, z);
		glVertex3d (x, y, z + sideLength);
		glVertex3d (x, y + sideLength, z + sideLength);
		glVertex3d (x, y + sideLength, z);
		glEnd();

		float color2[4] = {0, 0, 1, 1};
		setupDrawingMode(color2);
		glBegin (GL_QUADS);
		glVertex3d (x, y, z);
		glVertex3d (x, y, z + sideLength);
		glVertex3d (x + sideLength, y, z + sideLength);
		glVertex3d (x + sideLength, y, z);
		glEnd();

		float color3[4] = {1, 1, 0, 1};
		setupDrawingMode(color3);
		glBegin (GL_QUADS);
		glVertex3d (x + sideLength, y, z);
		glVertex3d (x + sideLength, y, z + sideLength);
		glVertex3d (x + sideLength, y + sideLength, z + sideLength);
		glVertex3d (x + sideLength, y + sideLength, z);
		glEnd();

		float color4[4] = {1, 0, 1, 1};
		setupDrawingMode(color4);
		glBegin (GL_QUADS);
		glVertex3d (x + sideLength, y, z + sideLength);
		glVertex3d (x, y, z + sideLength);
		glVertex3d (x, y + sideLength, z + sideLength);
		glVertex3d (x + sideLength, y + sideLength, z + sideLength);
		glEnd();

		float color5[4] = {0, 1, 1, 1};
		setupDrawingMode(color5);
		glBegin (GL_QUADS);
		glVertex3d (x, y, z + sideLength);
		glVertex3d (x + sideLength, y, z + sideLength);
		glVertex3d (x + sideLength, y + sideLength, z + sideLength);
		glVertex3d (x, y + sideLength, z + sideLength);
		glEnd();
	}

}

