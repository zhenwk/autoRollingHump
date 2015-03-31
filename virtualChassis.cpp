/*
 *  Adapted from David Rollinson's code (unifiedVC_fast.cpp) on 2012/2/2.
 *
 *  Virtual chassis implementation for use with snake simulator.
 */

/*************************************
 * Documentation from MATLAB version *
 *************************************/

//UNIFIEDVC
//   [ virtualChassis, chassisMags ] = 
//			unifiedVC( robotFrames, axisPerm )
//
//   Uses SVD to find the virtual chassis of the snake for any gait.  
//   The function finds the principal moments of inertia of the robot's 
//   shape and uses them to define the axes of the virtual chassis, with the
//   origin at the center of mass (each transform is treated as a point mass
//   of equal weight).
//
//   This code is intended for use with the Modsnake robots, but it should
//   theoretically be general enough to work on any system with a bunch of
//   rigid bodies described by transformation matrices in a common frame.
//
//   Inputs:
//	   robotFrames - a [4 x 4 x number of modules] array of homogeneous
//	   transform matrices describing the poses of the the centers of 
//	   the modules in the robot.  The matrices need to be in some common
//	   body frame, not relative to each other.
//
//	   axisPerm - a [3 x 3] permutation matrix that allows the user to
//	   re-align the virtual chassis however they want.  By default the 
//   	 1st, 2nd, and 3rd principal moments of inertia are aligned
//	   respectively with the Z, Y, and X axes of the virtual chassis.
//
//   Outputs:
//	   virtualChassis - [4 x 4] transformation matrix that describes the
//	   pose of the virtual chassis body frame, described in the frame of
//	   the input transformation matrices.
//
//	   chassisMags - [3 x 1] vector, diag(S) from SVD, used for reporting
//	   the magnitudes of the principle components of the robot's shape.
//
//   Dave Rollinson
//   July 2011

/* END MATLAB DOCUMENTATION */
								   
#include <math.h>
#include <vector>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Pose3d.h"

using namespace Eigen;

/// Returns a 4x4 (12 element, first four elements are first column) transform
/// matrix.
/// points is
void virtualChassis(double* virtualChassis, std::vector<Point3d> modulePos) {
	/*************
	 * VARIABLES *
	 *************/
	int i, j, k;		// for-loop counters
	double signCheck;   // for checking for sign-flips in the VC
	
	int numModules;		// number of modules

	numModules = modulePos.size();
	
	
	/*********************************************
	 * Create Eigen Matrices for doing the maths *
	 *********************************************/
	
	// Matrix for the virtual chassis transform
	Matrix4d VC = Matrix4d::Identity();
	
	// Form the data matrix for doing SVD
	MatrixXd xyz_pts(numModules,3);
	for (i = 0; i < numModules; i++)
	{
		// Copy the x,y,z coordinates from the input data.
		xyz_pts(i,0) = modulePos[i].x;
		xyz_pts(i,1) = modulePos[i].y;
		xyz_pts(i,2) = modulePos[i].z;
	}
	
	// Get the center of mass of the module positions.
	// Subtract it from the xyz positions
	Vector3d CoM = xyz_pts.colwise().mean();
	for (j = 0; j < 3; j++)
	{ 
		xyz_pts.col(j).array() -= CoM(j);
	}
	
	/*********************************************
	 * Take the SVD of the zero-meaned positions *
	 *********************************************/

	// Take the SVD, use thinSVD since we only need V and S.
	JacobiSVD<MatrixXd> svd(xyz_pts, ComputeThinU | ComputeThinV);
	
	// Get the singular values and V matrix
	Vector3d S = svd.singularValues();
	Matrix3d V = svd.matrixV();
	
	// Make sure the 1st principal moment points toward the
	// head module.
	signCheck = V.col(0).dot( xyz_pts.row(0) );
	if (signCheck < 0)
	{
		V.col(0) = -V.col(0);
	}
		
	// Ensure a right-handed frame, by making the 3rd moment
	// the cross product of the first two.
	V.col(2) = V.col(0).cross(V.col(1));
		
	// 4x4 transform for the virtual chassis
	VC.block<3,3>(0,0) = V;
	VC.block<3,1>(0,3) = CoM;
	
	
	/********************************************
	 * Copy the Eigen Matrices to the MexArrays *
	 ********************************************/
	
	// Matrix for the last virtual chassis
	for (j = 0; j < 4; j++)
	{
		for (k = 0; k < 4; k++)
		{
			virtualChassis[4*j + k] = VC(k,j);
		}
	}
}



