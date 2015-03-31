/*Parameters specification:
	MU = 5 (SnakeSim.h)
	TimeLeng = TIMESTEP*20 (example.cpp)
	P-control gain = -8 (SnakeSim.cpp)
*/
#include "SnakeSim.h"
#include "GaitHandler.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>


/// Read joint command from 'desiredAngles_cp.txt'
bool readCommand(std::vector<double>& desiredAngles)
{
    // Input File handler
    std::ifstream fin;
    int numJoints = desiredAngles.size();
    
    fin.open("desiredAngles_cp.txt");
    if (fin.is_open())
    {
        desiredAngles.clear();
        for (int i = 0; i < numJoints; i++)
        {
            double commandForSingleJoint = 0.0f;
            fin >> commandForSingleJoint;
            desiredAngles.push_back(commandForSingleJoint);
        }
        fin.close();
        return true;
    }
    else
    {
        return false;
    }
}

/// Output sensor feedback to 'sensorData.txt'
bool writeSensor(std::vector<std::vector<int> > sensorData, double TIME)
{
    int numModules = sensorData.size();
    int sensorEachMod = sensorData[0].size();
    std::ofstream fout;

	printf("Sensor Output: \n");
	for (int i = 0; i < sensorData.size(); i++)
	{
	    printf("Module %d: \t", i);
	    for (int j = 0; j < sensorData[i].size(); j++)
	    {
		printf("%d ", sensorData[i][j]);
	    }
	    printf("\n");
	}
	printf("\n");

    fout.open("sensorData.txt");
    if (!fout.is_open())
    {
        return false;
    }

    for (int i = 0; i < numModules; i++)
    {
	fout << TIME << " ";
        for (int j = 0; j < sensorEachMod; j++)
        {
            fout << sensorData[i][j] << " ";
        }
        if (i!= numModules-1)
	{
		fout << std::endl;
	}
    }
    fout.close();
    system("cp sensorData.txt sensorData_cp.txt");
    return true;
}

/// The main function
int main ()
{
	// Define variables
	Pose3d pose;
	TransitionCommand trans;
    	int numJoints = 16;
    	std::vector<double> desiredAngles(numJoints);

    	// File handlers
    	std::ofstream fout;
    	std::ifstream fin;

	// Counter for debug
	int cntr = 0;

	GaitHandler gh2(numJoints);
	SnakeSim sim2(1,0,numJoints); 		// U'll be sad if choose screenshot.
	pose.location = sim2.getPosition(0);
	
	int StepNum = 0;
	//Weikun 
	float SimTime=0;
    
	while (1)
    	{
	//Display step number:
	StepNum++;        

	// Read desired angles
        readCommand(desiredAngles);

	/*std::cout << StepNum <<"th Step" << std::endl;
	std::cout <<"Desired Angles: ";
	for (int i = 0; i < numJoints; i++)
	{
		std::cout << desiredAngles[i] << " ";
	}
	std::cout << std::endl;*/
        sim2.setDesiredAngles(desiredAngles);

        // Run the simulation
	double TimeLeng = TIMESTEP*20;
        sim2.simulateForTime(TimeLeng);
	SimTime+=TimeLeng;
	std::cout << "TIME: " << SimTime << std::endl;

        // Output the sensor data
	std::vector<std::vector<int> > sensorDataCp(sim2.getSensorData());
        writeSensor(sensorDataCp, SimTime);
	

   	}
	
	return 0;
}
