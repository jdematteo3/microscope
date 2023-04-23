#include "VelocityProfile.h"
#include <iostream>
#include <vector>

// generates velocity profile given a desired change in position, change in time, 
// and amplification of maximum velocity
std::vector<double> VelocityProfile::getVelocityProfile(double deltaX, int deltat, double amp)
{
	//deltaTh = difference in angle in deg, amp = arbitrary constant to increase max speed (must be > 1)
	int t_ramp = (int)(deltat * (1 - (1 / amp))); //time to get to max speed
	double maxSpeed = amp * deltaX / deltat;
	double slope = (amp * deltaX / deltat) / t_ramp;

	std::vector<double> thProf(deltat, 0);

	//each i represents 1 ms
	for (int i = 0; i < t_ramp; i++)
	{
		thProf.at(i) = slope * i;
	}

	for (int i = t_ramp; i < deltat - t_ramp; i++)
	{
		thProf.at(i) = maxSpeed;
	}

	for (int i = deltat - t_ramp; i < deltat; i++)
	{
		thProf.at(i) = maxSpeed - slope * (i - (deltat - t_ramp));
	}

	return thProf;
}