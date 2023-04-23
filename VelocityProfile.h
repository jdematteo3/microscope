#pragma once
#include <iostream>
#include <vector>

class VelocityProfile
{
	public:
		// generates velocity profile given a desired change in position, change in time, 
		// and amplification of maximum velocity
		static std::vector<double> getVelocityProfile(double deltaX, int deltat, double amp);
};