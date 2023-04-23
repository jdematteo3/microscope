#pragma once
#include <iostream>
#include <vector>

class StageInputs_Rotation
{
	public:
		//returns state vector derivative as first four entries, last two entries are inputs to motors
		//inputs are x, y, theta as for first array, desired theta, desired theta derivative
		static std::vector<double> getRotation(double X[4], double Td, double dTd);

	private:
		//signum function
		static int sign(double x);
};