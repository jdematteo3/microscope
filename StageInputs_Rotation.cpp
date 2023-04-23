#include "StageInputs_Rotation.h"
#include "Constants.h"
#include <iostream>
#include <vector>
using namespace std;

//signum function
int StageInputs_Rotation::sign(double x) {
	if (x > 0)
		return 1;
	else if (x < 0)
		return -1;
	else
		return 0;
}

//returns state vector derivative as first four entries, last two entries are inputs to motors
//inputs are x, y, theta as for first array, desired theta, desired theta derivative
vector<double> StageInputs_Rotation::getRotation(double X[4], double Td, double dTd)
{
	//finding alpha and adding to final state vector, X
	double XO = X[0] - sysparams::Dx;
	double YO = sysparams::Dy - X[1];

	//getting constants used in the paper
	double s = Td - X[2];
	double krot = .001 + 120 * abs(s);
	double K1 = YO - 2 * sysparams::Cy * cos(X[2])
		+ sin(X[2] + X[3]) * sqrt(pow(XO, 2) + pow(YO, 2));
	double K2 = YO - 2 * sysparams::Cx * cos(X[2])
		+ cos(X[2] + X[3]) * sqrt(pow(XO, 2) + pow(YO, 2));
	double H = 2 * sysparams::Icm * cos(X[2]) / sysparams::m
		+ sysparams::Cx * (XO - cos(X[2] + X[3]) * sqrt(pow(XO, 2) + pow(YO, 2)))
		+ sysparams::Cy * (YO - sin(X[2] + X[3]) * sqrt(pow(XO, 2) + pow(YO, 2)));

	double gTh[2] = { cos(X[2]) * K1 / H, cos(X[2]) * K2 / H };

	// alters what is seen in the paper, just a raw computation, no matrices
	//double urot[2] = { gTh[0] * dTd / (pow(gTh[0], 2) + pow(gTh[1], 2)) + krot * sign(s),
	//		gTh[1] * dTd / (pow(gTh[0], 2) + pow(gTh[1], 2)) + krot * sign(s) };

	double urot[2] = { krot * sign(s), krot * sign(s) };

	double dT = cos(X[2]) * (urot[0] * K1 + urot[1] * K2) / H;
	double dx = urot[0] * cos(X[2]) * cos(X[2]) - urot[1] * sin(X[2]) * cos(X[2]) + dT * (-XO * tan(X[2]) + YO);
	double dy = urot[0] * sin(X[2]) * cos(X[2]) + urot[1] * cos(X[2]) * cos(X[2]) + dT * (XO + YO * tan(X[2]));
	double da = -(dy * XO + YO * dx) / (pow(XO, 2) + pow(YO, 2));

	return vector<double> {dx, dy, dT, da, urot[0], urot[1]};
}
