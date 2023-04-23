#pragma once
#include <iostream>

namespace times 
{
	constexpr int deltat_theta = 6000; // time to move to desired theta
	constexpr int deltat_slide = 6000; // time to move to sliding line
	constexpr int deltat_translation = 6000; // time to move along sliding line to final position
}

namespace coords 
{
	constexpr double theta = 10; // desired theta
	constexpr double x = 10; // desired x
	constexpr double y = 10; // desired y
}

namespace amps 
{
	constexpr double thetaAmp = 6.0 / 5; // amplification of max angular speed while rotating
	constexpr double posAmp = 6.0 / 5; // amplification of max position speed while sliding/translating
}

namespace sysparams 
{
	constexpr double Dx = -47500000; // horizontal distance from P to world origin, nm
	constexpr double Dy = 47500000; // vertical distance from Q to world origin, nm
	constexpr double Cx = 39210000; // x coordinate of COM in base frame, nm
	constexpr double Cy = 39210000; // y coordinate of COM in base frame, nm
	constexpr double Icm = 1; // moment of inertia of stage
	constexpr double m = 4; // mass of stage

	constexpr double xWQinit = 20; // initial x coordinate of Q in world frame
}