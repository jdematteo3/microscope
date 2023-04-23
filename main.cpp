#include <stdlib.h>
#include <stdio.h>
#include <MCSControl.h>
#include <vector>
#include "VelocityProfile.h"
#include "Constants.h"
#include <NIDAQmx.h>
#include "StageInputs_Rotation.h"
#include <time.h>
#ifdef SA_PLATFORM_WINDOWS
#  include <Windows.h>
#else
#  include <time.h>
#endif

void PrintMcsError(SA_STATUS st) 
{
	printf("MCS error %u\n", st);
}

void ExitIfError(SA_STATUS st) 
{
	if (st != SA_OK) {
		PrintMcsError(st);
		exit(1);
	}
}

void setup(SA_INDEX &mcsHandle_d, unsigned int &numOfChannels_d, SA_INDEX &channel1_d, SA_INDEX& channel2_d)
{
	/* open the MCS with USB interface in asyncronous communication mode. We use async mode to allow multiple channels to receive and execute commands simultaneously */
	ExitIfError(SA_OpenSystem(&mcsHandle_d, "usb:ix:0", "async"));

	/*Senses number of channels with actuators attached*/
	ExitIfError(SA_GetNumberOfChannels(mcsHandle_d, &numOfChannels_d));
	printf("Number of Channels: %u\n", numOfChannels_d);

	ExitIfError(SA_SetSensorEnabled_A(mcsHandle_d, SA_SENSOR_ENABLED));
	printf("Sensors are enabled\n");

	ExitIfError(SA_FindReferenceMark_A(mcsHandle_d, channel1_d, SA_BACKWARD_DIRECTION, 0, SA_AUTO_ZERO));
	printf("Channel u1: Reference mark found\n");
	ExitIfError(SA_FindReferenceMark_A(mcsHandle_d, channel2_d, SA_BACKWARD_DIRECTION, 0, SA_AUTO_ZERO));
	printf("Channel u2: Reference mark found\n");

	ExitIfError(SA_Stop_A(mcsHandle_d, channel1_d));
	ExitIfError(SA_Stop_A(mcsHandle_d, channel2_d));
	printf("All stopped\n");

	ExitIfError(SA_SetClosedLoopMoveSpeed_A(mcsHandle_d, channel1_d, 2000000));
	ExitIfError(SA_SetClosedLoopMoveSpeed_A(mcsHandle_d, channel2_d, 2000000));
	printf("Speed control set\n");

	ExitIfError(SA_SetClosedLoopMoveAcceleration_A(mcsHandle_d, channel1_d, 20000));
	ExitIfError(SA_SetClosedLoopMoveAcceleration_A(mcsHandle_d, channel2_d, 20000));
	printf("Acceleration control set\n");

	ExitIfError(SA_SetChannelProperty_A(mcsHandle_d, channel1_d, SA_EPK(SA_GENERAL, SA_LOW_VIBRATION, SA_OPERATION_MODE), SA_ENABLED));
	ExitIfError(SA_SetChannelProperty_A(mcsHandle_d, channel2_d, SA_EPK(SA_GENERAL, SA_LOW_VIBRATION, SA_OPERATION_MODE), SA_ENABLED));
	printf("Low vibration mode on\n");

	ExitIfError(SA_SetBufferedOutput_A(mcsHandle_d, SA_BUFFERED_OUTPUT));
}

void close(SA_INDEX &mcsHandle_d, unsigned int &numOfChannels_d, SA_INDEX& channel1_d,
	int &stop_d, SA_PACKET &packet_d, int chanXcount_d, int chanXStopped_d) 
{
	ExitIfError(SA_CloseSystem(mcsHandle_d));
	system("pause");
}

// Adds initial measured x position of Q to sensor reading.
int getxWQ(SA_INDEX &mcsHandle_d, SA_INDEX &channel1_d, SA_PACKET &packet_d) 
{
	ExitIfError(SA_GetPosition_A(mcsHandle_d, channel1_d));
	ExitIfError(SA_ReceiveNextPacket_A(mcsHandle_d, 1000, &packet_d));
	if ((packet_d.packetType == SA_POSITION_PACKET_TYPE) && (packet_d.channelIndex == channel1_d))
		return sysparams::xWQinit - (double)packet_d.data2; //this is subtraction because the motor is flipped in the world frame
	else
		throw "Data not correct for u1 position";
}

// In this implementation, this is equivalent to getu2Pos.
// Assumption is that y position of P always starts at 0.
int getyWP(SA_INDEX &mcsHandle_d, SA_INDEX &channel2_d, SA_PACKET &packet_d)
{
	ExitIfError(SA_GetPosition_A(mcsHandle_d, channel2_d));
	ExitIfError(SA_ReceiveNextPacket_A(mcsHandle_d, 1000, &packet_d));
	if ((packet_d.packetType == SA_POSITION_PACKET_TYPE) && (packet_d.channelIndex == channel2_d))
		return packet_d.data2;
	else
		throw "Data not correct for u2 position";
}

// Gets measurement of OQ from fiber optic sensor.
// Necessary for calculations of state variables. 
int getOQ(SA_INDEX& mcsHandle_d, SA_INDEX& channel1_d, SA_PACKET& packet_d)
{
	//comes from fiber optic sensor, not yet implemented
	return 0;
}

//signum function
int  sign(int x) {
	if (x > 0)
		return 1;
	else if (x < 0)
		return -1;
	else
		return 0;
}

// Gets current u1 position
int getu1Pos(SA_INDEX& mcsHandle_d, SA_INDEX& channel1_d, SA_PACKET& packet_d)
{
	ExitIfError(SA_GetPosition_A(mcsHandle_d, channel1_d));
	ExitIfError(SA_ReceiveNextPacket_A(mcsHandle_d, 1000, &packet_d));
	if ((packet_d.packetType == SA_POSITION_PACKET_TYPE) && (packet_d.channelIndex == channel1_d))
		return packet_d.data2;
	else
		throw "Data not correct for u1 position";
}

// Gets current u2 position
int getu2Pos(SA_INDEX& mcsHandle_d, SA_INDEX& channel2_d, SA_PACKET& packet_d)
{
	ExitIfError(SA_GetPosition_A(mcsHandle_d, channel2_d));
	ExitIfError(SA_ReceiveNextPacket_A(mcsHandle_d, 1000, &packet_d));
	if ((packet_d.packetType == SA_POSITION_PACKET_TYPE) && (packet_d.channelIndex == channel2_d))
		return packet_d.data2;
	else
		throw "Data not correct for u2 position";
}

int main(int argc, char* argv[])
{ 

	/*Assigns names to each channel 0 through 2*/
	SA_INDEX mcsHandle;
	unsigned int numOfChannels = 2;
	SA_INDEX channel1 = 0;
	SA_INDEX channel2 = 1;
	int stop = 0;
	int chanXStopped = 0;
	int chanXcount = 0;
	SA_PACKET packet;

	setup(mcsHandle, numOfChannels, channel1, channel2);

	// Rotation of stage
	std::vector<double> rotprof = VelocityProfile::getVelocityProfile(coords::theta, times::deltat_theta, amps::thetaAmp); // grabs angular velocity profile (deg/ms)
	time_t startTime = time(NULL);
	while(difftime(time(NULL), startTime)*1000 < times::deltat_theta)
	{
		// Gets current values of necessary variables
		double OQ = (double)getOQ(mcsHandle, channel1, packet);
		double yWP = (double)getyWP(mcsHandle, channel1, packet);
		double xWQ = (double)getxWQ(mcsHandle, channel1, packet);

		// Calculating state variables
		double r = sqrt(pow(xWQ - sysparams::Dx, 2) + pow(sysparams::Dy - yWP, 2)) / 2;
		double xBP = sqrt(4 * pow(r, 2) + pow(OQ, 2));
		double alpha = atan(OQ/xBP);
		double thetacur = atan((sysparams::Dy - yWP) / (xWQ - sysparams::Dx)) - alpha;
		double xWO = sysparams::Dx + xBP * cos(thetacur);
		double yWO = sysparams::Dy + xBP * sin(thetacur);

		std::cout << thetacur + '\n';
		
		// Passing state variables to funciton to get motor velocity inputs
		double X[] = { xWO, yWO, thetacur, alpha };
		std::vector<double> stateAndSpeeds = StageInputs_Rotation::getRotation(X, coords::theta, rotprof[(int)difftime(time(NULL), startTime) * 1000]);

		// Moves to new position given velocity inputs
		//ExitIfError(SA_GotoPositionAbsolute_A(mcsHandle, channel1, sign(stateAndSpeeds[4])*1000000000, 1));
		//ExitIfError(SA_SetClosedLoopMoveSpeed_A(mcsHandle, channel1, (int)abs(stateAndSpeeds[4])));
		//ExitIfError(SA_GotoPositionAbsolute_A(mcsHandle, channel2, sign(stateAndSpeeds[5])*1000000000, 1));
		//ExitIfError(SA_SetClosedLoopMoveSpeed_A(mcsHandle, channel2, (int)abs(stateAndSpeeds[5])));

		ExitIfError(SA_FlushOutput_A(mcsHandle));
	}

	close(mcsHandle, numOfChannels, channel1, stop, packet, chanXcount, chanXStopped);
	return 0;
	
}