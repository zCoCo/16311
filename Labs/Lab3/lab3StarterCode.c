/**********************************************
 * Lab 3 : Starter code
 * Written by Kaushik Viswanathan,
 * Modified by Allan Wang (Jan 2017)

 * Feel free to modify any part of these codes.
 **********************************************/

//Global variables - you will need to change some of these
//Robot's positions
float robot_X = 0.0, robot_Y = 0.0, robot_TH = 0.0;

int velocityUpdateInterval = 5;
int PIDUpdateInterval = 2;

//Change these during demo
int inputStraight[2] = {0, 0}; // in mm
int inputTurn[2] = {0, 0}; // in degrees, negative means clockwise rotation
int motorPower = 50;

/*****************************************
 * Complete this function so that it
 * continuously updates the robot's position
 *****************************************/
task dead_reckoning()
{

	while(1)
	{
		//
		// Fill in code for numerical integration / position estimation here
		//

		/*Code that plots the robot's current position and also prints it out as text*/
		nxtSetPixel(50 + (int)(100.0 * robot_X), 32 + (int)(100.0 * robot_Y));
		nxtDisplayTextLine(0, "X: %f", robot_X);
		nxtDisplayTextLine(1, "Y: %f", robot_Y);
		nxtDisplayTextLine(2, "t: %f", robot_TH);

		wait1Msec(velocityUpdateInterval);
	}
}

/*****************************************
 * Main function - Needs changing
 *****************************************/
task main()
{
	/* Reset encoders and turn on PID control */
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;
	nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
	nPidUpdateInterval = PIDUpdateInterval;

	int goalStraight = 0;
	int goalTurn = 0;
	float start_X = 0;
	float start_Y = 0;
	float distTravelled = 0;

	draw_grid();
	startTask(dead_reckoning);

	for(int i = 0; i < 2; i++)
	{
		goalStraight = inputStraight[i];
		goalTurn = inputTurn[i];

		//
		// Write your own codes for turning
		//

		start_X = robot_X;
		start_Y = robot_Y;

		/* Example codes for moving in a striaght line a certain distance,
		 * you need to change this for MUCH better performance */
		motor[motorA] = motorPower;
		motor[motorB] = motorPower;
		distTravelled = sqrt(pow(robot_X - start_X, 2) + pow(robot_Y - start_Y, 2));
		while (!equal(distTravelled, goalStraight)) {
			distTravelled = sqrt(pow(robot_X - start_X, 2) + pow(robot_Y - start_Y, 2));
		}


		wait1Msec(100 * 5);
	}
	motor[motorA] = 0;
	motor[motorB] = 0;
	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
