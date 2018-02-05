#pragma config(Sensor, S3,     lightSensor,         sensorLightActive)
/**********************************************
 * Lab 3 : Starter code
 * Written by Kaushik Viswanathan,
 * Modified by Allan Wang (Jan 2017)

 * Feel free to modify any part of these codes.
 * Team 15
 **********************************************/

//Global variables - you will need to change some of these
//Robot's positions

#include "RobotCIncludes.h"

float robot_X = 0.0, robot_Y = 0.0, robot_TH = 0.0;

int velocityUpdateInterval = 5;
int PIDUpdateInterval = 2;

//Change these during demo
int inputStraight[2] = {0, 0}; // in mm
int inputTurn[2] = {0, 0}; // in degrees, negative means clockwise rotation
//int motorPower = 50;

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
		nxtDisplayTextLine(2, "S: %f", SensorValue[lightSensor]);

		wait1Msec(velocityUpdateInterval);
	}
}

/*****************************************
 * Function that draws a grid on the LCD
 * for easier readout of whatever is plot
 *****************************************/
void draw_grid()
{
	for(int i = 0; i < 65; i++)
	{
		nxtSetPixel(50, i);
		int grid5 = (i - 32) % 5;
		int grid10 = (i - 32) % 10;
		if(!grid5 && grid10)
		{
			for(int j = -2; j < 3; j++)
			{
				nxtSetPixel(50 + j, i);
			}
		}
		else if(!grid10)
		{
			for(int j = -4; j < 5; j++)
			{
				nxtSetPixel(50 + j, i);
			}
		}
	}
	for(int i = 0; i < 101; i++)
	{
		nxtSetPixel(i, 32);
		int grid5 = (i - 100) % 5;
		int grid10 = (i - 100) % 10;
		if(!grid5 && grid10)
		{
			for(int j = -2; j < 3; j++)
			{
				nxtSetPixel(i, 32 + j);
			}
		}
		else if(!grid10)
		{
			for(int j = -4; j < 5; j++)
			{
				nxtSetPixel(i, 32 + j);
			}
		}
	}
}

/**********************************************
 * Function that judges if two floats are equal
 **********************************************/
 bool equal(float a, float b) {
   float epsilon = 1;
   if (abs(a-b) < epsilon) {
     return true;
   } else {
     return false;
   }
 }


/*****************************************
 * Main function - Needs changing
 *****************************************/
task main()
{
	// Team 15 PID Code
	float Kp = 10; // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
	float Kd = 0; // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
	float RIGHT_MAX_SPEED = 80; // max speed of the robot
	float LEFT_MAX_SPEED = 80;  // max speed of the robot
	float RIGHT_BASE_SPEED = 15; // this is the speed at which the motors should spin when the robot is perfectly on the line
	float LEFT_BASE_SPEED = 15; // this is the speed at which the motors should spin when the robot is perfectly on the line
	float rightMotorSpeed = 0;
	float leftMotorSpeed = 0;
	int turn = 1; // 1 --> Right, -1 --> Left

	float lastError = 0;

	// Might have to adjust the middle dark value
	// should be minus black value
	float error = SensorValue[lightSensor] - 32;
	float motorPower = Kp * error + Kd * (error - lastError);

	lastError = error;

  // if statement right here based on curvature to add the motor power or subtract power base on which way turning
	float K = 1;

	if(K<0){
		rightMotorSpeed = RIGHT_BASE_SPEED + motorPower;
		leftMotorSpeed = LEFT_BASE_SPEED -  motorPower;
	} else{
		rightMotorSpeed = RIGHT_BASE_SPEED - motorPower;
		leftMotorSpeed = LEFT_BASE_SPEED +  motorPower;
	}

  if (rightMotorSpeed > RIGHT_MAX_SPEED ) rightMotorSpeed = RIGHT_MAX_SPEED; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > LEFT_MAX_SPEED ) leftMotorSpeed = LEFT_MAX_SPEED; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

	motor[motorA] = rightMotorSpeed;
	motor[motorB] = leftMotorSpeed;

	// Team 15 PID CODE

	/* Reset encoders and turn on PID control */
	//nMotorEncoder[motorB] = 0;
	//nMotorEncoder[motorC] = 0;
	//nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
	//nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
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
		//motor[motorA] = motorPower;
		//motor[motorB] = motorPower;
		distTravelled = sqrt(pow(robot_X - start_X, 2) + pow(robot_Y - start_Y, 2));
		while (!equal(distTravelled, goalStraight)) {
			distTravelled = sqrt(pow(robot_X - start_X, 2) + pow(robot_Y - start_Y, 2));
		}


		wait1Msec(100 * 5);
	}
	//motor[motorA] = 0;
	//motor[motorB] = 0;
	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
