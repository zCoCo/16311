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

#include "Toolbox/Display/DisplayStack.h"
#include "Toolbox/Util/UtilStack.h"
#include "Toolbox/Positioning/PositioningStack.h"
#include "Toolbox/HALs/HAL.h"

//Change these during demo
int inputStraight[2] = {0, 0}; // in mm
int inputTurn[2] = {0, 0}; // in degrees, negative means clockwise rotation

/*****************************************
 * Main function - Needs changing
 *****************************************/
task main()
{
	// Team 15 PID Code
	float Kp = 1; // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
	float Kd = 0; // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
	float RIGHT_MAX_SPEED = 80; // max speed of the robot
	float LEFT_MAX_SPEED = 80;  // max speed of the robot
	float RIGHT_BASE_SPEED = 40; // this is the speed at which the motors should spin when the robot is perfectly on the line
	float LEFT_BASE_SPEED = 40; // this is the speed at which the motors should spin when the robot is perfectly on the line
	float rightMotorSpeed = 0;
	float leftMotorSpeed = 0;

	float lastError = 0;

	int goalStraight = 0;
	int goalTurn = 0;
	float start_X = 0;
	float start_Y = 0;
	float distTravelled = 0;

	init_HAL();
	startTask(odometry);

	//Pre-Allocate:
	float error, motorPower;
	float K, norm_factor;

	while(1){
		// Might have to adjust the middle dark value

		error = SensorValue[lightSensor] - 22; //mySensorBar.getPosition() - 0; //getposition value can be negative check this

		motorPower = Kp * error + Kd * (error - lastError);

		lastError = error;

	  // if statement right here based on curvature to add the motor power or subtract power base on which way turning
		K = 0;
		norm_factor = 0;
		for(int i=1; i<=Hist_Curv.numElements; i++){ norm_factor += 1 / i; }
		for(int i=0; i<Hist_Curv.numElements; i++){
			K += Hist_Curv.que[i] / (i+1) / norm_factor;
		}

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

		motor[RightMotor] = rightMotorSpeed;
		motor[LeftMotor] = leftMotorSpeed;
	} // Line Following Loop

	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
