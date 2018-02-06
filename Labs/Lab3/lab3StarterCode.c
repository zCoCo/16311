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
//int inputStraight[2] = {0, 0}; // in mm
//int inputTurn[2] = {0, 0}; // in degrees, negative means clockwise rotation

#define SearchTimer T3

#define SEARCH_LIGHT_THRESH 35
void search__i_a(){
	static bool first_call = true;
	static float last_th = 0.0;
	static float flipped = false;
	if(first_call || SensorValue[lightSensor] < SEARCH_LIGHT_THRESH){
		clearTimer(SearchTimer);
		first_call = false;
		last_th = rob_pos->TH;
	}
	if(time1[SearchTimer] > 350){
		static int dir = 1;
		if(rob_pos->Y < -1*FT){
			dir = -1;
		} else{
			dir = 1;
		}
		while(SensorValue[lightSensor] > SEARCH_LIGHT_THRESH){
			if(adel(rob_pos->TH, last_th) < 120*DEG && !flipped && dir>0
			|| adel(rob_pos->TH, last_th) > -120*DEG &&!flipped && dir<0){
				moveAt(0,dir*0.35*MAX_OMEGA);
			} else {
				while(adel(rob_pos->TH, last_th) > 0 && dir>0
					 || adel(rob_pos->TH, last_th) < 0 && dir<0){
					moveAt(0,-dir*0.8*MAX_OMEGA);
					nxtSetPixel(50 + (int)(135.0 * 0.0), 32 + (int)(100.0 * 0.0));
					nxtDisplayTextLine(0, "L: %d", SensorValue[lightSensor]);
					nxtDisplayTextLine(1, "t: %dms", TSF_Last(Hist_Time));
					wait1Msec(1);
				}
				flipped = true;
			}
			if(flipped){
				moveAt(0,-dir*0.35*MAX_OMEGA);
			}
			wait1Msec(2);
		}
		flipped = false;
		moveAt(0,0);
	}
}

/*****************************************
 * Main function - Needs changing
 *****************************************/
task main()
{
	// Team 15 PID Code
	float Kp = 1.0; // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
	float Kd = .2; // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
	float RIGHT_MAX_SPEED = 100; // max speed of the robot
	float LEFT_MAX_SPEED = 100;  // max speed of the robot
	float RIGHT_BASE_SPEED = 65; // this is the speed at which the motors should spin when the robot is perfectly on the line
	float LEFT_BASE_SPEED = 65; // this is the speed at which the motors should spin when the robot is perfectly on the line
	float rightMotorSpeed = 0;
	float leftMotorSpeed = 0;

	static float lastError = 0;

	//int goalStraight = 0;
	//int goalTurn = 0;
	//float start_X = 0;
	//float start_Y = 0;
	//float distTravelled = 0;

	init_HAL();
	startTask(odometry);

	//Pre-Allocate:
	static float error, motorPower;
	static float K, norm_factor;

	while(1){
		// Might have to adjust the middle dark value

		error = SensorValue[lightSensor] - 24; //mySensorBar.getPosition() - 0; //getposition value can be negative check this

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

		search__i_a();
		wait1Msec(2);

		/*if (SensorValue[lightSensor] > 29 && time1[T4] > 3000) {
			if (K<0){
			rightMotorSpeed = 90;
			leftMotorSpeed = 0; }
			else{
			rightMotorSpeed = 0;
			leftMotorSpeed = 90;}
		}*/

		motor[RightMotor] = rightMotorSpeed;
		motor[LeftMotor] = leftMotorSpeed;

	    /*Code that plots the robot's current position and also prints it out as text*/
	    nxtSetPixel(50 + (int)(100.0 * 0.0), 32 + (int)(100.0 * 0.0));
	    nxtDisplayTextLine(0, "L: %d", SensorValue[lightSensor]);
			nxtDisplayTextLine(1, "t: %dms", TSF_Last(Hist_Time));
	} // Line Following Loop

	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
