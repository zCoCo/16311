#pragma config(Sensor, S3,     lightSensor,         sensorLightActive)
#pragma config(Sensor, S4,     sonarSensor,         sensorSONAR)
/**********************************************
 * Lab 3 : Starter code
 * Use Bayesian Localization to Determine Position in a Given Map Composed of 16
 * Blocks Arranged around a Circle. Bit Vector of Whether Blocks are Present at
 * Each Posible Location is Given.
 * Team 15
 **********************************************/

//Global variables - you will need to change some of these
//Robot's positions

#include "Toolbox/Display/DisplayStack.h"
#include "Toolbox/Util/UtilStack.h"
#include "Toolbox/Positioning/PositioningStack.h"
#include "Toolbox/Positioning/Odometry.h"
#include "Toolbox/HALs/HAL.h"

// ---- WORLD DATA ---- //
int bitmap[16] = {1,0,1,1,
							 		1,1,1,1,
						 	 		0,1,0,1,
						 	 		1,1,0,1};

// ---- PROBABILITY DATA ---- //
int probmap[16] = {1,1,1,1,
							 		 1,1,1,1,
						 	 		 1,1,1,1,
						 	 		 1,1,1,1}; // Initial Assumption is of Equal Probablility for All Locations
void normalize_prob_map(){

} // #normalize_prob_map

// ---- MOTION DATA ---- //
// Amount by Which Line-Following Oscillations cause the Travelled Distance to
// Exceed the Distance Travelled Around the Circle
float LINEAR_OVERDRIVE_FACTOR = (3*16.0 + 1.0) / (3*16.0);
// Radius of the Track (meters)
#define TRACK_RADIUS (0.3048)
float BLOCKS_PER_METER = 16.0 * LINEAR_OVERDRIVE_FACTOR / TRACK_RADIUS / 6.28318;

// ---- POSITION DATA ---- //
// Change in Blocks since Initialization based on Odometry Alone.
float DBlock_odo = 0.0;

void update_block_position(){
	DBlock_odo = BLOCKS_PER_METER * TSF_Last(Hist_Dist);
} // #update_block_position

#define SearchTimer T3

#define SEARCH_LIGHT_THRESH 35
void search__i_a(){
	static bool first_call = true;
	static float last_th = 0.0;
	static float flipped = false;
	if(first_call || SensorValue[lightSensor] < SEARCH_LIGHT_THRESH){
		clearTimer(SearchTimer);
		first_call = false;
		last_th = rob_pos_TH;
	}
	if(time1[SearchTimer] > 350){
		static int dir = 1;
		while(SensorValue[lightSensor] > SEARCH_LIGHT_THRESH){
			if(adel(rob_pos_TH, last_th) < 120*DEG && !flipped && dir>0
			|| adel(rob_pos_TH, last_th) > -120*DEG &&!flipped && dir<0){
				moveAt(0,dir*0.35*MAX_OMEGA);
			} else {
				while(adel(rob_pos_TH, last_th) > 0 && dir>0
					 || adel(rob_pos_TH, last_th) < 0 && dir<0){
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

float rightMotorSpeed = 0;
float leftMotorSpeed = 0;

task main()
{
	// Team 15 PID Code
	float Kp = 2; // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
	float Kd = .2; // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
	float RIGHT_MAX_SPEED = 60; // max speed of the robot
	float LEFT_MAX_SPEED = 60;  // max speed of the robot
	float RIGHT_BASE_SPEED = 20; // this is the speed at which the motors should spin when the robot is perfectly on the line
	float LEFT_BASE_SPEED = 20; // this is the speed at which the motors should spin when the robot is perfectly on the line
	static float lastError = 0;
	//  Ultrasonic Sensor has a range of states between 0 and 255
	//  Number is representative of the current reading in centimeters
	//  A reading of 255 means that the current sensor reading is out of range.
	float distance_in_cm = 10.0;

	init_HAL();
	startTask(odometry);

	//Pre-Allocate:
	static float error, motorPower;
	static float K, norm_factor;

	while(DBlock_odo < 3.0*16.0){
		// Might have to adjust the middle dark value

		error = SensorValue[lightSensor] - 35;

		motorPower = Kp * error + Kd * (error - lastError);

		lastError = error;

	  // if statement right here based on curvature to add the motor power or subtract power base on which way turning
		K = 0;
		norm_factor = 0;
		for(int i=1; i<=Hist_Curv.numElements; i++){ norm_factor += 1 / i; }
		for(int i=0; i<Hist_Curv.numElements; i++){
			K += Hist_Curv.que[i] / (i+1) / norm_factor;
		}
		/*
		if(K<0){
			rightMotorSpeed = RIGHT_BASE_SPEED + motorPower;
			leftMotorSpeed = LEFT_BASE_SPEED -  motorPower;
		} else{
			rightMotorSpeed = RIGHT_BASE_SPEED - motorPower;
			leftMotorSpeed = LEFT_BASE_SPEED +  motorPower;
		}
		*/
		rightMotorSpeed = RIGHT_BASE_SPEED + motorPower;
		leftMotorSpeed  = LEFT_BASE_SPEED  - motorPower;
		// Ensure Bounds aren't Exceeded. Scale Both Motor Speeds if one is capped.
	  if (rightMotorSpeed > RIGHT_MAX_SPEED ) { leftMotorSpeed = leftMotorSpeed * RIGHT_MAX_SPEED / rightMotorSpeed; rightMotorSpeed = RIGHT_MAX_SPEED; } // prevent the motor from going beyond max speed
	  if (leftMotorSpeed > LEFT_MAX_SPEED ) { rightMotorSpeed = rightMotorSpeed * LEFT_MAX_SPEED / leftMotorSpeed; leftMotorSpeed = LEFT_MAX_SPEED; } // prevent the motor from going beyond max speed
	  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
	  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

		search__i_a();
		wait1Msec(2);

		motor[RightMotor] = rightMotorSpeed;
		motor[LeftMotor] = leftMotorSpeed;

		// Sonar value less than distance value mean it seen the box
		if (SensorValue[sonarSensor] < distance_in_cm){
			bPlaySounds = true;   // ACCEPT new sound requests
			// play tone according to light sensor readings
			// first parameter frequency, duration in 10MsecTicks
			PlayImmediateTone(261, 100);
		}
		else {
			bPlaySounds = false;
			ClearSounds();
		}

		update_block_position();

    nxtSetPixel(50 + (int)(100.0 * 0.0), 32 + (int)(100.0 * 0.0));
    nxtDisplayTextLine(0, "L: %d", SensorValue[lightSensor]);
    //nxtDisplayTextLine(1, "DB: %f", DBlock_odo);
    nxtDisplayTextLine(1, "DB: %f", SensorValue[sonarSensor]);
		nxtDisplayTextLine(2, "t: %dms", TSF_Last(Hist_Time));
	} // Line Following Loop
	motor[RightMotor] = 0;
	motor[LeftMotor] = 0;

	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
