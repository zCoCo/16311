#pragma config(Sensor, S3, lightSensorFront,  sensorLightActive)
#pragma config(Sensor, S2, lightSensorRear,   sensorLightActive)
/**********************************************
 * Lab 4
 * Team 15
 **********************************************/
#include "Toolbox/Display/DisplayStack.h"
#include "Toolbox/Util/UtilStack.h"
#include "Toolbox/Positioning/PositioningStack.h"
#include "Toolbox/HALs/HAL.h"
/*****************************************
 * Main function - Needs changing
 *****************************************/
task main()
{
	// Team 15 PID Code
	float Kp = 5.0; // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
	float Kd = 0.0; // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
	float Ki = 0.0;
	float setpoint = -5.0;
	float RIGHT_MAX_SPEED = 100.0; // max speed of the robot
	float LEFT_MAX_SPEED = 100.0;  // max speed of the robot
	float RIGHT_MIN_SPEED = -100.0; // min speed
	float LEFT_MIN_SPEED = -100.0; // min speed
	float rightMotorSpeed = 0.0;
	float leftMotorSpeed = 0.0;
	static float lastError = 0.0;
	//Pre-Allocate:
	static float error, motorPower;
	while(1){
		// Might have to adjust the middle dark value
		// error = SensorValue[lightSensor] - 24; //mySensorBar.getPosition() - 0; //getposition value can be negative check this
		error = SensorValue[lightSensorFront] - SensorValue[lightSensorRear] + setpoint; // setpoint in sensor offset

		motorPower = Kp * error + Kd * (error - lastError); //0 at balance and neg. otherwise falling front, pos. falling back

		lastError = error;

		rightMotorSpeed = motorPower;
		leftMotorSpeed  = motorPower;

	  if (rightMotorSpeed > RIGHT_MAX_SPEED) rightMotorSpeed = RIGHT_MAX_SPEED; // prevent the motor from going beyond max speed
	  if (leftMotorSpeed  > LEFT_MAX_SPEED)  leftMotorSpeed  = LEFT_MAX_SPEED; // prevent the motor from going beyond max speed
	  if (rightMotorSpeed < RIGHT_MIN_SPEED) rightMotorSpeed = RIGHT_MIN_SPEED; // Prevent the motor from going below min speed
	  if (leftMotorSpeed  < LEFT_MIN_SPEED)  leftMotorSpeed  = LEFT_MIN_SPEED; // Prevent the motor from going below min speed

		motor[RightMotor] = rightMotorSpeed;
		motor[LeftMotor] = leftMotorSpeed;
	} // Line Following Loop
	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
