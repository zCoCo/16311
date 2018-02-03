/**********************************************
 * Lab 3 : Starter code
 * Written by Kaushik Viswanathan,
 * Modified by Allan Wang (Jan 2017)

 * Feel free to modify any part of these codes.
 **********************************************/

//Change these during demo
int inputStraight[2] = {0, 0}; // in mm
int inputTurn[2] = {0, 0}; // in degrees, negative means clockwise rotation
int motorPower = 50;

task main()
{

	int goalStraight = 0;
	int goalTurn = 0;
	float start_X = 0;
	float start_Y = 0;
	float distTravelled = 0;

  init_HAL();
	startTask(odometry);

	for(int i = 0; i < 2; i++)
	{
		goalStraight = inputStraight[i];
		goalTurn = inputTurn[i];

		//
		// Write your own codes for turning
		//

		start_X = TSF_Last(Hist_Position).X;
		start_Y = TSF_Last(Hist_Position).Y;

		/* Example codes for moving in a striaght line a certain distance,
		 * you need to change this for MUCH better performance */
     // Implement Real PID on Y-Component (minimize Cross-Track Motion)
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
