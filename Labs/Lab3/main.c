/**********************************************
 * Lab 3 : Starter code
 * Written by Kaushik Viswanathan,
 * Modified by Allan Wang (Jan 2017)

 * Feel free to modify any part of these codes.
 **********************************************/

#include "RobotCIncludes.h"

#include "Toolbox/Display/DisplayStack.h"
#include "Toolbox/Util/UtilStack.h"
#include "Toolbox/Positioning/PositioningStack.h"
#include "Toolbox/Planning/PlanningStack.h"
#include "Toolbox/Control/ControlStack.h"
#include "Toolbox/HALs/HAL.h"

//Change these during demo
TPose Pstart, Pend;
float inputStraight[2] = {33.94 * INCH, 26.83 * INCH}; // in mm
float inputTurn[2] = {-46.0*DEG, 162.57*DEG}; // in degrees, negative means clockwise rotation

task main()
{

	int goalStraight = 0.6096; // [m]
	int goalTurn = 0;
	float start_X = 0;
	float start_Y = 0;
	float distTravelled = 0;

  init_HAL();
	startTask(odometry);

  for(int i=0; i<2; i++){
    // TURN:
    Set_TPose(Pstart, 0,0,0);
    Set_TPose(Pend, 0,0,inputTurn[i]);

    LinearTrajectory ltt;
    Init_LinearTrajectory(ltt, &Pstart, &Pend, 0.3, 70*DEG);
    run_linearTrajectory(&ltt);

    // STRAIGHT:
    Set_TPose(Pend, inputStraight[i],0,0);

    LinearTrajectory lts;
    Init_LinearTrajectory(lts, &Pstart, &Pend, 0.3, 70*DEG);
    run_linearTrajectory(&lts);
  }
/*
	for(int i = 0; i < 2; i++)
	{
		goalStraight = inputStraight[i];
		goalTurn = inputTurn[i];

		start_X = TSF_Last(Hist_Position)->X;
		start_Y = TSF_Last(Hist_Position)->Y;

		moveAt(0.3,0);
    wait10Msec(200);
    moveAt(0,2);
    wait10Msec(200);
		distTravelled = sqrt(pow(rob_pos->X - start_X, 2) + pow(rob_pos->Y - start_Y, 2));
		while (!equal(distTravelled, goalStraight, 1)) {
			distTravelled = sqrt(pow(rob_pos->X - start_X, 2) + pow(rob_pos->Y - start_Y, 2));
		}

		wait1Msec(100 * 5);
	}*/
	motor[LeftMotor] = 0;
	motor[RightMotor] = 0;
	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
