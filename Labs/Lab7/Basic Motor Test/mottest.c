#pragma config(Sensor, S3,     lightSensor,         sensorLightActive)
#pragma config(Sensor, S1,     sonarSensor,         sensorSONAR)
/**********************************************
 * Basic Motor Test Script.
 **********************************************/

//Global variables - you will need to change some of these
//Robot's positions

#include "Toolbox/Util/UtilStack.h"
#include "Toolbox/Positioning/PositioningStack.h"
#include "Toolbox/Planning/PlanningStack.h"
#include "Toolbox/Control/ControlStack.h"
#include "Toolbox/HALs/HAL.h"

#define MainClock T1

// ---- DISPLAY --- //
void update_display(){
	nxtSetPixel(50 + (int)(100.0 * 0.0), 32 + (int)(100.0 * 0.0));
	nxtDisplayTextLine(4, "t: %dms", TSF_Last(Hist_Time));
}

/*****************************************
 * Main function
 *****************************************/

 TPose Pstart, Pend; // Start and End Poses for Each Trajectory
 LinearTrajectory ltt; // Trajectory to be run

task main()
{

	init_HAL();
	startTask(odometry);

	float target_X, target_Y, target_TH;

	static float TARG_Dx, TARG_Dy, TARG_Dth, VEL_FACTOR;
	TARG_Dx = TARG_Dy = TARG_Dth = 0.0;
	VEL_FACTOR = 0.9;

	while(1){
		if(TARG_Dx != 0.0 || TARG_Dy != 0.0 || TARG_Dth != 0.0){
			target_X = rob_pos_X + TARG_Dx;
			target_Y = rob_pos_Y + TARG_Dy;
			target_TH = rob_pos_TH + TARG_Dth;
			
			Set_TPose(Pstart, rob_pos_X, rob_pos_Y, rob_pos_TH);
			Set_TPose(Pend, target_X, target_Y, target_TH);

			Init_LinearTrajectory(ltt, &Pstart, &Pend, VEL_FACTOR*MAX_VEL, VEL_FACTOR*MAX_OMEGA); // Slow Turns
			run_linearTrajectory_fbk(&ltt);

			motor[RightMotor] = 0;
			motor[LeftMotor] = 0;

			TARG_Dx = 0.0; // Reset Command
			TARG_Dy = 0.0;
			TARG_Dth = 0.0;
		}

		update_display();
		wait1Msec(10);
	}

	motor[RightMotor] = 0;
	motor[LeftMotor] = 0;

	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
