/**********************************************
 * Lab 5
 *
 * Objective: Navigate to a Goal Position in the World-Frame using the Given
 * Set of Waypoints Generated using the Wavefront Algorithm in Matlab.
 **********************************************/

#include "RobotCIncludes.h"

// Define Initial Pose of Robot in World-Frame (must be before Odometry include):
#include "Toolbox/Math/Units.h"
#define INIT_POSE_X 6.0 * INCH
#define INIT_POSE_Y -66.0 * INCH
#define INIT_POSE_TH 0.0 * DEG

// CALIBRATION POINTS:
// #define NUM_WAYPOINTS 3
// float WayPoint_Xs [NUM_WAYPOINTS] = {0.0 * INCH, 12.0 * INCH, 24.0*INCH};
// float WayPoint_Ys [NUM_WAYPOINTS] = {12.0 * INCH, 12.0 * INCH, 24.0*INCH};

#define NUM_WAYPOINTS 10
float WayPoint_Xs [NUM_WAYPOINTS] = {
6.000000 * INCH,
20.000000 * INCH,
28.000000 * INCH,
39.000000 * INCH,
39.000000 * INCH,
38.000000 * INCH,
30.000000 * INCH,
39.000000 * INCH,
39.000000 * INCH,
41.000000 * INCH
}
float WayPoint_Ys [NUM_WAYPOINTS] = {
-66.000000 * INCH,
-60.000000 * INCH,
-60.000000 * INCH,
-49.000000 * INCH,
-47.000000 * INCH,
-45.000000 * INCH,
-37.000000 * INCH,
-28.000000 * INCH,
-25.000000 * INCH,
-19.000000 * INCH
}

float target_X = INIT_POSE_X;
float target_Y = INIT_POSE_Y;
float target_TH = INIT_POSE_TH;

#include "Toolbox/Display/DisplayStack.h"
#include "Toolbox/Util/UtilStack.h"
#include "Toolbox/Control/Controller.h"
#include "Toolbox/HALs/HAL.h"

#define MainClock T1

// Update Display with Relevant Information:
task disp(){
  nSchedulePriority = 1; // Very Low Priority (not lowest)
  draw_grid();
  while(1){
    nxtDisplayTextLine(0, "Cx: %f m", rob_pos_X);
    nxtDisplayTextLine(1, "Cy: %f m", rob_pos_Y);
    nxtDisplayTextLine(2, "Cth: %f rad", rob_pos_TH);
    nxtDisplayTextLine(3, "Tx: %f m", target_X);
    nxtDisplayTextLine(4, "Ty: %f m", target_Y);
    nxtDisplayTextLine(5, "Tth: %f rad", target_TH);
    nxtDisplayTextLine(6, "V: %f mV", nAvgBatteryLevel);
    wait1Msec(10); // CPU Relief
  }
} // #disp

TPose Pstart, Pend; // Start and End Poses for Each Trajectory
LinearTrajectory ltt; // Trajectory to be run

task main(){
// Initialize:
  init_HAL();
  startTask(disp);
  startTask(odometry);

  int i = 0;
  while(i < NUM_WAYPOINTS){
    target_X = WayPoint_Xs[i];
    target_Y = WayPoint_Ys[i];
    target_TH = 0.0;

    Set_TPose(Pstart, rob_pos_X, rob_pos_Y, rob_pos_TH);
    Set_TPose(Pend, target_X, target_Y, target_TH);

    Init_LinearTrajectory(ltt, &Pstart, &Pend, 0.9*MAX_VEL, 0.3*MAX_OMEGA); // Slow Turns
    run_linearTrajectory_fbk(&ltt);

    wait1Msec(100); // Wait 100ms between trajectories
    i += 1;
  } // i<NUM_WAYPOINTS?

  motor[LeftMotor] = 0;
  motor[RightMotor] = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
