/**********************************************
 * Lab 4
 * Written by Connor W. Colombo
 *
 * Objective: Self Balancing Robot.
 **********************************************/

#include "RobotCIncludes.h"

#include "Toolbox/Display/DisplayStack.h"
#include "Toolbox/Util/UtilStack.h"
#include "Toolbox/Control/PID.h"
#include "Toolbox/HALs/HAL.h"

#define BalanceClock T1

#define PRIMARY_STREAM_LENGTH 15 /* Length of Primary Data Streams */
#define PSL PRIMARY_STREAM_LENGTH /*alias*/
Construct_TSFifo(Hist_SensorFront, float, (PSL/2)); // Front Sensor Data Log (Short for smoothing)
Construct_TSFifo(Hist_SensorFrontSmooth, float, PSL); // Smoothed Front Sensor Data Log
float smoothing_norm; // Normalization Factor for Data Smoothing.

PIDController pidc; // PID Control Data
Construct_TSFifo(Hist_Error, float, PSL); // History of PID Error

// Initialize All Data Sets and Controllers.
void init_data(){
  Init_TSFifo(Hist_SensorFront, (PSL/2));
  TSF_add(Hist_SensorFront, 0);

  Init_TSFifo(Hist_SensorFrontSmooth, PSL);
  TSF_add(Hist_SensorFrontSmooth, 0);

  // Normalization Factor for Pseudo-Exponential Moving Average:
  smoothing_norm = 0;
  for(int i=1; i<=Hist_SensorFront.numElements; i++){ smoothing_norm += 1 / i; }
}

// Update Display with Relevant Information:
task disp(){

} // #disp

/* ---- CONTROL ---- */
task balance(){

  float smooth;
  for(int i=0; i<Hist_Curv.numElements; i++){
    K += Hist_Curv.que[i] / (i+1) / norm_factor;
  }
  // Load Sensor Value

} // #odometry

task main(){
  init_HAL();
  init_data();
  startTask(disp);

  // Determine Baseline after ~1sec for about 1sec.

	startTask(balance);

  // End of Program.

	motor[LeftMotor] = 0;
	motor[RightMotor] = 0;
	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
