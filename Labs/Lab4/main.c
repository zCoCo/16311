#pragma config(Sensor, S3,     lightSensorFront,         sensorLightActive)
#pragma config(Sensor, S2,     lightSensorRear,         sensorLightActive)
/**********************************************
 * Lab 4
 *
 * Objective: Self Balancing Robot using Differential Light signals to maintain
 * Vertical Orientation and Odometry to Maintain Position.
 **********************************************/

#include "RobotCIncludes.h"

#include "Toolbox/Display/DisplayStack.h"
#include "Toolbox/Util/UtilStack.h"
#include "Toolbox/Control/PID.h"
#include "Toolbox/HALs/HAL.h"

#define USE_REAR_SENSOR /* Allow this to work with only front sensor */

#define MainClock T1

#define SET_POINT (1.5 * 45) // Lower -> Rearward
float Kp = (-130.0 / 30.0 / 3.5);
float Kd = (-130.0 / 35.0 / 2.5);
float Ki = (-130.0 / 25.0 / 75.0);

// Odometry-Based X-Constraints:
float KpX = 0.0;//(70.0 / 0.40); // Command 10% Power if 50cm off target
#define KdX (KpX/2.5)
#define KiX 0

bool balance_on = false; // Whether Balancing Action should be Running Right Now
bool odometry_on = false; // Whether Odometry Confines should be Running Right Now

// -- Vertical Orientation Data --

float smooth_front = 0.0, smooth_rear = 0.0; // Most Recent Smoothed Sensor Values

float sensor_diff = 0.0; // Difference between Smoothed Sensor Values
float target_diff = 0.0; // Ideal Target Difference between Smoothed Sensor Values

float smoothing_norm; // Normalization Factor for Data Smoothing.

// -- Constant Position Data --

float curr_X = 0.0; // Current X-Position of Robot
float target_X = 0.0; // No Change in X-Position

// -- Controller Data --

PIDController pidc; // PID Control Data (for Main Control of Vertical Orientation)
PIDStream Hist_Error; // Data Log of PID Error (special sub-type of TSFifo)

PIDController pidcX; // Control Data for PID of X-Position
PIDStream Hist_XError; // Data Log of X-Position PID Error (special sub-type of TSFifo)

// -- Control Values --
int u_sens = 0;
int u_pos = 0;
int u_comm = 0;
int u_comm_last = 0;

// Initialize All Data Sets and Controllers.
void init_data(){
  clearTimer(MainClock);

  Init_TSFifo(Hist_SensorFront, PSL);
  TSF_add(Hist_SensorFront, 0);
  Init_TSFifo(Hist_SensorRear, PSL);
  TSF_add(Hist_SensorRear, 0);

  Init_TSFifo(Hist_Error, PSL);
  TSF_add(Hist_Error, 0);
  Init_PIDController(pidc, &sensor_diff, &Hist_Error, &target_diff, Kp, Kd, Ki, 95.0);

  Init_TSFifo(Hist_XError, PSL);
  TSF_add(Hist_XError, 0);
  Init_PIDController(pidcX, &curr_X, &Hist_XError, &target_X, KpX, KdX, KiX, 90.0);

  // Compute Normalization Factor for Pseudo-Exponential Moving Average:
  smoothing_norm = 0.0;
  for(int i=0; i<Hist_SensorFront.numElements; i++){ smoothing_norm += 1.0 / ((float)i+1); }
}

// Update Display with Relevant Information:
task disp(){
  nSchedulePriority = 1; // Very Low Priority (not lowest)
  draw_grid();
  while(1){
    nxtDisplayTextLine(0, "X: %f", curr_X);
    nxtDisplayTextLine(1, "LFr: %d", SensorRaw[lightSensorFront]);
    nxtDisplayTextLine(2, "LFs: %d", smooth_front);
    nxtDisplayTextLine(3, "RFr: %d", SensorRaw[lightSensorRear]);
    nxtDisplayTextLine(4, "RFs: %d", smooth_rear);
    nxtDisplayTextLine(5, "u_s: %d", u_sens);
    nxtDisplayTextLine(6, "V: %d", nAvgBatteryLevel);
    wait1Msec(10); // CPU Relief
  }
} // #disp

// CONTROL
long t_last_call_bal = 0;
task balance(){
  nSchedulePriority = 200; // High Priority.

  while(1){
  if((time1[MainClock] - t_last_call_bal) > 2){
  // Log Sensor Data:
    TSF_add(Hist_SensorFront, (SensorRaw[lightSensorFront]));
    TSF_add(Hist_SensorRear, (SensorRaw[lightSensorRear]));

      sensor_diff = SensorRaw[lightSensorFront] - SensorRaw[lightSensorRear];//smooth_front - smooth_rear;

  // Update Sensor PID:
      if(balance_on){
        u_sens = ((int)getControl(&pidc)); // Get Control Signal from Sensor PID
      }
  // Update Motion PID:
      if(balance_on && odometry_on){
        curr_X = TSF_Last(Hist_Dist); // Update Current X-Position used in Control Loop
        u_pos = ((int)getControl(&pidcX));
      }

  // Command Robot:
      u_comm = (u_sens + u_pos); // Combine Command Signals

      // Ensure No Out-of-Bounds Commands are Sent:
      u_comm = (abs(u_comm) > 100) ? ((u_comm/abs(u_comm)) * 100) : u_comm;
      if(abs(u_comm-u_comm_last) > 70){ // Put a Hard-Cap on Thrashing
        u_comm = u_comm + ((u_comm-u_comm_last) / abs(u_comm-u_comm_last)) * 30;
      }

      motor[LeftMotor] = u_comm;
      motor[RightMotor] = u_comm;
      u_comm_last = u_comm;

      t_last_call_bal = time1[MainClock];
    } // t_last_call_bal?
    wait1Msec(2);
 } // loop
} // #balance


task main(){
// Initialize:
  init_HAL();
  init_data();
  //startTask(disp);
  startTask(odometry); odometry_on = true;
	startTask(balance);

  target_diff = SET_POINT;//target_diff / ((float)count); // Divide to get Average -- Lower is Rearward

  balance_on = true; // With Baseline determined, Start Balancing Act

// End of Main Task - decrease priority to min:
  nSchedulePriority = kLowPriority;
	while(nNxtButtonPressed != kExitButton) {}
}
