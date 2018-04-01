#ifndef _HAL_LAB7_TRACKS_BAD_H
#define _HAL_LAB7_TRACKS_BAD_H

#define LEADING_EDGE -1.0
// 1.0 -> Wheels first, -1.0 -> Tracks first

/****
  Config Description:
    Two Wheeled Mobile Robot (WMR) with Castor at Rear to Support Balancing.
  Connectivity:
    (ref: ball castor at rear, light sensor at front)
    Left Motor -> MotorC
    Right Motor -> MotorB
****/

#include "../Positioning/Odometry.h"

/* ---- PARAMETERS ---- */
  #define MOTOR_PID_UPDATE_INTERVAL 2
  #define VELOCITY_UPDATE_INTERVAL 4

  #define WHEEL_DIAMETER (0.036667) // [m] (Effective OD, accounting for gearing)
  #define WHEEL_CIRCUMFERENCE (0.096342) // [m] (Effective C, accounting for gearing)
  #define WHEEL_TREAD 0.140 // [m], Distance between Left and Right Wheel Tracks

  // Distance per Encoder Tick:
  float METERS_PER_TICK = (((float) (WHEEL_CIRCUMFERENCE)) / ((float) (TICKS_PER_REV))); // [m/tick]

  #define MAX_REV_PER_SECOND 2.0 // Make sure this is an attainable value

  // [m/s] - Maximum Linear Velocity:
  // Def:L 0.6, 2.0
  #define MIN_VELOCITY 0.06
  #define MIN_VEL MIN_VELOCITY /*Alias*/
  float MAX_VEL = (METERS_PER_TICK * TICKS_PER_REV * MAX_REV_PER_SECOND);
  #define MAX_ACCEL 0.4 //[m/s/s] - Maximum Linear Acceleration

  // [rad/s] - Maximum Angular Velocity
  float MAX_OMEGA = (2.0 * MAX_VEL) / WHEEL_TREAD;
  #define MAX_ALPHA 3.5 // [rad/s/s] - Maximum Angular Acceleration

  #define COMMAND_DELAY 0.0 //  -Delay, in sec, a Velocity been Commanded and
                            //     it being Implemented.
  #define COMMAND_READ_DELAY 0.05 //  -Delay, in sec, a Velocity been Commanded and
                            //     it being Implemented and Read Back by Odometry.

  #define LeftMotor motorC
  #define RightMotor motorB

  #define OdometryClock T2

/* ---- CORE: ---- */
void init_HAL(){

  // Tell Motors to Operate Closed-Loop (being able to command a specific number
  // of ticks per second:)
  nMotorPIDSpeedCtrl[LeftMotor] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[RightMotor] = mtrSpeedReg;
  nMaxRegulatedSpeedNxt = TICKS_PER_REV * MAX_REV_PER_SECOND;
  nPidUpdateInterval = MOTOR_PID_UPDATE_INTERVAL;

  // Initialize Sensor Deltas:
  nMotorEncoder[LeftMotor] = 0;
  nMotorEncoder[RightMotor] = 0;
  clearTimer(OdometryClock);

  // Initialize Continuous Data Streams for Odometry:
  init_odometry();

} // #init

void reset_HAL(){
  // Reset Continuous Data Streams for Odometry:
  /* -- TODO -- */
} // #reset

/* ---- SENSING ---- */
task odometry(){
  // Preallocate Loop Variables:
  float Ds_left, Ds_right;
  float dt;
  float v_l, v_r;
  float V, om;

  // Loop:
  while(1){
    // Capture Encoder Values Immediately:
    Ds_left = nMotorEncoder[LeftMotor];
    Ds_right = nMotorEncoder[RightMotor];
    dt = time1[OdometryClock]; // Ensure this is at time of capture.

    // Reset (after capture) so Value Represents a Delta (and to help prevent overflows):
    nMotorEncoder[LeftMotor] = 0;
    nMotorEncoder[RightMotor] = 0;
    clearTimer(OdometryClock);

    // Convert to Standard Units:
    Ds_left = METERS_PER_TICK * Ds_left;
    Ds_right = METERS_PER_TICK * Ds_right;
    dt = dt * 0.001; // ms -> s

    if(dt){
      // Compute Velocity Profile:
      v_l = Ds_left / dt; v_r = Ds_right / dt;
    }

    // Compute Inverse Kinematics:
    V = (v_r + v_l) / 2.0;
    om = (v_r - v_l) / WHEEL_TREAD;

    update_odometry(LEADING_EDGE*V,om,dt);

    wait1Msec(VELOCITY_UPDATE_INTERVAL);
  } // loop
} // #odometry

/* ---- MOTION ---- */

/****
 * moveAt(V,omega)
 * Moves the Robot's center with the given path-velocity and rotational velocity
****/
void moveAt(float V, float omega){
  // Compute Forward Kinematics:
  float v_l = V - (WHEEL_TREAD / 2.0) * omega; // [m/s]
  float v_r = V + (WHEEL_TREAD / 2.0) * omega;
  // Convert to ticks/s:
  v_l = v_l / METERS_PER_TICK; v_r = v_r / METERS_PER_TICK;
  // Convert to % of Max Speed:
  v_l = 100.0 * (v_l / TICKS_PER_REV / MAX_REV_PER_SECOND);
  v_r = 100.0 * (v_r / TICKS_PER_REV / MAX_REV_PER_SECOND);

  motor[LeftMotor] = LEADING_EDGE*v_l;
  motor[RightMotor] = LEADING_EDGE*v_r;
}

#endif // _HAL_LAB7_TRACKS_BAD_H
