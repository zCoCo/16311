#ifndef _HAL_LAB4_TOWER_H
#define _HAL_LAB4_TOWER_H

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
  #define VELOCITY_UPDATE_INTERVAL 2

  #define WHEEL_DIAMETER 0.0816 // [m]
  #define WHEEL_CIRCUMFERENCE 0.2564 // [m]
  #define WHEEL_TREAD 0.159 // [m], Distance between Left and Right Wheel Tracks

  // Distance per Encoder Tick:
  float METERS_PER_TICK = (((float) (WHEEL_CIRCUMFERENCE)) / ((float) (TICKS_PER_REV))); // [m/tick]

  #define MAX_REV_PER_SECOND 2.0 // Make sure this is an attainable value

  // [m/s] - Maximum Linear Velocity:
  // Def:L 0.6, 2.0
  #define MIN_VELOCITY 0.06
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

  #define OdometryClock T4

/* ---- CORE: ---- */
void init_HAL(){

  // Tell Motors to Operate Closed-Loop (being able to command a specific number
  // of ticks per second:)
  nMotorPIDSpeedCtrl[LeftMotor] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[RightMotor] = mtrSpeedReg;
  // Sync Motors:
  // nSyncedMotors = synchBC; // Sync Drive Motors
  // nSyncedTurnRatio = 100; // 100 = 1:1 Ratio

  // SYNC MOTORS
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
    nSchedulePriority = 200; // High Priority.
    // Preallocate Loop Variables:
    long t_last_update_odo = 0;
    float Ds_left, Ds_right;
    float dt;
    float v_l, v_r;
    float V, om;

    // Loop:
    while(1){
      if((time1[OdometryClock] - t_last_update_odo) > VELOCITY_UPDATE_INTERVAL){
        // // Capture Encoder Values Immediately:
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

        update_odometry(V,om,dt);
        t_last_update_odo = time1[OdometryClock];
      } // t>VELOCITY_UPDATE_INTERVAL
      wait1Msec(VELOCITY_UPDATE_INTERVAL);
    } // loop
} // #odometry


#endif // _HAL_LAB4_TOWER_H
