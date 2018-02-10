#ifndef _HAL_LAB3_WMR_H
#define _HAL_LAB3_WMR_H

/****
  Config Description:
    Two Wheeled Mobile Robot (WMR) with Castor at Rear to Support Balancing.
  Connectivity:
    (ref: ball castor at rear, light sensor at front)
    Left Motor -> MotorC
    Right Motor -> MotorB
****/

/* ---- PARAMETERS ---- */
  #define MOTOR_PID_UPDATE_INTERVAL 2
  #define VELOCITY_UPDATE_INTERVAL 4

  #define WHEEL_DIAMETER 0.0572 // [m]
  #define WHEEL_CIRCUMFERENCE 0.1797 // [m]
  #define WHEEL_TREAD 0.1236 // [m], Distance between Left and Right Wheel Tracks

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

/* ---- CORE: ---- */
void init_HAL(){


  // Tell Motors to Operate Closed-Loop (being able to command a specific number
  // of ticks per second:)
  nMotorPIDSpeedCtrl[LeftMotor] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[RightMotor] = mtrSpeedReg;
  // Sync Motors:
  nSyncedMotors = synchABC; // Sync All Motors
  nSyncedTurnRatio = 100; // 100 = 1:1 Ratio

  // SYNC MOTORS
  nMaxRegulatedSpeedNxt = TICKS_PER_REV * MAX_REV_PER_SECOND;
  nPidUpdateInterval = MOTOR_PID_UPDATE_INTERVAL;

  // Initialize Sensor Deltas:
  nMotorEncoder[LeftMotor] = 0;
  nMotorEncoder[RightMotor] = 0;
  clearTimer(OdometryClock);

} // #init

void reset_HAL(){
  // Reset Continuous Data Streams for Odometry:
  /* -- TODO -- */
} // #reset

/* ---- CONTROL ---- */
task balance(){

  // Load Sensor Value

} // #odometry


#endif // _HAL_LAB3_WMR_H
