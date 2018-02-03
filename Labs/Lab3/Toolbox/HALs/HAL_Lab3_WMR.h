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
  #define VELOCITY_UPDATE_INTERVAL 4

  #define WHEEL_DIAMETER 0.0572 // [m]
  #define WHEEL_CIRCUMFERENCE 0.1797 // [m]
  #define WHEEL_TREAD 0.1236 // [m], Distance between Left and Right Wheel Tracks

  // Distance per Encoder Tick:
  #define METERS_PER_TICK (WHEEL_CIRCUMFERENCE / TICKS_PER_REV) // [m/tick]

  #define MAX_REV_PER_SECOND 700 // Make sure this is an attainable value

  #define LeftMotor motorC
  #define RightMotor motorB

  #define OdometryClock T1

/* ---- CORE: ---- */
void init(){

  // Tell Motors to Operate Closed-Loop (being able to command a specific number
  // of ticks per second:)
  nMotorPIDSpeedCtrl[LeftMotor] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[RightMotor] = mtrSpeedReg;

  // Initialize Continuous Data Streams for Odometry:
  TSF_add(Hist_Position, (TPose){0,0,0});
  TSF_add(Hist_Time, 0);
  TSF_add(Hist_Vel, 0);
  TSF_add(Hist_Omega, 0);
  TSF_add(Hist_Curv, 0);

} // #init

void reset(){
  // Reset Continuous Data Streams for Odometry:
  TSF_add(Hist_Position, (TPose){0,0,0});
  TSF_add(Hist_Time, 0);
  TSF_add(Hist_Vel, 0);
  TSF_add(Hist_Omega, 0);
  TSF_add(Hist_Curv, 0);
} // #reset

/* ---- SENSING ---- */
task odometry(){
  // Preallocate Loop Variables:
  float Ds_left, Ds_right;
  float Dt;
  float v_l, v_r;

  // Loop:
  while(1){
    // Capture Encoder Values Immediately:
    Ds_left = nMotorEncoder[LeftMotor];
    Ds_right = nMotorEncoder[RightMotor];
    dt = time1[OdometryClock]; // Ensure this is at time of capture.

    // Reset (after capture) so Value Represents a Delta (and to help prevent overflows):
    nMotorEncoder[LeftMotor] = 0;
    nMotorEncoder[RightMotor] = 0;
    ClearTimer(OdometryClock);

    // Convert to Standard Units:
    Ds_left = METERS_PER_TICK * Ds_left;
    Ds_right = METERS_PER_TICK * Ds_right;
    dt = dt * 1e-3; // ms -> s

    // Compute Velocity Profile:
    v_l = Ds_left / dt; v_r = Ds_right / dt;

    // Compute Inverse Kinematics:
    V = (v_r + v_l) / 2.0f;
    om = (v_r - v_l) / WHEEL_TREAD;

    update_odometry(V,om,dt);

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
  v_l = V - (WHEEL_TREAD / 2.0f) * omega; // [m/s]
  v_r = V + (WHEEL_TREAD / 2.0f) * omega;

  // Command Specific Velocity (in m/s):
}

// #limitWheelVelocity(V,om)

#endif // _HAL_LAB3_WMR_H
