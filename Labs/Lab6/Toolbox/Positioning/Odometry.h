#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include "TPose.h"
#include "../Util/TSFifo.h"

//#define PRINT_ODOMETRY_DATA // Comment-Out to Disable

// Define Initial Pose of Robot in World-Frame (if not defined elsewhere)
#ifndef INIT_POSE_X
  #define INIT_POSE_X 0.0
  #define INIT_POSE_Y 0.0
  #define INIT_POSE_TH 0.0
#endif // INIT_POSE_X

// Defines Standard Variables and Functions Neccessary to Perform Odometry.
// Actual Odometry Data Implementation (calculating V,om,dt)
// is Carried Out by the HAL.

Construct_TSFifo(Hist_Pos_X, float, 2); // [m,m,rad] Logs TPose in World-Frame
Construct_TSFifo(Hist_Pos_Y, float, 2); // [m,m,rad] Logs TPose in World-Frame
Construct_TSFifo(Hist_Pos_TH, float, 2); // [m,m,rad] Logs TPose in World-Frame
Construct_TSFifo(Hist_Time, long, 2); // [ms] Logs Odometry Update Time Deltas
Construct_TSFifo(Hist_Dist, float, 3); // [m] Logs Path-Length Distance Travelled
Construct_TSFifo(Hist_Vel, float, 3); // [m/s]
Construct_TSFifo(Hist_Omega, float, 3); // [rad/s]
Construct_TSFifo(Hist_Curv, float, 2); // [1/m]

#define rob_pos_X (TSF_Last(Hist_Pos_X))
#define rob_pos_Y (TSF_Last(Hist_Pos_Y))
#define rob_pos_TH (TSF_Last(Hist_Pos_TH))

// Initialize Odometry Data:
void init_odometry(){
  Init_TSFifo(Hist_Pos_X, 2);
  TSF_add(Hist_Pos_X, INIT_POSE_X);
  Init_TSFifo(Hist_Pos_Y, 2);
  TSF_add(Hist_Pos_Y, INIT_POSE_Y);
  Init_TSFifo(Hist_Pos_TH, 2);
  TSF_add(Hist_Pos_TH, INIT_POSE_TH);

  Init_TSFifo(Hist_Time, 2);
  TSF_add(Hist_Time, 0);

  Init_TSFifo(Hist_Dist, 3);
  TSF_add(Hist_Dist, 0);

  Init_TSFifo(Hist_Vel, 3);
  TSF_add(Hist_Vel, 0);

  Init_TSFifo(Hist_Omega, 3);
  TSF_add(Hist_Omega, 0);

  Init_TSFifo(Hist_Curv, 2);
  TSF_add(Hist_Curv, 0);
} // #init_odometry

// Increments the Time Log, Hist_Time, by the Given Value dt in seconds, accounting
// for Overflow.
void update_timeLog(float dt){
  TSF_add( Hist_Time, ((long) ( TSF_Last(Hist_Time) + (float) (1000.0*dt) )) ); // [ms]
  // TODO: Account for Overflow (won't be issue unless robot runs for more
  // than 24days; so, hold off on this one)
} // #update_timeLog

/****
 * Updates the Odometry with the Given Velocity Profile (V,om) over the Given
 * Update Interval dt, in seconds.
****/
void update_odometry(float V, float om, float dt){
  // Allocate static space for variables since these will likely be used often
  //static float new_pose_X = INIT_POSE_X;
  //static float new_pose_Y = INIT_POSE_Y;
  static float new_pose_TH = INIT_POSE_TH;
  static float V0, om0, s0;

  update_timeLog(dt);

  // Compute Position Change (uses the LAST iteration's data):
  if((1000.0*dt) > 0.0){ // *1000 fixes an issue this was having, for some reason @RobotC
    V0 = TSF_Last(Hist_Vel);
    om0 = TSF_Last(Hist_Omega);

    //new_pose_X = TSF_Last(Hist_Pos_X);
    //new_pose_Y = TSF_Last(Hist_Pos_Y);
    new_pose_TH = TSF_Last(Hist_Pos_TH);

    // Mid-Point Algorithm:
    new_pose_TH = new_pose_TH + om0 * dt/2.0;
    //new_pose_X = new_pose_X + V0 * cos(new_pose_TH) * dt;
    //new_pose_Y = new_pose_Y + V0 * sin(new_pose_TH) * dt;
    new_pose_TH = new_pose_TH + om0 * dt/2.0;

    s0 = TSF_Last(Hist_Dist);
    TSF_add( Hist_Dist, (s0 + V0 * dt) );

    TSF_add(Hist_Pos_TH, 0.0);//TSF_add(Hist_Pos_X, new_pose_X);
    TSF_add(Hist_Pos_TH, 0.0);//TSF_add(Hist_Pos_Y, new_pose_Y);
    TSF_add(Hist_Pos_TH, new_pose_TH);
  } else{ // Prevent Data-Length Mis-Match:
    TSF_add(Hist_Dist, TSF_Last(Hist_Dist));
    TSF_add(Hist_Pos_X, rob_pos_X);
    TSF_add(Hist_Pos_Y, rob_pos_Y);
    TSF_add(Hist_Pos_TH, rob_pos_TH);
  }

  // Update Velocity Profile:
  TSF_add(Hist_Vel, V);
  TSF_add(Hist_Omega, om);
  TSF_add(Hist_Curv, (V==0 ? 0 : (om/V)));
} // #update_odometry

#endif // _ODOMETRY_H
