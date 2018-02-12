#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include "TPose.h"
#include "../Util/TSFifo.h"

//#define PRINT_ODOMETRY_DATA // Comment-Out to Disable

// Defines Standard Variables and Functions Neccessary to Perform Odometry.
// Actual Odometry Data Implementation (calculating V,om,dt)
// is Carried Out by the HAL.

Construct_TSFifo(Hist_Pos_X, float, 15); // [m,m,rad] Logs TPose in World-Frame
Construct_TSFifo(Hist_Pos_Y, float, 15); // [m,m,rad] Logs TPose in World-Frame
Construct_TSFifo(Hist_Pos_TH, float, 15); // [m,m,rad] Logs TPose in World-Frame
Construct_TSFifo(Hist_Time, long, 2); // [ms] Logs Odometry Update Time Deltas
Construct_TSFifo(Hist_Dist, float, 2); // [m] Logs Path-Length Distance Travelled
Construct_TSFifo(Hist_Vel, float, 2); // [m/s]
Construct_TSFifo(Hist_Omega, float, 2); // [rad/s]
Construct_TSFifo(Hist_Curv, float, 2); // [1/m]

#define rob_pos_X TSF_Last(Hist_Pos_X)
#define rob_pos_Y TSF_Last(Hist_Pos_Y)
#define rob_pos_TH TSF_Last(Hist_Pos_TH)
//Hist_Position.que[Hist_Position.numElements-1]

// Initialize Odometry Data:
void init_odometry(){
  Init_TSFifo(Hist_Pos_X, 15);
  TSF_add(Hist_Pos_X, 0);
  Init_TSFifo(Hist_Pos_Y, 15);
  TSF_add(Hist_Pos_Y, 0);
  Init_TSFifo(Hist_Pos_TH, 15);
  TSF_add(Hist_Pos_TH, 0);

  Init_TSFifo(Hist_Time, 2);
  TSF_add(Hist_Time, 0);

  Init_TSFifo(Hist_Dist, 2);
  TSF_add(Hist_Dist, 0);

  Init_TSFifo(Hist_Vel, 2);
  TSF_add(Hist_Vel, 0);

  Init_TSFifo(Hist_Omega, 2);
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
  TPose new_pose; // not static, needs new instance each iteration
  static float V0, om0, s0;

  update_timeLog(dt);

  // Compute Position Change (uses the LAST iteration's data):
  if((1000.0*dt) > 0.0){ // *1000 fixes an issue this was having
    V0 = TSF_Last(Hist_Vel);
    om0 = TSF_Last(Hist_Omega);

    if(Hist_Pos_X.numElements <= 1){
      // Ensure Proper Initialization (zero_pose seems faulty)
      new_pose.X = 0.0; new_pose.Y = 0.0; new_pose.TH = 0.0;
    } else{
      // Grab Data from Last Iteration:
      new_pose.X = rob_pos_X; new_pose.Y = rob_pos_Y; new_pose.TH = rob_pos_TH;
    }

    // Mid-Point Algorithm:
    new_pose.TH = new_pose.TH + om0 * dt/2.0;
    new_pose.X = new_pose.X + V0 * cos(new_pose.TH) * dt;
    new_pose.Y = new_pose.Y + V0 * sin(new_pose.TH) * dt;
    new_pose.TH = new_pose.TH + om0 * dt/2.0;

    // s0 = TSF_Last(Hist_Dist);
    // TSF_add( Hist_Dist, (s0 + abs(V0) * dt) );

    TSF_add(Hist_Pos_X, new_pose.X);
    TSF_add(Hist_Pos_Y, new_pose.Y);
    TSF_add(Hist_Pos_TH, new_pose.TH);
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
