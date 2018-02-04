#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include "TPose.h"
#include "../Util/TSFifo.h"

#include "../Display/DisplayStack.h"

#define PRINT_ODOMETRY_DATA // Comment-Out to Disable

// Defines Standard Variables and Functions Neccessary to Perform Odometry.
// Actual Odometry Data Implementation (calculating V,om,dt)
// is Carried Out by the HAL.

Construct_TSFifo(Hist_Position, TPose, 15); // [m,m,rad] Logs TPose in World-Frame
Construct_TSFifo(Hist_Time, long, 15); // [ms] Logs Odometry Update Time Deltas
Construct_TSFifo(Hist_Dist, float, 10); // [m] Logs Path-Length Distance Travelled
Construct_TSFifo(Hist_Vel, float, 15); // [m/s]
Construct_TSFifo(Hist_Omega, float, 15); // [rad/s]
Construct_TSFifo(Hist_Curv, float, 15); // [1/m]

// Initialize Odometry Data:
void init_odometry(){
  Init_TSFifo(Hist_Position);
  TPose zero_pose; zero_pose.X = 0; zero_pose.Y = 0; zero_pose.TH = 0;
  TSF_add(Hist_Position, zero_pose);

  Init_TSFifo(Hist_Time);
  TSF_add(Hist_Time, 0);

  Init_TSFifo(Hist_Vel);
  TSF_add(Hist_Vel, 0);

  Init_TSFifo(Hist_Omega);
  TSF_add(Hist_Omega, 0);

  Init_TSFifo(Hist_Curv);
  TSF_add(Hist_Curv, 0);

  #ifdef PRINT_ODOMETRY_DATA
    draw_grid();
  #endif
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
  static float new_x, new_y, new_th;

  update_timeLog(dt);

  // Compute Position Change (uses the LAST iteration's data):
  if(dt > 0){
    // Mid-Point Algorithm:
    // Allocate static space for variables since these will likely be used often
    new_th = TSF_Last(Hist_Position).TH - TSF_Last(Hist_Omega) * dt/2.0;
    new_x = TSF_Last(Hist_Position).X - TSF_Last(Hist_Vel) * cos(new_th) * dt;
    new_y = TSF_Last(Hist_Position).Y - TSF_Last(Hist_Vel) * sin(new_th) * dt;
    new_th = TSF_Last(Hist_Position).TH - TSF_Last(Hist_Omega) * dt/2.0;

    TSF_add( Hist_Dist, (TSF_Last(Hist_Dist) + TSF_Last(Hist_Vel) * dt) );

    TPose new_pose;
    new_pose.X = new_x; new_pose.Y = new_y; new_pose.TH = new_th;
    TSF_add(Hist_Position, new_pose);
  } else{ // Prevent Data-Length Mis-Match:
    TSF_add(Hist_Dist, TSF_Last(Hist_Dist));
    TSF_add(Hist_Position, TSF_Last(Hist_Position));
  }

  // Update Velocity Profile:
  TSF_add(Hist_Vel, V);
  TSF_add(Hist_Omega, om);
  TSF_add(Hist_Curv, (om/V));

  #ifdef PRINT_ODOMETRY_DATA
    /*Code that plots the robot's current position and also prints it out as text*/
    nxtSetPixel(50 + (int)(100.0 * new_x), 32 + (int)(100.0 * new_y));
    nxtDisplayTextLine(0, "X: %fm", new_x);
    nxtDisplayTextLine(1, "Y: %fm", new_y);
    nxtDisplayTextLine(2, "th: %frad", new_th);
    nxtDisplayTextLine(3, "dt: %fs", dt);
    nxtDisplayTextLine(4, "t: %dms", TSF_Last(Hist_Time));
  #endif
} // #update_odometry

#endif // _ODOMETRY_H
