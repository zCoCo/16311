#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include "TPose.h"
#include "../Util/TSFifo.h"

// Defines Standard Variables and Functions Neccessary to Perform Odometry.
// Actual Odometry Data Implementation (calculating V,om,dt)
// is Carried Out by the HAL.

Construct_TSFifo(Hist_Position, TPose, 15); // [m,m,rad] Logs TPose in World-Frame
Construct_TSFifo(Hist_DTime, float, 15); // [s] Logs Odometry Update Time Deltas
Construct_TSFifo(Hist_Dist, float, 10); // [m] Logs Path-Length Distance Travelled
Construct_TSFifo(Hist_Vel, float, 15); // [m/s]
Construct_TSFifo(Hist_Omega, float, 15); // [rad/s]
Construct_TSFifo(Hist_Curv, float, 15); // [1/m]

/****
 * Updates the Odometry with the Given Velocity Profile (V,om) since the Last
 * Update at the Given Time Interval, dt, Prior.
****/
void update_odometry(float V, float om, float dt){
  TSF_add(Hist_DTime, dt); // Add

  // Compute Position Change (uses the LAST iteration's data):
  if(dt > 0){
    // Mid-Point Algorithm:
    float new_th = TSF_Last(Hist_Position).TH - TSF_Last(Hist_Omega) * dt/2.0f;
    float new_x = TSF_Last(Hist_Position).X - TSF_Last(Hist_Vel) * cos(new_th) * dt;
    float new_y = TSF_Last(Hist_Position).Y - TSF_Last(Hist_Vel) * sin(new_th) * dt;
    new_th = TSF_Last(Hist_Position).TH - TSF_Last(Hist_Omega) * dt/2.0f;

    TSF_add( Hist_Dist, (TSF_Last(Hist_Dist) + TSF_Last(Hist_Vel) * dt) );

    TSF_add(Hist_Position, (TPose){new_x, new_y, new_th});
  } else{ // Prevent Data-Length Mis-Match:
    TSF_add(Hist_Dist, TSF_Last(Hist_Dist));
    TSF_add(Hist_Position, TSF_Last(Hist_Position));
  }

  // Update Velocity Profile:
  TSF_add(Hist_Vel, V);
  TSF_add(Hist_Omega, om);
  TSF_add(Hist_Curv, (om/V));
} // #update_odometry

#endif // _ODOMETRY_H
