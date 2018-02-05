#ifndef _FEEDFORWARD_H
#define _FEEDFORWARD_H

  #include "../Math/MathStack.h"
  #include "../Planning/Trajectory.h"

  // Data Necessary for Successfully Executing a Trapezoidal Velocity Profile
  // for Either Linear or Angular Motion in which the Robot Covers a Distance,
  // dist, while Ramping Up to the Peak Velocity (if possible), V_peak,
  // Holding there (if applicable), and Ramping Back Down, all While Staying
  // Below the Maximum Acceleration, A_max.
  typedef struct{
      float dist; //      - Total Distance Covered (computed value)
      float V_peak; //    - Peak Velocity
      float A_max; //     - Maximum Acceleration

      // Values to Store for Faster, Smoother Trajectory Execution
      float t_ramp; //  - Time of Velocity Ramp Up (under const. acceleration)
      float t_T; //     - Total Time for Trajectory to be Executed
  } TrapezoidalProfileData;

  // Initializes the Given Trapezoidal Profile based on the Distance to be
  // Covered, s, the Peak Velocity, Vp, and the Maximum Acceleration, Am
  void Init_TrapezoidalProfile(TrapezoidalProfileData* trap,
                               float s, float Vp, float Am){
    trap->dist = s;
    trap->V_peak = Vp;
    trap->A_max = Am;
    // Duration of the constant acceleration, a_max, ramp up/down:
    trap->t_ramp = Vp / Am;
    // Total duration of the trajectory profile:
    trap->t_T = s/Vp + trap->t_ramp;
    // BUT:
    if(sq(Vp)/Am > s){ // not enough time to reach v_max and come back down
                      //  before reaching dist.
      trap->V_peak = sqrt(Am*s); // Peak attainable velocity
      trap->t_ramp = trap->V_peak/Am;
      trap->t_T = 2.0 * (trap->t_ramp);
    }

  } // #Init_TrapezoidalProfile

  /****
    * Returns the Velocity (Linear or Angular) Necessary to Execute the
    * Trapezoidal Profile Described by the TrapezoidalProfileData, trap, at
    * time at Time from Start t
  ****/
  float control_trap_time(TrapezoidalProfileData* trap, float t){
    float u = 0.0;
    // Determine Control Signal from Position in Velocity Trajectory:
    if(t < 0){
      u = 0.0;
    } else if(t <= trap->t_ramp){
      u = (trap->A_max) * t;
    } else if( (trap->t_T - t) <= trap->t_ramp && (trap->t_T - t) > 0){
      u = (trap->A_max) * (trap->t_ramp - t);
    } else if(t_ramp<t && t<=(trap->t_T - trap->t_ramp)){
      u = trap->V_peak;
    }
    // Account for Reverse Trajectories:
    if(dist < 0){
      u = -u;
    }

    return u;
  } // #control_trap_time

  /****
    * Contains the Data Neccessary for Executing a LinearTrajectory with Direct
    * Motion by:
    *   -Turning to Face the Target Position,
    *   -Driving Directly to the Target Position,
    *   -Turning to Acquire the Target Orientation (end.TH)
  ***/
  typedef struct{
    TrapezoidalProfileData* init_turn;
    TrapezoidalProfileData* drive;
    TrapezoidalProfileData* fin_turn;
  } LinearDirectProfileData;

  // Initializes the Given LinearDirect Profile based on the Reference
  // LinearTrajectory to be Executed while not exceeding the Given
  // Maximum Linear Acceleration, Am, and Angular Acceleration, alm
  void Init_LinearDirectProfile(LinearTrajectory* rlt, float Am, float alm){
    TrapezoidalProfileData t_1, l_1, t_2;

    float travel_ang = atan2( (rlt->end->Y - rlt->start->Y),
                              (rlt->end->X - rlt->start->X) );
    Init_TrapezoidalProfile(&t_1,
                            adel(travel_ang, rlt->start->TH),
                            rlt->om_peak, alm);

    Init_TrapezoidalProfile(&l_1, rlt->s_T, rlt->V_peak, Am);

    Init_TrapezoidalProfile(&t_2,
                            adel(rlt->end->TH, travel_ang),
                            rlt->om_peak, alm);
  } // #Init_LinearDirectProfile
  
  /****
    * Modifies the Given Vector, Vprof, to Contain the Vector Profile (V,om,1)
    * Necessary to Execute the Linear Trajectory, rlt, at Time from Start t with
    * Direction Motion by:
    *   -Turning to Face the Target Position,
    *   -Driving Directly to the Target Position,
    *   -Turning to Acquire the Target Orientation (end.TH)
  ****/
  void control_linTraj_direct(Vector3x1* Vprof, LinearTrajectory* rlt, float t){

  } // #follow_trajectory


#endif // _FEEDFORWARD_H
