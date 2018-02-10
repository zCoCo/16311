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
    float sa = abs(s);
    trap->V_peak = Vp;
    trap->A_max = Am;
    // Duration of the constant acceleration, a_max, ramp up/down:
    trap->t_ramp = Vp / Am;
    // Total duration of the trajectory profile:
    trap->t_T = sa/Vp + trap->t_ramp;
    // BUT:
    if(sq(Vp)/Am > sa){ // not enough time to reach v_max and come back down
                      //  before reaching dist.
      trap->V_peak = sqrt(Am*sa); // Peak attainable velocity
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
    float u;
    u = 0.0;
    // Determine Control Signal from Position in Velocity Trajectory:
    if(t < 0){
      u = 0.0;
    } else if(t <= trap->t_ramp){
      u = (trap->A_max) * t;
    } else if( (trap->t_T - t) <= trap->t_ramp && (trap->t_T - t) > 0){
      u = (trap->A_max) * (trap->t_T - t);
    } else if(trap->t_ramp<t && t<=(trap->t_T - trap->t_ramp)){
      u = trap->V_peak;
    }
    // Account for Reverse Trajectories:
    if(trap->dist < 0){
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
    TrapezoidalProfileData init_turn;
    TrapezoidalProfileData drive;
    TrapezoidalProfileData fin_turn;
    float t_buff; //  -Buffer Time between Each Motion
    float t_T; //     -Total Time for Execution of this trajectory
  } LinearDirectProfileData;

  // Initializes the Given LinearDirect Profile based on the Reference
  // LinearTrajectory, rlt, to be Executed while Not Exceeding the Given
  // Maximum Linear Acceleration, Am, and Angular Acceleration, alm, and
  // Including the Specified Buffer Time, tb, after Each Step.
  void Init_LinearDirectProfile(LinearDirectProfileData* ldpd,
                                LinearTrajectory* rlt,
                                float Am, float alm,
                                float tb){
    float travel_ang = atan2( (rlt->end->Y - rlt->start->Y),
                              (rlt->end->X - rlt->start->X) );
    float th = adel(travel_ang, rlt->start->TH);
    Init_TrapezoidalProfile(&(ldpd->init_turn),
                            th, rlt->om_peak, alm);

    Init_TrapezoidalProfile(&ldpd->drive, rlt->s_T, rlt->V_peak, Am);

    Init_TrapezoidalProfile(&ldpd->fin_turn,
                            adel(rlt->end->TH, travel_ang),
                            rlt->om_peak, alm);

    ldpd->t_buff = tb;
    ldpd->t_T = ldpd->init_turn.t_T
              + ldpd->drive.t_T
              + ldpd->fin_turn.t_T
              + 3.0*tb;
  } // #Init_LinearDirectProfile

  // --- Overloaded Methods ---
  // Set of Methods to Get the Control Signal (V,om,1) for the Given Time since
  // Start dt - Overloaded for Each Trajectory Profile Data Type (ex.
  // LinearDirectProfileData), b/c allows for creating single macro for
  // controlling any trajectory
  /****
    * Modifies the Given Vector, Vprof, to Contain the Vector Profile (V,om,1)
    * Necessary to Execute the Linear Trajectory, rlt, at Time from Start t with
    * Direction Motion by:
    *   -Turning to Face the Target Position,
    *   -Driving Directly to the Target Position,
    *   -Turning to Acquire the Target Orientation (end.TH)
  ****/
  // Absolute End Times of Each Move:
  #define t_1_Tf (ldpd->init_turn.t_T)
  #define l_1_Tf (t_1_Tf + ldpd->t_buff + ldpd->drive.t_T)
  #define t_2_Tf (l_1_Tf + ldpd->t_buff + ldpd->fin_turn.t_T)
  void getControl_ffwd_time(Vector3x1* Vprof,
                            LinearDirectProfileData* ldpd,
                            float t){
    static float V, om; // Keep Allocati

    if(t < t_1_Tf){
      V = 0.0;
      om = control_trap_time(ldpd->init_turn, t);
    } else if(t > (t_1_Tf + ldpd->t_buff) && t < l_1_Tf){
      V = control_trap_time(ldpd->drive, (t-(ldpd->t_buff)-t_1_Tf));
      om = 0.0;
    } else if(t > (l_1_Tf + ldpd->t_buff) && t < t_2_Tf){
      V = 0.0;
      om = control_trap_time(ldpd->fin_turn, (t-(ldpd->t_buff)-l_1_Tf));
    } else{
      V = 0.0; om = 0.0;
    }

    Vprof->v[0] = V;
    Vprof->v[1] = om;
    Vprof->v[2] = 1;
  } // #getControl_ffwd_time

#endif // _FEEDFORWARD_H
