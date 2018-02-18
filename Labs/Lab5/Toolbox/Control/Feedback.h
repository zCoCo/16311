#ifndef _FEEDBACK_H
#define _FEEDBACK_H

  #include "../Math/Vector.h"
  #include "../Positioning/TPose.h"
  #include "../Positioning/Odometry.h"
  #include "../HALs/HAL.h"

  typedef struct{
    float corrective_time; // - Time Constant for Feedback Convergence
    float corrective_time_recip;// - Reciprocal of Corrective Time Constant
    //float t_last; //          - Time of the Last Call to the Feedback Controller
    //TPose error_last; //      - Pose-Vector of Errors from the Last Iteration
  } FeedbackControlData;

  // Initialize the Feedback Control with the Corrective Time Constant of tau
  #define Init_FeedbackControlData(fbkc, tau) do{ \
    fbkc.corrective_time = tau; \
    fbkc.corrective_time_recip = ((float) 1.0 / ((float) tau)); \
    /*fbkc.t_last = 0.0;*/ \
    /*Init_TPose(error_last, 0,0,0);*/ \
  } while(0)

  /****
   * Sets the Contents of the Given Vector to the Required Control Signal (V,om,1)
   * to Ensure the Robot (according to its Odometry) Stays on its Trajectory by
   * Comparing its Odometry Position with the Interpolated Position, Pcomm, which is
   * where the Robot should have been when the Current Odometry Readings were Taken.
  ****/
  void getControl_fbk_time(Vector3x1* Vprof, FeedbackControlData* fbkc, TPose* Pcomm){
    static float k_x, k_y, k_th, V;
    k_x = fbkc->corrective_time_recip; // Don't Make Constant to Allow Active-Tuning
    k_th = fbkc->corrective_time_recip;
    V = TSF_Last(Hist_Vel); // Should be Interpolated for Collection Time but this is
                          //   Close Enough if dt is not large (more than 10ms or so).
    if(V > 1.5 * MIN_VEL){
      k_y = 2.0 * sq(fbkc->corrective_time_recip) / V;
    } else{ // Below V-floor. k_y will be too large
      k_y = 0.0;
    }

    float wrp_x, wrp_y; // Position of Target Point, P, Relative to Robot in World-Frame
    wrp_x = (Pcomm->X - rob_pos_X);
    wrp_y = (Pcomm->Y - rob_pos_Y);

    float ex, ey, eth; // Position of Target Point, P, Relative to Robot in Robot-Frame
    ex = cos(rob_pos_TH) * wrp_x + sin(rob_pos_TH) * wrp_y;
    ey = cos(rob_pos_TH) * wrp_y - sin(rob_pos_TH) * wrp_x;
    eth = adel(Pcomm->TH, rob_pos_TH);

    Vprof->v[0] = ex*k_x; // u_v
    Vprof->v[1] = ey*k_y + eth*k_th; // u_om
    Vprof->v[2] = 1;
  } // #getControl_t

#endif // _FEEDBACK_H
