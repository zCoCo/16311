#ifndef _PID_H
#define _PID_H

  #include "../Util/TSFifo.h"

  #define PIDClock T2

  typedef struct{
    float* target; //           - Target Value to Achieve
    TSFifo* hist_input; //      - Log of Input Values Controlled
    TSFifo* hist_error; //      - Log of Error Values Controlled

    float k_p; //               - Proportional Parameter
    float k_d; //               - Differential Parameter
    float k_i; //               - Integral Parameter

    float last_time;//          - [ms] Last Time the Controller Was Called
  } PIDController;

  // Initialize the PID Controller to Vary the Input Parameter pointed to by pIn,
  // to Achieve the target value at pTarg with the Given Parameters kp, kd, ki
  #define Init_PIDData(pidc, pIn, pTarg, kp, kd, ki) do{ \
    pidc.input = pIn; \
    pidc.target = pTarg; \
    \
    pidc.k_p = kp; \
    pidc.k_d = kd; \
    pidc.k_i = ki; \

    clearTimer(PIDClock);
  } while(0)

  // Returns the Required Control Signal to Move the given PIDController's Input
  // towards its Target.
  float getControl(PIDController* pidc){
    // Update Timer:
    float dt = pidc->last_time - time1[PIDClock]; // [ms] Time Elapsed since Last Update
    pidc->last_time += dt;

    // Deal with Current Error:
    float e_p = *(pidc->target) - TSFP_Last(pidc->hist_input); // Compute Error
    TSFP_add(pidc->hist_error, e_p); // Update Error Logs

    // Retrieve Integral, and Derivative of Error:
    // N.B. Not quite an Integral since each element is not multiplied by its own
    // dt BUT since each dt should be similar, this shouldn't be too bad unless there's
    // some sort of clock stall due to a CPU hog but, in that case, there would be
    // bigger bugs to fry.
    float e_i = SUM_ARRAY(pidc->hist_error->que) * dt; // Only Sums Recent Values (.: Automatic purge)
    float e_d = ((float) TSFP_Delta(pidc->hist_error)) / dt;

    // Return Control Signal:
    return ( pidc->k_p * e_p + pidc->k_i * e_i + pidc->k_d * e_d );
  }


#endif // _PID_H
