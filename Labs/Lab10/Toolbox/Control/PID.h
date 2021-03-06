#ifndef _PID_H
#define _PID_H

  #include "../Util/TSFifo.h"
  #include "../Math/ArrayMath.h" // Needs SUM_ARRAY

  #define PIDClock T2

  #ifndef PID_STREAM_LENGTH //Allow External Settings to Override this Default
    #define PID_STREAM_LENGTH 10 /* Length of Primary Data Streams for PID*/
    #define PSL PID_STREAM_LENGTH /*alias*/
  #endif
  Type_TSFifo(PIDStream, float, PSL); // Data Type Used for All PID Data Streams

  typedef struct{
    float *target; //           - Target Value to Achieve
    float *input; //            - Input Value being Controlled
    PIDStream *hist_error; //   - Log of Error Values Controlled
    float last_error; //        - Error Value on Last Signal Computation

    float max; //               - Maximum Value for Control Signal

    float k_p; //               - Proportional Parameter
    float k_d; //               - Differential Parameter
    float k_i; //               - Integral Parameter

    float sum; //               - Sum of All Errors over Time (~=Integral)

    // float last_time;//          - [ms] Last Time the Controller Was Called
  } PIDController;

  // Initialize the PID Controller to Vary the Input Parameter pointed to by pIn,
  // to Achieve the Target value at pTarg, Driven by the Error Log, pErr, with
  // the Given Parameters kp, kd, ki. Ensures control signal remains below m
  #define Init_PIDController(pidc, pIn, pErr, pTarg, kp, kd, ki, m) do{ \
    pidc.input = pIn; \
    pidc.hist_error = pErr; \
    pidc.last_error = 0.0; \
    pidc.target = pTarg; \
    \
    pidc.k_p = kp; \
    pidc.k_d = kd; \
    pidc.k_i = ki; \
    \
    pidc.max = m; \
    \
    pidc.sum = 0.0; \
  } while(0)

  // Returns the Required Control Signal to Move the given PIDController's Input
  // towards its Target.
  float getControl(PIDController* pidc){
    static float e_p = 0.0; // Static Variables for External Viewing
    static float e_i = 0.0;
    static float e_d = 0.0;

    static float sig = 0.0;

    if(pidc->target != NULL && pidc->input != NULL && pidc->hist_error != NULL){ // Prevent Out-of-Sync Error
      // Deal with Current Error:
      e_p = *(pidc->target) - *(pidc->input); // Compute Error (DO NOT MAKE STATIC - or any not necessarily init'd ptr math)
      float e_p_pass = e_p; // Why?
      TSFP_add(pidc->hist_error, e_p_pass); // Update Error Logs

      pidc->sum += e_p;
      e_i = pidc->sum; // Dump to static e_i to display value
      e_d = e_p - pidc->last_error; // Derivative
      pidc->last_error = e_p; // Reset

      // Return Control Signal:
      sig = ( pidc->k_p * e_p + pidc->k_i * e_i + pidc->k_d * e_d );
    }
    return (abs(sig) < pidc->max ? sig : ((sig/abs(sig)) * pidc->max));
  }
/**/

#endif // _PID_H
