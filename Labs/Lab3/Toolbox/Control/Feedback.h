#ifndef _FEEDBACK_H
#define _FEEDBACK_H

  #include "../Math/Vector.h"
  #include "Odometry.h"

  #define COMMAND_DELAY 0.05f //  -Delay, in sec, a Velocity been Commanded and
                            //     it being Implemented and Read by Odometry.

  /****
   * Returns a Vector Containing the Required Control Signal [u_v, u_w, 1]
   * to Ensure the Robot (according to its Odometry) Stays on the Given Trajectory
   * at the Global Time, t.
  ****/
  Vector3x1 getControl_t(){
    
  } // #getControl_t

#endif // _FEEDBACK_H
