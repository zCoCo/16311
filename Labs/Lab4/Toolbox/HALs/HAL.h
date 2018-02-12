#ifndef _HAL_H
#define _HAL_H
  // Include this file wherever HAL functionality (access to physical
  // data/control on the robot without referencing a specific geometry) is
  // needed.
  /****
    Universally Supported HAL Functionality:
      PARAMETERS:
        -MAX_VELOCITY
        -MAX_ACCEL
        -MAX_OMEGA
        -MAX_ALPHA
        -COMMAND_DELAY
        -COMMAND_READ_DELAY
      CORE:
        -Initialize:
        #init_HAL()

        -Reset (effectively, make robot init again as if power had been toggled):
        #reset_HAL()

      SENSING:
        -Odometry - Measures Characteristics about the Absolute Distance Travelled.

      MOTION:
        -Make the Robot's Center Move with the Given Path and Rotational Velocity:
        #moveAt(Velocity, Omega)

  ****/

  // General NXT Parameters (first, used by HAL):
  #define TICKS_PER_REV 360.0

  //#include "HAL_Lab3_WMR.h"; //   - Lab 3 Robot Form Factor
  #include "HAL_Lab4_Tower.h"; // - Lab 4 Robot Form Factor

#endif // _HAL_H
