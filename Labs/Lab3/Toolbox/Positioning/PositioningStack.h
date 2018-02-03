#ifndef _POSITIONING_STACK_H
#define _POSITIONING_STACK_H

/********* --- NOTE ON ALL COORDINATES USED --- *****

  Positive X is Forward Relative to the Robot (Alongtrack).
  Positive Y is Left Relative to the Robot (Crosstrack).
  Positive TH is Counter Clockwise (CCW).

  * This holds true for world coordinates too, so, if the robot starts at the origin
  * [0,0,0] pointing forward in world coordinates and moves forward [1,0,0] along
  * IT'S X-axis, it also forward along the X-axis in the world frame (NOT the Y-axis
  * as one might initially assume).

                           .^ x+
                        .   |
                      .     |
                    |_ th+  |
                y+  < - - - + - - - y-
                            |
                            |
                            | x-

***********/

  #include "TPose.h"

#endif // _POSITIONING_STACK_H
