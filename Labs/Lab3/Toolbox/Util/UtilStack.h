#ifndef _UTIL_STACK_H
#define _UTIL_STACK_H

  #include "SlidingFifo.h"

  // Misc. Utilities:

  /*********************************************************
   * Function that judges if two floats are effectively equal
   *********************************************************/
   bool equal(float a, float b, float epsilon) {
     if (abs(a-b) < epsilon) {
       return true;
     } else {
       return false;
     }
   }

#endif // _UTIL_STACK_H
