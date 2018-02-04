#ifndef _UTIL_STACK_H
#define _UTIL_STACK_H

  #include "TSFifo.h"

  // Misc. Utilities:

  #define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

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
