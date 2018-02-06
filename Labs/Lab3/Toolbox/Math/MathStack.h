#ifndef _MATH_STACK_H
#define _MATH_STACK_H

// CONSTANTS/CONVERSIONS:
  #define DEG 0.017453 /* ex. 180*deg = 3.14159 */

  #include "Vector.h"
  #include "Matrix.h"

  // Misc Functions:

  // Square a Number:
  #define sq(v) (v * v)

  // Compute the Angular Difference between Two Angles:
  // Ex: Computing the necessary turn angle, t, to go from world-frame
  // orientation O1 to world-frame orientation O2 would be:
  // t = adel(O2, O1)
  float adel(a, b){
    float Dth = (a - b);
    Dth = atan2(sin(Dth),cos(Dth));
   return Dth;
  } // #adel

#endif // _MATH_STACK_H
