#ifndef _MATH_STACK_H
#define _MATH_STACK_H

  #include "Units.h"
  #include "Vector.h"
  #include "Matrix.h"

/**** --- MISC. FUNCTIONS --- ****/

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

  // Find the Sign of a Number:
  #define sgn(x) ((x) / abs(x))

  // Find the Minimum of Two Numbers.
  // * Uses define so it works with any or mixed number types (without casting)
  #define min(a,b) (((a)<(b)) ? (a) : (b))

  // Square a Number:
  //#define sq(v) (((v) * (v)))
  float sq(float a){ return a*a; }

  // Compute the Angular Difference between Two Angles:
  // Ex: Computing the necessary turn angle, t, to go from world-frame
  // orientation O1 to world-frame orientation O2 would be:
  // t = adel(O2, O1)
  float adel(float a, float b){
    float Dth = (a - b);
    Dth = atan2(sin(Dth),cos(Dth));
   return Dth;
  } // #adel



#endif // _MATH_STACK_H
