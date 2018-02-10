#ifndef _UTIL_STACK_H
#define _UTIL_STACK_H

  #include "TSFifo.h"
  #include "ArrayTools.h"

  // Misc. Utilities:

  #include "../Math/MathStack.h" // Needs #min
  // Use Linear Interpolation to Determine the Value in vs Array which
  // Corresponds to the Given Value x in the xs and stores it in the given
  // pointer vp.
  // ** Array xs must be sorted in ascending order.
  // Endref starts the search at the ends of both arrays, if one array is ends
  // before an interpolation region is found, the value for vs at the index
  // before the runout is given.
  // * Uses a define so it works for arrays of any and even mixed number types
  #define interpolate_endref(xs, vs, x, vp) do{ \
    int len_vs = ARRAY_SIZE(vs); \
    int len_xs = ARRAY_SIZE(xs); \
    int n = min(len_vs, len_xs); \
    float *vptr = vp; \
    if(x > xs[len_xs-1]){ \
      *(vptr) = vs[len_vs-1]; \
    } else{ \
      int i=0; \
      while(i<n){ \
        /* Min is found Interpolation Region Found when x is greater than value: */ \
        if(x > xs[len_xs-1-i]){ \
          *(vptr) = vs[len_vs-1-i] + \
          (x - xs[len_vs-1-i]) \
          * (vs[len_vs-i] - vs[len_vs-1-i]) / (xs[len_vs-i] - xs[len_vs-1-i]); \
          i = n; /*end loop*/ \
        } \
       i++; \
      } \
      if(i >= n){ \
        *(vptr) = vs[len_vs-n]; \
      } \
    } \
  } while(0)

#endif // _UTIL_STACK_H
