#ifndef _ARRAY_MATH_H
#define _ARRAY_MATH_H

  #include "../Util/ArrayTools.h" // Needs ARRAY_SIZE
  // Sum the Contents of the Given Array, a, puts the result in the variable
  // pointed to by pRes.
  #define SUM_ARRAY(a, pRes) do{ \
    int len = ARRAY_SIZE(a); \
    *(pRes) = 0.0; \
    for(int i=0; i<len; i++){ \
      *(pRes) += a[i]; \
    } \
  } while(0)

  #include "../Util/ArrayTools.h" // Needs ARRAY_SIZE
  // Average the Contents of the Given Array, a, puts the result in the variable
  // pointed to by pRes.
  #define AVG_ARRAY(a, pRes) do{ \
    int len = ARRAY_SIZE(a); \
    *(pRes) = 0.0; \
    for(int i=0; i<len; i++){ \
      *(pRes) += a[i]; \
    } \
    *(pRes) = ((float)(*(pRes))) / ((float)(len)); \
  } while(0)

#endif // _ARRAY_MATH_H
