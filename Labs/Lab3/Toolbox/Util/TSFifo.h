#ifndef _T_SLIDING_FIFO_H
#define _T_SLIDING_FIFO_H

#include "../Util/UtilStack.h"

// Implements a Sliding FiFo Queue for Limited Storage of Continuous Data Streams
#define Construct_TSFifo(name, type, maxSize) \
typedef struct{ \
  type que[maxSize]; \
  int numElements; \
  int maxElements; \
} name ## _TSF; \
name ## _TSF name

// Must be called in an executable area (ex. task):
// maxSize should be consistent with maxSize given in #Construct_TSFifo
#define Init_TSFifo(name, maxSize) do{ \
  name.numElements = 0; \
  name.maxElements = maxSize; /*= ARRAY_SIZE(name.que) // Doesn't work with *[] */ \
} while(0)

#define TSF_add(tsf, elem) do{ \
  if(tsf.maxElements != 0){ \
    if(tsf.numElements == tsf.maxElements){ \
      for(int i=0; i<(tsf.maxElements-1); i++){ \
        tsf.que[i] = tsf.que[i+1]; \
      } \
      tsf.que[tsf.maxElements-1] = elem; \
    } else{ \
      tsf.que[tsf.numElements] = elem; \
      tsf.numElements = tsf.numElements + 1; \
    } \
  } \
} while(0)

#define TSF_first(tsf) ((tsf.numElements>0) ? (tsf.que[0]) : 0)

// TSF_Last is unsafe if no values have been added (numElem=0). Has to be this
// way for accessors (.X, .Y) to work.
#define TSF_last(tsf) (tsf.que[tsf.numElements-1]) // Make SURE you have ADDED A VALUE to the fifo!
#define TSF_Last(tsf) TSF_last(tsf) //alias

#define TSF_penult(tsf) ((tsf.numElements>1) ? (tsf.que[tsf.numElements-2]) : 0) // Second to Last

#define TSF_delta(tsf) (TSF_last(tsf) - TSF_penult(tsf))
#define TSF_Delta(tsf) TSF_delta(tsf) //alias


#define TSF_isEmpty(tsf) (tsf.numElements == 0)

#endif // _T_SLIDING_FIFO_H
