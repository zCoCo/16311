#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

  #include "../Math/MathStack.h"

  /* Defines a Linear Trajectory from a Given Starting Pose to a Given Terminal
  Pose (both in the World Frame) with the Given Peak Linear and Angular Velocities */
  typedef struct{
    TPose* start;
    TPose* end;

    float V_peak;
    float om_peak;

    float s_T; // Total Translational Distance Covered (computed)
  } LinearTrajectory;

  #define Init_LinearTrajectory(lt, st,e, Vp,omp) do{ \
    lt.start = st; \
    lt.end = e; \
    lt.V_peak = Vp; \
    lt.om_peak = omp; \
    \
    lt.s_T = sqrt( sq(e.X - st.X) + sq(e.Y - st.Y) ); \
  } while(0)

  /* Defines a CompoundTrajectory Composed of a Series of LinearTrajectories */
  #define Construct_CompoundTrajectory(name, numElem) \
    typedef struct{ \
      LinearTrajectory* traj[numElem]; \
      int numElements; \
    } CompoundTrajectory_ ## numElem;

  #define Init_CompoundTrajectory(name, numElem) do{ \
    name.numElements = numElem; \
    /* Initialize Other Parameters */ \
  } while(0)

#endif // _TRAJECTORY_H
