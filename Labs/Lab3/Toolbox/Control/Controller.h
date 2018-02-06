#ifndef _CONTROLLER_H
#define _CONTROLLER_H

  #include "../Util/UtilStack.h"
  #include "../Planning/Trajectory.h"
  #include "Feedforward.h"
  #include "Feedback.h"
  #include "../Positioning/TPose.h"
  #include "../Positioning/Odometry.h"
  #include "../HALs/HAL.h" /* Needs MAX_VEL, MAX_ACCEL, MAX_OMEGA, MAX_ALPHA,
                                    COMMAND_DELAY, COMMAND_READ_DELAY */

  // Clock Used for Executing Motions
  #define MotionClock T2

  // Feedback Convergence Time Constant
  #define K_TAU 1.2

  // Command the Robot to Follow the Given Trajectory (using only feedforward
  // control)
  void run_linearTrajectory(LinearTrajectory* rlt){
    static float HEY_LOOK_AT_ME;
    char isFirstRun; // [bool] Whether this is the First Execution
    long t, dt; // in [ms]
    Vector3x1 u_ffwd; // Feedforward Control Signal

    LinearDirectProfileData ldpd;
    Init_LinearDirectProfile(&ldpd, rlt, MAX_ACCEL, MAX_ALPHA, 0.5);

    isFirstRun = 1;
    t = 0;
    dt = 0;
    HEY_LOOK_AT_ME = ldpd.t_T;
    wait1Msec(100);
    while((((float)t)/1000.0) < ldpd.t_T){
      if(isFirstRun==1){
        clearTimer(MotionClock); // Do this here b/c performing first step into
                                //  can take some time.
        isFirstRun = 0;
      } else{
        dt = time1[MotionClock];
        clearTimer(MotionClock);
        t += dt;

        // Get the velocity the robot should have by the time it will be
        // executed (avoids doing data-log interpolation over time)
        // ~= how fast should the robot be going by the time it gets this command
        getControl_ffwd_time(&u_ffwd, &ldpd, (t/1000.0 + COMMAND_DELAY));

        moveAt(u_ffwd.v[0], u_ffwd.v[1]);
      } // isFirstRun?

      wait1Msec(VELOCITY_UPDATE_INTERVAL); // CPU Relief
    } // loop
  } // #run_linearTrajectory

  // Command the Robot to Follow the Given Trajectory (using feedforward and
  // feedback control)
  #define rlt_fbk_buffer_length 25 /* Number of Entries to Log for Interpolation */
  void run_linearTrajectory_fbk(LinearTrajectory* rlt){
    char isFirstRun; // [bool] Whether this is the First Execution
    long t, dt; // in [ms]

    //Sliding Fifo Storage for Continuous Data Stream of Command Data for
    //Interpolation.
    Construct_TSFifo(Hist_CommTime, long, rlt_fbk_buffer_length); // [ms]
    Init_TSFifo(Hist_CommTime, rlt_fbk_buffer_length);
    TSF_add(Hist_CommTime, 0);

    Construct_TSFifo(Hist_CommVel, float, rlt_fbk_buffer_length); // [m/s]
    Init_TSFifo(Hist_CommVel, rlt_fbk_buffer_length);
    TSF_add(Hist_CommVel, 0.0);

    Construct_TSFifo(Hist_CommOmega, float, rlt_fbk_buffer_length); // [rad/s]
    Init_TSFifo(Hist_CommOmega, rlt_fbk_buffer_length);
    TSF_add(Hist_CommOmega, 0.0);

    Vector3x1 u_ffwd; // Feedforward Control Signal
    Vector3x1 u_fbk; // Feedback Control Signal

    TPose P_comm;   // Commanded Position in the World-Frame (where the Robot
                    // should be).
    Copy_TPose(P_comm, (*(rlt->start)));

    LinearDirectProfileData ldpd;
    Init_LinearDirectProfile(&ldpd, rlt, MAX_ACCEL, MAX_ALPHA, 0.5);

    isFirstRun = 1;
    t = 0;
    dt = 0;
    while(t < ldpd.t_T){
      if(isFirstRun==1){
        clearTimer(MotionClock); // Do this here b/c performing first step into
                                //  can take some time.
        isFirstRun = 0;
      } else{
        dt = time1[MotionClock];
        clearTimer(MotionClock);
        t += dt;

        // Get the velocity the robot should be going now:
        getControl_ffwd_time(&u_ffwd, &ldpd, t/1000.0);
        TSF_add(Hist_CommVel, u_ffwd.v[0]);
        TSF_add(Hist_CommOmega, u_ffwd.v[1]);
        TSF_add(Hist_CommTime, t);

        // Update Commanded Position (where the robot should the Odometry See
        // the Robot to be RIGHT NOW based ONLY ON the feedforward control of
        // THE REFERENCE TRAJECTORY):
        float Vr, omr; // Linear and Angular Velocity at times
        interpolate_endref(Hist_CommTime.que, Hist_CommVel.que, (t-COMMAND_READ_DELAY*1000.0), &Vr);
        interpolate_endref(Hist_CommTime.que, Hist_CommOmega.que, (t-COMMAND_READ_DELAY*1000.0), &omr);
        // Mid-Point Algorithm:
        P_comm.TH = P_comm.TH + omr * dt/2.0;
        P_comm.X = P_comm.X + Vr * cos(P_comm.TH) * dt;
        P_comm.Y = P_comm.Y + Vr * sin(P_comm.TH) * dt;
        P_comm.TH = P_comm.TH + omr * dt/2.0;

        // Get the Necessary Control Signal to Keep the Robot on Track by Comparing
        // Where Odometry Sees the Robot Now and Where we Should See it Now.
        FeedbackControlData fbkc;
        Init_FeedbackControlData(fbkc, K_TAU);
        getControl_fbk_time(&u_fbk, &fbkc, &P_comm);

        moveAt((u_ffwd.v[0]+u_fbk.v[0]), (u_ffwd.v[1]+u_fbk.v[1]));
      } // isFirstRun?

    // CPU Relief & Ensure there's enough data stored for interpolation
    wait1Msec((float)(2000.0*COMMAND_READ_DELAY/rlt_fbk_buffer_length));
    } // loop
  } // #run_linearTrajectory

#endif // _CONTROLLER_H
