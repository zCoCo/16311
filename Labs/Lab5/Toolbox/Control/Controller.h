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
  #define K_TAU 0.50

  // Command the Robot to Follow the Given Trajectory (using only feedforward
  // control)
  void run_linearTrajectory(LinearTrajectory* rlt){
    char isFirstRun; // [bool] Whether this is the First Execution
    long t, dt; // in [ms]
    Vector3x1 u_ffwd; // Feedforward Control Signal

    LinearDirectProfileData ldpd;
    Init_LinearDirectProfile(&ldpd, rlt, MAX_ACCEL, MAX_ALPHA, 0.1);

    isFirstRun = 1;
    t = 0;
    dt = 0;
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
        getControl_ffwd_time(&u_ffwd, &ldpd, (t/1000.0));
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

    // Ensure there's enough data stored for interpolation:
    float loop_delay = (float)(2000.0*COMMAND_READ_DELAY/rlt_fbk_buffer_length);

    //Sliding Fifo Storage for Continuous Data Stream of Command Data for
    //Interpolation.
    Construct_TSFifo(Hist_CommTime, float, rlt_fbk_buffer_length); // [ms]
    Init_TSFifo(Hist_CommTime, rlt_fbk_buffer_length);
    TSF_add(Hist_CommTime, 0.0);

    Construct_TSFifo(Hist_CommVel, float, rlt_fbk_buffer_length); // [m/s]
    Init_TSFifo(Hist_CommVel, rlt_fbk_buffer_length);
    TSF_add(Hist_CommVel, 0.0);

    Construct_TSFifo(Hist_CommOmega, float, rlt_fbk_buffer_length); // [rad/s]
    Init_TSFifo(Hist_CommOmega, rlt_fbk_buffer_length);
    TSF_add(Hist_CommOmega, 0.0);

    static Vector3x1 u_ffwd; // Feedforward Control Signal
    static Vector3x1 u_fbk; // Feedback Control Signal

    TPose P_comm;   // Commanded Position in the World-Frame (where the Robot
                    // should be).

    static LinearDirectProfileData ldpd;
    Init_LinearDirectProfile(&ldpd, rlt, MAX_ACCEL, MAX_ALPHA, 0.5);

    static FeedbackControlData fbkc;
    Init_FeedbackControlData(fbkc, K_TAU);

    static float COMM_X, COMM_Y, COMM_TH;

    isFirstRun = 1;
    t = 0;
    dt = 0;
    while((((float)t)/1000.0) < ldpd.t_T){
      if(isFirstRun==1){
        clearTimer(MotionClock); // Do this here b/c performing first step into
                                //  can take some time.

        COMM_X = rlt->start->X;
        COMM_Y = rlt->start->Y;
        COMM_TH = rlt->start->TH;

        isFirstRun = 0;
      } else{
        dt = time1[MotionClock];
        clearTimer(MotionClock);
        t += dt;

        // Get the velocity the robot should be going now:
        getControl_ffwd_time(&u_ffwd, &ldpd, (t/1000.0));
        TSF_add(Hist_CommVel, u_ffwd.v[0]);
        TSF_add(Hist_CommOmega, u_ffwd.v[1]);
        TSF_add(Hist_CommTime, t);

        // Update Commanded Position (Where the Robot Odometry should See the
        // Robot's Location RIGHT NOW based ONLY ON the feedforward control of
        // THE REFERENCE TRAJECTORY):
        static float Vr, omr; // Linear and Angular Velocity at times
        Vr = u_ffwd.v[0];// interpolate_endref(Hist_CommTime.que, Hist_CommVel.que, (t-COMMAND_READ_DELAY*1000.0));
        omr = u_ffwd.v[1];// interpolate_endref(Hist_CommTime.que, Hist_CommOmega.que, (t-COMMAND_READ_DELAY*1000.0));
        // Mid-Point Algorithm:
        dt = dt * 0.001; // ms -> s
        COMM_TH = COMM_TH + omr * dt/2.0;
        COMM_X = COMM_X + Vr * cos(COMM_TH) * dt;
        COMM_Y = COMM_Y + Vr * sin(COMM_TH) * dt;
        COMM_TH = COMM_TH + omr * dt/2.0;

        Set_TPose(P_comm, COMM_X, COMM_Y, COMM_TH);

        // COMM_X = P_comm.X;
        // COMM_Y = P_comm.Y;
        // COMM_TH = P_comm.TH;

        // Get the Necessary Control Signal to Keep the Robot on Track by Comparing
        // Where Odometry Sees the Robot Now and Where we Should See it Now.
        getControl_fbk_time(&u_fbk, &fbkc, &P_comm);

        moveAt((u_ffwd.v[0]+u_fbk.v[0]), (u_ffwd.v[1]+u_fbk.v[1]));
      } // isFirstRun?

    // CPU Relief & Ensure there's enough data stored for interpolation
    wait1Msec(loop_delay);
    } // loop
  } // #run_linearTrajectory

#endif // _CONTROLLER_H
