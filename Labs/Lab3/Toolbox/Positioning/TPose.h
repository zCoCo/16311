#ifndef _T_POSE_H
#define _T_POSE_H

#include "../Math/MathStack.h";

  #define POSITION_DIMENSIONS 3;
  typedef union{
    struct{
      float X; //   - X Position
      float Y; //   - Y Position
      float TH; //  - Heading (World-Frame)
    };
    float vec[POSITION_DIMENSIONS];
  } TPose;

  TPose Robot_Pose; // Robot Pose in the World Frame

  /****
   * Adds the Contents of the Given Robot Pose Delta to the Contents of Pose P.
  ***/
  void addTo(TPose &Delta, TPose &P){
    P.X = P.X + Delta.X;
    P.Y = P.Y + Delta.Y;
    P.TH = P.TH + Delta.TH
    P.TH = atan2(sin(P.TH), cos(P.TH)); // Ensure Proper Wrap-Around
  }

#endif // _T_ROBOT_POSITION_H
