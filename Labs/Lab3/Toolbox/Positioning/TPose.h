#ifndef _T_POSE_H
#define _T_POSE_H

#include "../Math/Matrix.h";
#include "../Math/Vector.h";

  typedef union{
    struct{
      float X; //   - X Position
      float Y; //   - Y Position
      float TH; //  - Heading, Radians (World-Frame)
    };
    Vector3x1 vec;
  } TPose;

  TPose Robot_Pose; // Robot Pose in the World Frame

  /****
   * Returns the Homogeneous Transform that Converts Coordinates from the b Frame
   * to the a Frame of the Given Pose.
  ****/
  Matrix3x3 bToA(TPose &P){
    Matrix3x3 mat;

    mat.m[0][0] = cos(P.TH); mat.m[0][1] = -sin(P.TH); mat.m[0][2] = P.X;
    mat.m[1][0] = sin(P.TH); mat.m[1][1] = cos(P.TH); mat.m[1][2] = P.Y;
    mat.m[2][0] = 0.0f; mat.m[2][1] = 0.0f; mat.m[2][2] = 1.0f;

    return mat;
  } // #bToA

  /****
   * Adds the Vector Pose Delta (in the robot-frame) to the Given World Pose P
   * - returns result, does not modify P.
   *
   * Essentially, this represents what would happen if a robot followed a line
   * oriented as the pose vector {x,y] of Delta in it's LOCAL Frame and turned
   * to face the th of Delta in its LOCAL Frame from the start of the motion.
   *
   * Ex: If P is [1,1,45deg] adding [1,1,15deg] would NOT produce [2,2,60deg] BUT
   * [1,2.414,60deg].
   *
   * More Examples:
   * Add D=[1,2,0] to P=[0,0,0] -> [1,2,0]
   * Add D=[1,2,0] to P=[0,0,90deg] -> [-2,1,90deg]
   * Add D=[1,2,-90deg] to P=[0,0,90deg] -> [-2,1,0]
   *
   * Add D=[1,0,0] to P=[0,0,45deg] -> [cos(45deg),sin(45deg),45deg]
   * Add D=[1,0,45deg] to P=[0,0,45deg] -> [cos(45deg),sin(45deg),90deg]
   *
   * Add D=[1,0,310deg] to P=[0,0,60deg] -> [cos(60deg),sin(60deg),10deg]
  ***/
  TPose addTo(TPose &Delta, TPose &P){
    TPose Pres;

    Matrix3x3 Mb2a = bToA(P);
    Vector3x1 Vr;
    Vr.v = Delta.v; Vr.v[2] = 1.0f;

    Pres.v = MultMatVec(Mb2a,Vr);

    Pres.TH = P.TH + Delta.TH;
    Pres.TH = atan2(sin(Pres.TH), cos(Pres.TH)); // Ensure Proper Wrap-Around
  } // #addTo

#endif // _T_ROBOT_POSITION_H
