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

  // Set Contents of TPose to (x,y,t)
  #define Set_TPose(P, x,y,t) do{ \
    P.X = (x); P.Y = (y); P.TH = (t); \
  } while(0)

  // Initialize TPose, P, with the Given Position (x,y,t)
  #define Init_TPose(P, x,y,t) Set_TPose(P, x,y,t)

  // Copies the Contents of Pa into Pb
  #define Copy_TPose(Pb, Pa) do{ \
    Pb.X = Pa.X; Pb.Y = Pa.Y; Pb.TH = Pa.TH; \
  } while(0)

  // Copies the TPose Contents Pointed to by PPa into Pb
  #define Copy_PTPose(Pb, PPa) do{ \
    Pb.X = PPa->X; Pb.Y = PPa->Y; Pb.TH = PPa->TH; \
  } while(0)

  /****
   * Returns the Homogeneous Transform that Converts Coordinates from the b Frame
   * to the a Frame of the Given Pose.
  ****/
  void bToA(TPose &P, Matrix3x3 &mat){

    mat.m[0][0] = cos(P.TH); mat.m[0][1] = -sin(P.TH); mat.m[0][2] = P.X;
    mat.m[1][0] = sin(P.TH); mat.m[1][1] = cos(P.TH); mat.m[1][2] = P.Y;
    mat.m[2][0] = 0.0; mat.m[2][1] = 0.0; mat.m[2][2] = 1.0;

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
  void addTo(TPose* Delta, TPose* P, TPose* Pres){
    Matrix3x3 Mb2a;
    bToA(*P, Mb2a);
    Vector3x1 Vr;
    Vr.v = Delta->vec; Vr.v[2] = 1.0;

    MultMatVec(Mb2a,Vr, Pres->vec);

    Pres->TH = P->TH + Delta->TH;
    Pres->TH = atan2(sin(Pres->TH), cos(Pres->TH)); // Ensure Proper Wrap-Around
  } // #addTo

#endif // _T_ROBOT_POSITION_H
