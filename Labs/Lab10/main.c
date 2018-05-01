#pragma config(Sensor, S3,     lightSensor,         sensorLightActive)
#pragma config(Sensor, S4,     sonarSensor,         sensorSONAR)
/****
  * Logic for a Simple Sumo Robot which must Push another Robot outside of a white
  * ring while avoiding being pushed out itself.
  *
  * Robot uses a rotating carousel shield around itself to laterally deflect
  * oncoming opponents and to detect whether it is currently in contact with an
  * opponent by measuring changes in its rotation rate.
  *
  * Robot uses SONAR to locate and advance onto opponent.
****/

#include "Toolbox/Util/UtilStack.h"
#include "Toolbox/Math/MathStack.h"
#include "Toolbox/HALs/HAL.h"

#define ShieldMotor motorA

#define RING_DIAMETER (95.0*CM) // Diameter of Sumo Ring

#define ROBOT_DEPTH (26.0*CM) // Depth of Robot along Direction of SONAR Sensor's Normal Vector
#define DISTANCE_SONAR_TO_OUTSIDE (14.0*CM) // Distance from SONAR Sensor to regions Outside of the Robot's Bounds

#define LIGHT_DARK 16 // Median Reading of Light Sensor when Over a Dark Surface
#define LIGHT_BLUE 39 // Median Reading of Light Sensor when Over a Blue Surface
#define LIGHT_WHITE 40 // Minimum Reading of Light Sensor when Over a White Surface
#define LIGHT_UP 11 // Maximum Reading of Light Sensor when 1 Set of Wheels Lifted Up off the Ground




// Deprecated before Implementation:
// #define Lidar motorA
// #define LIDAR_SPEED 30
// #define LIDAR_RANGE 180
// int lidarData[LIDAR_RANGE]; // Distance Data from Lidar. Each index is an angle in degrees.
//                            //  Forward is 0deg, index LIDAR_RANGE/2.
// Deprecated.
// void updateLidar(){
//   int th = nMotorEncoder[Lidar];
//   int d = SensorValue[sonarSensor];
// // todo...
//   if(th < -LIDAR_RANGE/2){
//     motor[Lidar] = LIDAR_SPEED;
//   } else if(th > LIDAR_RANGE/2){
//     motor[Lidar] = -LIDAR_SPEED;
//   }
// } // #update Lidar

// Checks if Robot is Currently in Contact with another Robot by Measuring Shield Rotation Rate.
int inContact(){
  // @TODO
}

float currV = 0.0; // Current Body-Centered Linear-Velocity
float currOm = 0.0; // Current Body-Centered Rotational-Velocity

// Switch Direction of Linear Velocity and Set Speed to spd
void reverseAt(float spd){
  currV = spd * -sgn(TSF_Last(Hist_Vel)); // Oppose /measured/ velocity (in case being pushed)
  if(equal(currV, 0.0, MIN_VEL)){
    currV = -spd; // DeaL with Case of Zero Initial Velocity, chose standard "reverse"
  } // currV ~~ 0
} // #reverseAt

int backingUpFromLine = 0; // Whether Robot is Currently Backing Up away from the White Line
//
int line_clearance_maneuver_started = 0; // Whether the Robot has Started backing Away from the White Line
#define LINE_BACKUP_CLEARANCE 0.15 // Distance to back away from the Border Line before attempting next maneuver, in meters.
float odo_at_line_clearance = 0; // Odometer Path Distance of Robot as Soon as the Line Border was Cleared.
//
int post_line_clearance_rotation_started = 0; // Whether Robot has Begun Rotating in Place after Clearing Line
#define POST_CLEARANCE_ROTATION (135*DEG) // Amount to Rotate after Backing up and Clearing Ring Border
float heading_at_clearance_rotation = 0; // Heading Angle of Robot after Clearing Line

int performing_general_backup = 0; // Whether the Robot is Currently Backing Away from Something (not the border)
float general_backup_targ_dist = 0; // Target Distance for the Robot to Backup, in meters
float odo_at_general_backup_start = 0; // Odometer Reading at Start of a General Backup
void init_general_backup(float targ_dist, float spd){ // Setup the Parameters for a General Backup of Distance, targ_dist, at Speed, spd
  reverseAt(spd);
  currOm = 0.0;
  performing_general_backup = 1;
  general_backup_targ_dist = targ_dist;
  odo_at_general_backup_start = TSF_Last(Hist_Dist);
} // #init_general_backup
#define general_backup_dist_covered (abs(odo_at_general_backup_start - TSF_Last(Hist_Dist)))
#define general_backup_done (general_backup_dist_covered > general_backup_targ_dist)

int performing_general_spin = 0; // Whether the Robot is Currently Spinning
float general_spin_targ_dist = 0; // Target Angular Distance for the Robot to Spin
float heading_at_general_spin_start = 0; // Robot Heading at Start of a General Spin
void init_general_spin(float ang, float spd){ // Setup the Parameters for a General Backup of Angular Delta, ang, at Rotation Speed, spd
  currOm = spd;
  currV = 0.0;
  performing_general_spin = 1;
  general_spin_targ_dist = ang;
  heading_at_general_spin_start = rob_pos_TH;
} // #init_general_spin
#define general_spin_dist_covered (abs(heading_at_general_spin_start - rob_pos_TH))
#define general_spin_done (general_spin_dist_covered > general_spin_targ_dist)

int scanning_for_target = 0; // Whether Target is Currently Being Scanned for
float heading_at_scan_start = 0.0; // Heading of Robot at Start of Scan
int switching_scan_regions = 0; // Whether currently switching between first and second half of scan.
#define SCAN_SPEED (-0.75*MAX_OMEGA) // Rotation Speed of Oppponent Scan
#define SCAN_RANGE 390.0*DEG // Max Angular Range of a Scan for Opponent, in degrees
#define scan_distance (abs(heading_at_scan_start - rob_pos_TH))
#define target_acquired (SensorValue[sonarSensor]*CM < (RING_DIAMETER - ROBOT_DEPTH) && SensorValue[sonarSensor]*CM > DISTANCE_SONAR_TO_OUTSIDE)

// int engaging_target = 0; // Whether Opponent Has been Found and is being Attacked

int currLight = 0; // Current Light Sensor Reading
int currSONAR = 0; // Current SONAR Sensor Reading
                                                                                float DUMB_DEBUG = 0;
task main(){
	init_HAL();
	startTask(odometry);

  // Initialize Shield Motor:
  bFloatDuringInactiveMotorPWM = false;
  nMotorPIDSpeedCtrl[ShieldMotor] = mtrSpeedReg;
  nMotorEncoder[ShieldMotor] = 0;
  motor[ShieldMotor] = 100;

  while(1){ // main loop
  // Loop as Frequently as Possible and Always Start by Checking for Ring Border.
  // ^ This is the top priority of all logic and reason for oddly structured loop.
  // All Other Manuever's are enacted in order of priority to robot's survival then offense
  // **A state of no current activity by higher priority functionalities should be indicated by V=0, om=0;
    currLight = SensorValue[lightSensor];
    currSONAR = SensorValue[sonarSensor];

    if(SensorValue[lightSensor] > LIGHT_WHITE){
      if(!backingUpFromLine){
        // As soon as the White Border is Contacted, Drive Away from It Quickly.
        playTone(350,20);
        playTone(300,10);
        reverseAt(MAX_VEL);
        currOm = 0.0;
        backingUpFromLine = 1;
      }
      line_clearance_maneuver_started = 0; // Reset in case pushed during maneuver
      post_line_clearance_rotation_started = 0; // Reset in case pushed during maneuver
    } else if(backingUpFromLine){

      // No longer over the white border but still backing up
      if(!line_clearance_maneuver_started){ // Just crossed out of white line
        line_clearance_maneuver_started = 1;
        odo_at_line_clearance = TSF_Last(Hist_Dist);
      } else if(abs(odo_at_line_clearance - TSF_Last(Hist_Dist)) > LINE_BACKUP_CLEARANCE){
        // Backed up Enough. Line Clearance Maneuver and Line Backup Complete.
        backingUpFromLine = 0;
        line_clearance_maneuver_started = 0;

        //Start Post-Line Clearance Rotation:
        heading_at_clearance_rotation = rob_pos_TH;
        post_line_clearance_rotation_started = 1;
        currV = 0.0;
        currOm = -0.99*MAX_OMEGA; // Post-Line Clearance Rotation Direction is Controlled by this Sign
      }

    } else if(post_line_clearance_rotation_started){
      if(abs(heading_at_clearance_rotation - rob_pos_TH) > POST_CLEARANCE_ROTATION){
        post_line_clearance_rotation_started = 0;
        currV = 0.0;
        currOm = 0.0;
      }
    } else if(SensorValue[lightSensor] <= LIGHT_UP){
      // If Lifted Up off the Ground, Back Up (fast)
      if(!performing_general_backup){
        init_general_backup(9.0*INCH, 0.8*MAX_VEL); // 3/4 of Robot Size at 80% of Max Speed
      }
    } else{ // Deal with All Other Lower-Priority Functionality Here:

      if(performing_general_backup){
        if(general_backup_done){
          currV = 0.0;
          currOm = 0.0;
          performing_general_backup = 0;
        } // general_backup_done?
      } // performing_general_backup?

      if(performing_general_spin){
        if(general_spin_done){

          currV = 0.0;
          currOm = 0.0;
          performing_general_spin = 0;

          if(scanning_for_target){ // Reset Scan Neutral Point if a General Spin was Required.
            currOm = SCAN_SPEED;
            heading_at_scan_start = rob_pos_TH;
          } // scanning_for_target?

        } // general_spin_done?
      } // performing_general_spin?

      if(currV == 0.0 && currOm == 0.0){ // No Activity by Higher Priority Functionalities (and not currently engaging target)

        if(!scanning_for_target){ // if not scanning for target, start.
          playTone(550,10);
          playTone(500,15);
          playTone(475,10);
          currOm = SCAN_SPEED;
          heading_at_scan_start = rob_pos_TH;
          scanning_for_target = 1;
        }

      } // No Higher Priority Activity?

      if(scanning_for_target){
        if(scan_distance > SCAN_RANGE/2.0){
          if((sgn(currOm) / sgn(SCAN_SPEED)) > 0){ // Currently in first half of scan
            currOm = -0.95*sgn(SCAN_SPEED)*MAX_OMEGA; // Flip Scan Direction Quickly
            switching_scan_regions = 1;
            playTone(400,10);
          } else if(!performing_general_spin && !switching_scan_regions){ // ** NOTE: Potential bug if direction flips but distance stays >SR/2.0 for more than 1 cycle (would trigger general spin early) <- probably fixed
          // Second half of scan finished, flip to 180deg away from central angle of scan and start over
            playTone(400,20);
            playTone(450,15);
            init_general_spin((180.0*DEG - SCAN_RANGE/2.0), -0.95*MAX_OMEGA);
          }
        } else if(scan_distance < (0.1*SCAN_RANGE/2.0) && abs(currOm) > abs(1.1*SCAN_SPEED)){
          // End Rapid Switch Between Scan Direction Near Center of Scan and Start Scaning Second Half at Normal Scan Speed.
          playTone(700,20);
          playTone(1000,10);
          currOm = -1.0*SCAN_SPEED;
          switching_scan_regions = 0;
        }


                                                                                  DUMB_DEBUG = SensorValue[sonarSensor];
        if(target_acquired){
          playTone(700, 40);
          scanning_for_target = 0;
          performing_general_spin = 0; // Mark spin as done in case target was acquired during spin

          currV = MAX_VEL; // Engage Target
          currOm = 0.0;
        } // target_acquired?
      } // scanning_for_target?

    } // Priority Heirarchy

    moveAt(currV,currOm);
    wait1Msec(2); // Task Relief
  } // main loop
} // #main
