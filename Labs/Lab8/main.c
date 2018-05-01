#pragma config(Sensor, S3,     lightSensor,         sensorLightActive)

#define LeftMotor motorC
#define RightMotor motorB

// Whether performing a Real Test (and, thus, Stopping at White Rung)
#define THIS_IS_NOT_A_DRILL 1

long BATT = nAvgBatteryLevel;

long run_vel = 48; // Changeable via Debug
int light = 0; // Light Sensor Reading used for Positioning (recorded once per lift)
int light_inst = 0; // Instantaneous Light Sensor Reading

// Number of Encoder Ticks (degrees) per Lift:
#define LIFT_DEG 360

// Allow for Arms to Start Off the Bar (after a pull has been completed):
#define START_OFFSET 45
float last_enc = 20 - START_OFFSET; // Use Left Motor Encoder for "Odometry" (force read on first run)

int rung_idx = 1; // Rung the Robot is Resting on

#define WHITE_THRESH 39

task main(){
  // Tell Motors to Operate Closed-Loop (being able to command a specific number
  // of ticks per second:)
  nMotorPIDSpeedCtrl[LeftMotor] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[RightMotor] = mtrSpeedReg;
  nMaxRegulatedSpeedNxt = 720;
  nPidUpdateInterval = 2;

  // Initialize Sensor Deltas:
  nMotorEncoder[LeftMotor] = 0;
  nMotorEncoder[RightMotor] = 0;

  if(SensorValue[lightSensor] < WHITE_THRESH){ // Ensure Not Starting on White

    int cont = 1; // Continue
    while(cont){
      light_inst = SensorValue[lightSensor];

      if((nMotorEncoder[LeftMotor] - last_enc) > LIFT_DEG){ // Scan Bar Every Lift
        light = SensorValue[lightSensor];
        last_enc = nMotorEncoder[LeftMotor];

        if(light >= WHITE_THRESH){
          if(THIS_IS_NOT_A_DRILL){ cont = 0; } // If stopping at White, Don't Cont(inue)
          playTone(780, 10);
        } else{
          playTone(445, 15);
        }

        rung_idx++;
      }

      if(rung_idx == 5){
        cont = 0; // Stop at Fifth (last) Rung Always (if app.)
      }

      motor[LeftMotor] = cont*run_vel;
      motor[RightMotor] = cont*run_vel;

      wait1Msec(4);
    } // loop

  } else{ // Starting on White
    motor[LeftMotor] = 0;
    motor[RightMotor] = 0;
    playTone(445, 15);
    wait1Msec(15);
    playTone(300, 10);
    while(1){ wait1Msec(1); /* Do Nothing */ }
  }

} // #main
