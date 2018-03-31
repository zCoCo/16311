#pragma config(Sensor, S1,     TIR,                 sensorI2CCustom)
#pragma config(Sensor, S2,     sonarSensor,         sensorSONAR)
/**********************************************
 * Basic Joystick Control Script for NXT Base
 **********************************************/

 #include "JoystickDriver.c"
 #pragma DebuggerWindows("joystickSimple")

#include "Toolbox/Util/UtilStack.h"
#include "Toolbox/HALs/HAL.h"

#define MainClock T1

#define ArmMot motorA

#define ArmMin 0
#define ArmMax 80

float V = 0; // Body Center Speed (alongtrack) [m/s]
float om = 0; // Body Center Angular Speed [rad/s]

int PanSpeed = 0; // Amount to Change Pan Angle By [deg]
int DTilt = 0; // Amount to Change Tilt Angle By [deg]

int btnX = 0; int btnY = 0; int btnA = 0; int btnB = 0;

// ---- DISPLAY --- //
void update_display(){
	nxtSetPixel(50 + (int)(100.0 * 0.0), 32 + (int)(100.0 * 0.0));
	nxtDisplayTextLine(1, "V: %fm/s", V);
	nxtDisplayTextLine(2, "O: %frad/s", om);
	nxtDisplayTextLine(3, "PS: %ddeg", PanSpeed);
	nxtDisplayTextLine(4, "DT: %ddeg", DTilt);
	nxtDisplayTextLine(5, "t: %dms", TSF_Last(Hist_Time));
}

ubyte last_comm = 0;
// Sends a Byte over I2C
void sendI2C(ubyte command){
	ubyte msgSize = 0x04;
	ubyte deviceAddress = 0x14; //0x0A on Arduino
	ubyte regAddress = 0x01;

	ubyte i2cMsg[7]; //Create empty msg on stack
	i2cMsg[0] = msgSize;
	i2cMsg[1] = deviceAddress;
	i2cMsg[2] = regAddress;
	i2cMsg[3] = command;

	//These are unused, but should be 0 anyway
	i2cMsg[4] = 0x00;
	i2cMsg[5] = 0x00;
	i2cMsg[6] = 0x00;

	sendI2CMsg(S1, &i2cMsg[0], 0x00);
  last_comm = command;
	wait1Msec(15); //Wait for I2C Hardware
}

// Commands the Servo with Index idx to Change by Angle Da
void I2CServo(int idx, int Da){
  if(Da >= 0){ // Increment
    sendI2C(11); sendI2C(idx); sendI2C(Da);
  } else{ // Decrement
    sendI2C(12); sendI2C(idx); sendI2C(-Da);
  }
}

// Commands the Servo with Index idx to Take Up Angle pos
void I2CServoPos(int idx, int pos){
  sendI2C(10); sendI2C(idx); sendI2C(pos);
}

// Pass Odometry Data to Web Socket
void I2CPassOdo(){
  sendI2C(90); sendI2C(1); sendI2C((int)(rob_pos_X));
  sendI2C(90); sendI2C(2); sendI2C((int)(100*(rob_pos_X - ((int)(rob_pos_X)))));
  sendI2C(90); sendI2C(3); sendI2C((int)(rob_pos_Y));
  sendI2C(90); sendI2C(4); sendI2C((int)(100*(rob_pos_Y - ((int)(rob_pos_Y)))));
  sendI2C(90); sendI2C(5); sendI2C((int)(rob_pos_TH));
  sendI2C(90); sendI2C(6); sendI2C((int)(100*(rob_pos_TH - ((int)(rob_pos_TH)))));
}

// Commands the RGB LEDs to Take up the Color with the Given Index
#define LED_DARK 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3
#define LED_WHITE 4
void I2CRGB(int color_idx){
  sendI2C(2); sendI2C(color_idx); sendI2C(0);
}

void I2CLaser(int state){
  sendI2C(20); sendI2C(state); sendI2C(0);
}

void toggleLaser(){
  static int laser_state = 1;
  laser_state = !laser_state;
  I2CLaser(laser_state);
}

/*****************************************
 * Joystick Control
 * Using XBOX 360 Controller for Windows (Wireless)
 * Custom Controller Config:
 *  X1: Left_X
 *  Y1: Left_Y
 *  X2: Right_X
 *  Y2: Right_Y
 *  Pad: POV1
 *  Button Mapping: Default
 *****************************************/
 #define JOYSTICK_THRESHOLD 20 // Min Joystick Reading (Noise Threshold)
 int filter_joy(int raw){ // Filter Joystick Value with Threshold
   return abs(raw) > JOYSTICK_THRESHOLD ? raw : 0;
 }

/*****************************************
 * Main function
 *****************************************/
long last_odo_pass = 0;
task main()
{
	init_HAL();
	startTask(odometry);

  nMotorPIDSpeedCtrl[ArmMot] = mtrSpeedReg;
  motor[ArmMot] = 0;

  wait1Msec(500);
  I2CRGB(LED_RED);

	while(1){
    getJoystickSettings(joystick);

    V = -1.0 * MAX_VEL * filter_joy(joystick.joy1_y1) / 128.0;
    om = -1.0 * MAX_OMEGA * filter_joy(joystick.joy1_x1) / 128.0;

    PanSpeed = -1.0 * 5.0 * filter_joy(joystick.joy1_x2) / 128.0;
    DTilt = -1.0 * 5.0 * filter_joy(joystick.joy1_y2) / 128.0;

    moveAt(V,om);

    if(joystick.joy1_TopHat == 0){
      motor[ArmMot] = 100;
      wait1Msec(20);
      motor[ArmMot] = 0;
    }
    if(joystick.joy1_TopHat == 4){
      motor[ArmMot] = -100;
      wait1Msec(20);
      motor[ArmMot] = 0;
    }

    I2CServoPos(1, abs(91 + PanSpeed)); // 91 is Neutral Position (no speed)
    I2CServo(2, DTilt);

    if(joy1Btn(5)){
      toggleLaser();
    }

    // Update Button Controls:
    int btnX_temp = joy1Btn(3); // Blue Button
    int btnY_temp = joy1Btn(4); // Yellow Button
    int btnA_temp = joy1Btn(1); // Green Button
    int btnB_temp = joy1Btn(2); // Red Button

    if(btnB != btnB_temp){
      if(btnB_temp){ I2CRGB(LED_RED); }
      btnB = btnB_temp;
    }
    if(btnA != btnA_temp){
      if(btnA_temp){ I2CRGB(LED_GREEN); }
      btnA = btnA_temp;
    }
    if(btnX != btnX_temp){
      if(btnX_temp){ I2CRGB(LED_BLUE); }
      btnX = btnX_temp;
    }
    if(btnY != btnY_temp){
      if(btnY_temp){ I2CRGB(LED_WHITE); }
      btnY = btnY_temp;
    }

    if((TSF_Last(Hist_Time)-last_odo_pass) > 500){
      I2CPassOdo();
      last_odo_pass = TSF_Last(Hist_Time);
    }

		update_display();
		wait1Msec(10);
	}

	motor[RightMotor] = 0;
	motor[LeftMotor] = 0;

	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
