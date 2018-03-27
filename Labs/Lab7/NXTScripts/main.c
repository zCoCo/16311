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

float V = 0; // Body Center Speed (alongtrack) [m/s]
float om = 0; // Body Center Angular Speed [rad/s]

int DPan = 0; // Amount to Change Pan Angle By [deg]
int DTilt = 0; // Amount to Change Tilt Angle By [deg]

int btnX = 0; int btnY = 0; int btnA = 0; int btnB = 0;

// ---- DISPLAY --- //
void update_display(){
	nxtSetPixel(50 + (int)(100.0 * 0.0), 32 + (int)(100.0 * 0.0));
	nxtDisplayTextLine(1, "V: %fm/s", V);
	nxtDisplayTextLine(2, "O: %frad/s", om);
	nxtDisplayTextLine(3, "DP: %ddeg", DPan);
	nxtDisplayTextLine(4, "DT: %ddeg", DTilt);
	nxtDisplayTextLine(5, "t: %dms", TSF_Last(Hist_Time));
}

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
	wait1Msec(25); //Wait for I2C Hardware
}

// Commands the Servo with Index idx to Change by Angle Da
void I2CServo(int idx, int Da){
  if(Da >= 0){ // Increment
    sendI2C(11); sendI2C(idx); sendI2C(Da);
  } else{ // Decrement
    sendI2C(12); sendI2C(idx); sendI2C(-Da);
  }
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

task main()
{
	init_HAL();
	startTask(odometry);

  wait1Msec(500);
  I2CRGB(LED_RED);

	while(1){
    getJoystickSettings(joystick);

    V = -1.0 * MAX_VEL * filter_joy(joystick.joy1_y1) / 128.0;
    om = -1.0 * MAX_OMEGA * filter_joy(joystick.joy1_x1) / 128.0;

    int DPan_temp = -1.0 * 10.0 * filter_joy(joystick.joy1_x2) / 128.0;
    int DTilt_temp = -1.0 * 5.0 * filter_joy(joystick.joy1_y2) / 128.0;

    moveAt(V,om);

    if(DPan != DPan_temp){
      I2CServo(1, DPan);
      DPan = DPan_temp;
    }
    if(DTilt != DTilt_temp){
      I2CServo(2, DTilt);
      DTilt = DTilt_temp;
    }

    // Update Button Controls:
    int btnX_temp = joy1_Buttons & 8; // Blue Button
    int btnY_temp = joy1_Buttons & 4; // Yellow Button
    int btnA_temp = joy1_Buttons & 0; // Green Button
    int btnB_temp = joy1_Buttons & 2; // Red Button

    if(btnB != btnB_temp){
      if(btnB){ I2CRGB(LED_RED); }
      btnB = btnB_temp;
    }
    if(btnA != btnA_temp){
      if(btnA){ I2CRGB(LED_GREEN); }
      btnA = btnA_temp;
    }
    if(btnX != btnX_temp){
      if(btnX){ I2CRGB(LED_BLUE); }
      btnX = btnX_temp;
    }
    if(btnY != btnY_temp){
      if(btnY){ I2CRGB(LED_WHITE); }
      btnY = btnY_temp;
    }

		update_display();
		wait1Msec(10);
	}

	motor[RightMotor] = 0;
	motor[LeftMotor] = 0;

	nNxtButtonTask  = 0;
	while(nNxtButtonPressed != kExitButton) {}
}
