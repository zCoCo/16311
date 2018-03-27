#define LED_R 9
#define LED_B 6
#define LED_G 11

#define SERV_1 4
#define SERV_2 8
#define SERV_3 10

#include <Servo.h>
#include <Wire.h>

#include "Colors.h"

#define NUM_SERVOS 3
Servo serv[NUM_SERVOS];
int serv_pos[NUM_SERVOS]; // Array of Servo Positions

#define PAN serv[0]
#define TILT serv[1]

#define LASER 12

#define sign(v) (((v)>0)-((v)<0))

// Set Servo with Index idx to Position pos.
void servPos(int idx, int pos){
  idx = constrain(idx, 0,NUM_SERVOS-1);
  pos = constrain(pos, 0,200);
  serv[idx].write(pos);
  serv_pos[idx] = pos;
}

#define SERVO_SWEEP_TIMESTEP 10
#define DEFAULT_PAN 90
void panTo(int ang){ // -- DEPRECATED --
  static int last_angle = DEFAULT_PAN;
  for(int i=last_angle; abs(ang-last_angle)>0; i+=sign(ang-last_angle)){ //Perform Change Gradually
    PAN.write(i);
    delay(SERVO_SWEEP_TIMESTEP);
  }
}
#define DEFAULT_TILT 0
void tiltTo(int ang){ // -- DEPRECATED --
  static int last_angle = DEFAULT_TILT;
  for(int i=last_angle; abs(ang-last_angle)>0; i+=sign(ang-last_angle)){ //Perform Change Gradually
    TILT.write(i);
    delay(SERVO_SWEEP_TIMESTEP);
  }
}

void setup(){
  Serial.begin(115200);

  Wire.begin(0x0A);
  Wire.onReceive(read_I2C_message);

  serv[0].attach(SERV_1); serv[1].attach(SERV_2); serv[2].attach(SERV_3);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LASER, OUTPUT);
   delay(100);

  color(WHITE);

  // panTo(DEFAULT_PAN);
  // tiltTo(DEFAULT_TILT);
  servPos(0, 119);
  servPos(1, 0);

  digitalWrite(LASER, HIGH);
} // #setup

void loop(){
  read_uart_message();
}

// I2C Message Intake
// Uses Same Messaging Protocol as UART
void read_I2C_message(int bytesIn){
  static char msg_byte = 0; // Current Message Byte being Processed
  static int curr_msg[3] = {0,0,0};

  Wire.read(); // Get rid of Garbage Data
  curr_msg[msg_byte] = (int) Wire.read();
  Wire.read(); // Get rid of Garbage Data

  if(msg_byte == 2){
    process_I2C_message(curr_msg);
    msg_byte = 0;
                                                                              // Serial.print(curr_msg[0]); Serial.print(curr_msg[1]); Serial.println(curr_msg[2]);
  } else{
    msg_byte++;
  }
}
void process_I2C_message(int* msg){ process_uart_message(msg); } // I2C Uses Same Protocol as UART

/* UART MESSAGING PROTOCOL:
 - 3 Byte Message:
 -- 1: Command   2: Target   3: Value
  Comm 1 -> Control RGB -> LED(0->R, 1->G, 2->B) -> Value
  Comm 2 -> Display RGB Color -> Pre-Defined Color -> N/A
    - Color 0: DARK (off)
    - Color 1: RED
    - Color 2: GREEN
    - Color 3: BLUE
    - Color 4: WHITE

  Comm 10 -> Control Servo -> Servo Number (1 indexed) -> Servo Angle
  Comm 11 -> Increase Servo Position -> Servo Number -> Increment
  Comm 12 -> Decrease Servo Position -> Servo Number -> Decrement

  Comm 20 -> Toggle Laser -> 0 Off, 1 On -> N/A
  */
void read_uart_message(){
  static char msg_byte = 0; // Current Message Byte being Processed
  static int curr_msg[3] = {0,0,0};

  if(Serial.available() > 0){
    curr_msg[msg_byte] = (int) Serial.read();

    if(msg_byte == 2){
      process_uart_message(curr_msg);
      msg_byte = 0;
                                                                                // Serial.print(curr_msg[0]); Serial.print(curr_msg[1]); Serial.println(curr_msg[2]);
    } else{
      msg_byte++;
    }
  }
}

void process_uart_message(int* msg){
  int commcolor[3] = {currRed, currGreen, currBlue}; // Default to Current Values

  switch(msg[0]){ // Command
    case 1: //                                - Control RGB
      commcolor[constrain(msg[1], 0, 2)] = constrain(msg[2], 0,255);
      color(commcolor);
    break;

    case 2: //                                - Display Color
      color(COMMAND_COLORS[constrain(msg[1], 0,4)]);
    break;


    case 10: //                               - Control Servo
    {
      // int targ = constrain(msg[2], 0,200);
      // int sid = constrain(msg[1], 1,3)-1;
      // void (*move)(int) = (sid==0) ? &panTo : &tiltTo;
      // move(targ);
      serv[constrain(msg[1], 1,3)-1].write(constrain(msg[2], 0,200));
    }
    break;

    case 11: //                               - Increase Servo Position
    {
      int idx = constrain(msg[1], 1,3) - 1;
      int inc = msg[2];
      servPos(idx, serv_pos[idx] + inc);
    }
    break;

    case 12: //                               - Decrease Servo Position
    {
      int idx = constrain(msg[1], 1,3) - 1;
      int dec = msg[2];
      servPos(idx, serv_pos[idx] - dec);
    }
    break;

    case 20:
      digitalWrite(LASER, msg[1]);
    break;

    default:
      // Do Nothing.
    break;
  }
}
