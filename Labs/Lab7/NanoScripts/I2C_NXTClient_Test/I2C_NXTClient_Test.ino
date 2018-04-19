#include <Wire.h>

byte currentCommand;

void setup(){
  Serial.begin(115200);
  Wire.begin(0x0A);
  Wire.onReceive(receiveCommand);

  Serial.println("-- Serial and Wire Online --");
}

void loop(){

}

void receiveCommand(int bytesIn){
    byte garbage = Wire.read();
    currentCommand = Wire.read();
    garbage = Wire.read();

    Serial.print("I2C In: "); Serial.println(currentCommand);
}
