// Include the required Wire library for I2C
// Include the required Wire library for I2C
//This is the code for the master and slave setup for the Teleoperation arms. Please refer to the Master/Slave Schematic file to see the wiring schematic.
//This code is strictly for the use of joy sticks to control the motors, maybe potentiometer, but nothing else. Please alter code to use other controllers. 
//This code has issues with communication. Sometimes communication is broken, so to avoid disaster, set initial x values as neutral value.
#include <Wire.h>
#include <Servo.h>
Servo esc1;
Servo esc2;
int x = 520 / 4;

void setup() {
  esc1.attach(3);
  esc2.attach(4);
  Wire.begin(8);   //initiate this slave on channel 8, refer to master code for commands sent
  Serial.begin(9600);
  Wire.onReceive(receiveEvent);  //Create evemt to receive information from master
}

void loop() {
  int  x1 = 4 * x;
  int y1 = 4 * y;
  while (Wire.available()) {
    int val1 = map(x1, 0, 1023, 1000, 2000);
    esc1.writeMicroseconds(val1);
  }
}

void receiveEvent(int bytes) {
  int x =  Wire.read();    // read one character from the I2C

}
//Slave #2 controls the motor used on the elbow joint. 
