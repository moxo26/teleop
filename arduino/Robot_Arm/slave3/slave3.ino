// Include the required Wire library for I2C
//This is the code for the master and slave setup for the Teleoperation arms. Please refer to the Master/Slave Schematic file to see the wiring schematic.
//This code is strictly for the use of joy sticks to control the motors, maybe potentiometer, but nothing else. Please alter code to use other controllers.
//This code has issues with communication. Sometimes communication is broken, so to avoid disaster, set initial x values as neutral value.
#include <Wire.h>
#include <Arduino.h>
#include "DRV8825.h"

#define MOTOR_STEPS 200
#define RPM 120

#define DIR 2
#define STEP 3
#define MODE0 6
#define MODE1 5
#define MODE2 4
DRV8825 stepper(MOTOR_STEPS, DIR, STEP, MODE0, MODE1, MODE2);

#define DIR2 8
#define STEP2 9
#define MODE02 12
#define MODE12 11
#define MODE22 10
DRV8825 stepper2(MOTOR_STEPS, DIR2, STEP2, MODE02, MODE12, MODE22);

int x = 520 / 4;
int y = 520/4;

void setup() {
  stepper.begin(RPM);
  stepper.enable();
  Wire.begin(7);   //initiate this slave on channel 7, refer to master code for commands sent
  Serial.begin(9600);
  Wire.onReceive(receiveEvent);  //Create evemt to receive information from master
}

void loop() {
  int  x1 = 4 * x;
  int y1 = 4 * y;
  while (Wire.available()) {
    if (x1 > 650 && x1 <= 1023) {
      stepper.move(MOTOR_STEPS);
      delay(1);
    }
    else if (x1 < 400 && x1 > 0) {
      stepper.move(-MOTOR_STEPS);
    }
    else if (x1 > 650 && x1 <= 1023) {
      stepper2.move(MOTOR_STEPS);
      delay(1);
    }
    else if (y1 < 400 && y1 > 0) {
      stepper2.move(-MOTOR_STEPS);
    }
    else {
      stepper.move(0);
      stepper2.move(0);
    }
  }
}

void receiveEvent(int bytes) {
  int x =  Wire.read();    // read one character from the I2C
  int y = Wire.read();
}
