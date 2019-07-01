#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;
Servo servo9;

int joyX = 0;
int joyY = 1;

int joyVal;

void setup()
{
  servo1.attach(4);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(7);
  servo5.attach(8);
  servo6.attach(9);
  servo7.attach(10);
  servo8.attach(11);
  servo9.attach(12); 
}

void loop()
{
  joyVal = analogRead(joyX);
  joyVal = map (joyVal, 0 ,1023, 0, 180);
  servo1.write(joyVal);
  servo2.write(joyVal);
  servo3.write(joyVal);
  servo4.write(joyVal);
  servo5.write(joyVal);
  servo6.write(joyVal);
  servo7.write(joyVal);
  servo8.write(joyVal);
  servo9.write(joyVal);

  delay(100);
  
//  joyVal = analogRead(joyY);
//  joyVal = map(joyVal, 0, 1023, 0 , 180);
//  servo4.write(joyVal);
//  servo5.write(joyVal);
//  servo6.write(joyVal);
// 
//  delay(100);


  
}

