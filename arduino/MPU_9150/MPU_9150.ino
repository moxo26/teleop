#include<Wire.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t x,y,z;


void setup(){
  //Gyro
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop(){
  //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Gyro();
  Serial.print(" | X = "); Serial.print(x);
  Serial.print(" | Y = "); Serial.print(y);
  Serial.print(" | Z = "); Serial.println(z);
  
}

void Gyro(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);  // starting with register 0x43 (GYRO_XOUT_H) reference data sheet for any other values such as TEMP
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,6,true);  // request a total of 6 registers, Request more for Acellerometer & compass
  //tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  x=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  y=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  z=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}

