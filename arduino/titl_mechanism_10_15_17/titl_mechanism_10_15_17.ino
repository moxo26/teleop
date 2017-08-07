/*
 * Differential Drive
 * 
 * Written by: Josh Saunders
 * Written on: 12/22/2016
 *
 * Modified by: Josh Saunders
 * Modified on: 1/6/2016
 *
 * This sketch takes in a geometry_msgs/Twist message
 * and writes it to left and right Servos. This is an
 * adaptation of:
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * Input: geometry_msgs/Twist cmd_msg that corresponds to
 *        commands to send the differential drive robot
 *
 * Output: Movement based on the geometry_msgs/Twist commands
 *
 * Subscribers: "cmd_vel" receives a geometry_msg/Twist command
 *
 * Publishers: None
 *
 * Direction conventions used:
 *      TRANSLATIONS
 *      ------------
 *              FORWARD: linear.x = 1
 *              REVERSE: linear.x = -1
 *
 *      ROTATIONS
 *      ---------
 *              COUNTER CLOCKWISE: angular.z = 1
 *              CLOCKWISE        : angular.z = -1
 *
 * Constants: 
 *      STOP (float)       : Angle that stops the servo
 *      LEFT_SCALE (float) : Use this value to "tune" the system so that the servos spin at the same rate
 *      RIGHT_SCALE (float): Use this value to "tune" the system so that the servos spin at the same rate
 *      THRESHOLD (float)  : This value to keep the servos from spinning when the joystick is near center
 *      
 * How to run: 
 * - Make sure that 'roscore' is running
 * - Run 'rosrun rosserial_python serial_node.py /dev/ttyACMx' in the terminal.
 *   [Note: Replace 'x' with the port that your device is connected to]
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;




// use only for tilt arduino

Servo servo_tilt1; //bottom tilt motor
Servo servo_tilt2; //bottom tilt motor



void servo_cb(const geometry_msgs::Twist& cmd_msg){
  // @param direction_left: speed to be written to the Servos

  float tilt =  cmd_msg.linear.z;//key


//tilt mecahnism
  if (tilt == 2){
    servo_tilt1.write (88); // tilt fordward
    delay(1000);
    servo_tilt1.write(90); // pause tilt
  }

  if (tilt == -2){
    servo_tilt1.write (102); // tilt fordward
    delay(1000);
    servo_tilt1.write(90); // pause tilt;
  }
  
  if (tilt == 3){
    servo_tilt2.write (85); // tilt fordward
    delay(1000);
    servo_tilt2.write(90); // pause tilt
  }

  if (tilt == -3){
    servo_tilt2.write (98); // tilt fordward
    delay(1000);
    servo_tilt2.write(90); // pause tilt;
  }






}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", servo_cb);

void setup(){
  nh.initNode();
  nh.subscribe(sub_cmd_vel);


// use only for tilt mechanism  
  servo_tilt1.attach(10); // attach it to pin 10 for tilt
  servo_tilt2.attach(11); // attach it to pin 10 for tilt
  servo_tilt2.write (90); // tilt fordward
  servo_tilt1.write (90); // tilt fordward



}

void loop(){
  nh.spinOnce();
  delay(1);
}
