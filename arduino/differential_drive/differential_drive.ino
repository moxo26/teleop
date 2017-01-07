/*
 * Differential Drive
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
 * Written by: Josh Saunders
 * Written on: 12/22/2016
 *
 * Modified by: Josh Saunders
 * Modified on: 1/2/2016
 * 
 * How to run: 
 * - Make sure that 'roscore' is running
 * - Run 'rosrun rosserial_python serial_node.py /dev/ttyUSB0' in the terminal.
 *   - Replace 'ttyUSB0' with the port that your device is connected to
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

// Angles for the servos
const float STOP = 90;  // Stop
const float LEFT_SCALE = 0.15;
const float RIGHT_SCALE = 0.15;
const float THRESHOLD = 0.01;

Servo servo_left;
Servo servo_right;

void servo_cb(const geometry_msgs::Twist& cmd_msg){
  // @param direction_left: speed to be written to the Servos

  int direction_left  = STOP;
  int direction_right = STOP;

  float linear = cmd_msg.linear.x;
  float angular = cmd_msg.angular.z;

  if (linear > THRESHOLD || linear < -THRESHOLD) {
    // Go forward
    direction_left = int ( STOP * (1 - LEFT_SCALE * linear) );
    direction_right = int ( STOP * (1 + RIGHT_SCALE * linear) );
    
    Serial.print("Translate\n");
  } else if (angular > THRESHOLD || angular < -THRESHOLD) {
    // Rotate counter clockwise
    direction_left = int ( STOP * (1 + LEFT_SCALE * angular) );
    direction_right = int ( STOP * (1 + RIGHT_SCALE * angular) );
    
    Serial.print("Rotate\n");
  } else {
    // STOP!

    // TODO: engage the brakes here...
    direction_left = STOP;
    direction_right = STOP;
    
    Serial.print("Stop\n");
  }

  servo_left.write(direction_left); //set servo_left angle
  servo_right.write(direction_right); //set servo_right angle
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", servo_cb);

void setup(){
  nh.initNode();
  nh.subscribe(sub_cmd_vel);

  servo_left.attach(8); //attach it to pin 8
  servo_right.attach(9); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}
