#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>


ros::NodeHandle  nh;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void servo_cb( const std_msgs::UInt16MultiArray& cmd_msg){
  pwm.setPWM(cmd_msg.data[0], 0, cmd_msg.data[1]);
}


ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servo_cb);

void setup(){
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
