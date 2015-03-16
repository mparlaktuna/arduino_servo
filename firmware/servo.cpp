#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>

// create position and speed values, maybe make them into a struck in the future
int motor_positions[11];
int speeds[11];
int motor_goal_positions[11];


ros::NodeHandle  nh;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void servo_cb( const std_msgs::UInt16MultiArray& cmd_msg){
	motor_goal_positions[cmd_msg.data[0]] = cmd_msg.data[1];
	speeds[cmd_msg.data[0]] = cmd_msg.data[2];
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servo_cb);


void setup(){

	for (int x = 0; x < 11  ; x++)
	{
//		initial positions are the middle positions
		motor_positions[x]=200;
		motor_goal_positions[x]=200;
		speeds[x] = 5;
	}

  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
	for (int x = 0; x < 11; x++)
		{
			//calculate error and next position depending on speed
			motor_positions[x] = motor_positions[x] + (motor_goal_positions[x]-motor_positions[x])/speeds[x];
			pwm.setPWM(x, 0, motor_positions[x]);
		}
	nh.spinOnce();
	delay(1);
}
