#include <ros.h>
#include <std_msgs/UInt16.h>

const int m1_in1 = 10;
const int m1_in2 = 11;

const int m2_in1 = 5;
const int m2_in2 = 7;

const int en1_pwm = 9;
const int en2_pwm = 6;

ros::NodeHandle  nh;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  digitalWrite(m1_in1, HIGH-digitalRead(m1_in1));
  digitalWrite(m1_in2, LOW-digitalRead(m1_in2));
  digitalWrite(m2_in1, HIGH-digitalRead(m2_in1));
  digitalWrite(m2_in2, LOW-digitalRead(m2_in2));

  analogWrite(en1_pwm, 200);
  analogWrite(en2_pwm, 200);
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  //pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
