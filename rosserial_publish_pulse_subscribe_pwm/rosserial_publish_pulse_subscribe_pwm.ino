#include <Encoder.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

const int m1_in1 = 5;   // left
const int m1_in2 = 7;

const int m2_in1 = 10;  // right
const int m2_in2 = 11;

const int en1_pwm = 6;  // left
const int en2_pwm = 9;  // right

long oldPositionLeft  = 0;
long oldPositionRight  = 0;
const int check_rpm_period = 20;
const int lowest_power_value = 50;

// 8423 pulses per revolution by eye
//int pulses_per_revolution = 8423;  // for old worm gear motors
int pulses_per_revolution = 663;

Encoder myEncLeft(2, 4);
Encoder myEncRight(3, 12);
long oldPosition  = -999;
unsigned long start_time = millis();
unsigned long last_time_for_revolution = millis();
unsigned long prev_time = start_time;

ros::NodeHandle  nh;
std_msgs::Int32 lwheel_pulse_msg;
ros::Publisher lwheel_pub("lwheel", &lwheel_pulse_msg);
std_msgs::Int32 rwheel_pulse_msg;
ros::Publisher rwheel_pub("rwheel", &rwheel_pulse_msg);

// one callback or two?
// Float32? or int?

void left_motor_cmd_callback( const std_msgs::Int16& cmd_msg){
  Serial.println("Within left Callback");
  digitalWrite(m1_in1, HIGH-digitalRead(m1_in1));
  digitalWrite(m1_in2, LOW-digitalRead(m1_in2));

  if(cmd_msg.data > 0) {
    // todo delete or duplicate
    //motor_value = constrain(motor_value, 60, 255);
    MotorCounterClockwiseLeft(abs(cmd_msg.data));
  }
  else {
    //motor_value = constrain(motor_value, -255, -60);
    MotorClockwiseLeft(abs(cmd_msg.data));
  }
}

void right_motor_cmd_callback( const std_msgs::Int16& cmd_msg){
  Serial.println("Within right Callback");
  /*digitalWrite(m2_in1, HIGH-digitalRead(m2_in1));
  digitalWrite(m2_in2, LOW-digitalRead(m2_in2));*/

  if(cmd_msg.data > 0) {
    //motor_value = constrain(motor_value, 60, 255);
    MotorCounterClockwiseRight(abs(cmd_msg.data));
  }
  else {
    //motor_value = constrain(motor_value, -255, -60);
    MotorClockwiseRight(abs(cmd_msg.data));
  }
}

ros::Subscriber<std_msgs::Int16> rmotor_sub("lmotor_cmd", left_motor_cmd_callback);
ros::Subscriber<std_msgs::Int16> lmotor_sub("rmotor_cmd", right_motor_cmd_callback);

void setup(){
  Serial.println("Hey everybody");
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);
  pinMode(en1_pwm, OUTPUT);
  pinMode(en2_pwm, OUTPUT);

  digitalWrite(m1_in1, HIGH);
  digitalWrite(m1_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  digitalWrite(m2_in2, LOW);

  analogWrite(en1_pwm, 0);
  analogWrite(en2_pwm, 0);

  start_time = millis();

  nh.initNode();
  nh.subscribe(lmotor_sub);
  nh.subscribe(rmotor_sub);
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);
  Serial.println("End of setup");
}

void loop(){
  //analogWrite(en1_pwm, 130);
  //analogWrite(en2_pwm, 130);
  long curr_time = millis() - start_time;
  long time_difference = curr_time - prev_time;
  if (time_difference > check_rpm_period) {
    long newPositionLeft = myEncLeft.read();
    long newPositionRight = myEncRight.read();

    prev_time = curr_time;

    lwheel_pulse_msg.data = newPositionLeft;
    lwheel_pub.publish(&lwheel_pulse_msg);
    rwheel_pulse_msg.data = newPositionRight;
    rwheel_pub.publish(&rwheel_pulse_msg);
    
    oldPositionLeft = newPositionLeft;
    oldPositionRight = newPositionRight;
  }

  nh.spinOnce();
  
  //delay(1);  // todo keep or not? don't
}


void MotorClockwiseLeft(int power) {
  //if(power > lowest_power_value) {
    analogWrite(en1_pwm, power);
    /*digitalWrite(m1_in1, HIGH);
    digitalWrite(m1_in2, LOW);*/
    digitalWrite(m1_in1, HIGH-digitalRead(m1_in1));
    digitalWrite(m1_in2, LOW-digitalRead(m1_in2));
  /*}
  else {
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, LOW);
  }*/
}

void MotorCounterClockwiseLeft(int power) {
  if(power > lowest_power_value) {
    analogWrite(en1_pwm, power);
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, HIGH);
  }
  else {
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, LOW);
    
    // todo try below?
    /*analogWrite(en1_pwm, lowest_power_value);
    digitalWrite(m1_in1, HIGH);
    digitalWrite(m1_in2, LOW);*/
  }
}

void MotorClockwiseRight(int power) {
  if(power > lowest_power_value) {
    analogWrite(en2_pwm, power);
    /*digitalWrite(m2_in1, HIGH);
    digitalWrite(m2_in2, LOW);*/
    digitalWrite(m2_in1, HIGH-digitalRead(m2_in1));
    digitalWrite(m2_in2, LOW-digitalRead(m2_in2));
  }
  else {
    digitalWrite(m2_in1, LOW);
    digitalWrite(m2_in2, LOW);
  }
}

void MotorCounterClockwiseRight(int power) {
  if(power > lowest_power_value) {
    analogWrite(en2_pwm, power);
    digitalWrite(m2_in1, LOW);
    digitalWrite(m2_in2, HIGH);
  }
  else {
    digitalWrite(m2_in1, LOW);
    digitalWrite(m2_in2, LOW);
  }
}
