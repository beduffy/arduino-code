#include <Servo.h>
#include <Encoder.h>
#include <ros.h>
#include <ros/time.h>
#include <rosserial_arduino/Test.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

using rosserial_arduino::Test;

// right
const int m1_in1 = 23;   // in1
const int m1_in2 = 29;   // in2

// left
const int m2_in1 = 35;  // in3
const int m2_in2 = 44;  // in4

const int en1_pwm = 6;  // right. ENA
const int en2_pwm = 5;  // left. ENB

long oldPositionLeft  = 0;
long oldPositionRight  = 0;
const int check_rpm_period = 20;
const int lowest_power_value = 50;

int pulses_per_revolution = 663;

//Servo gripper_servo;

// it somehow doesn't matter the order you put
Encoder myEncLeft(2, 50);
Encoder myEncRight(9, 3);  // todo should be 2 and 3
long oldPosition  = -999;
unsigned long start_time = millis();
unsigned long last_time_for_revolution = millis();
unsigned long prev_time = start_time;

ros::NodeHandle  nh;
std_msgs::Int32 lwheel_pulse_msg;
ros::Publisher lwheel_pub("lwheel", &lwheel_pulse_msg);
std_msgs::Int32 rwheel_pulse_msg;
ros::Publisher rwheel_pub("rwheel", &rwheel_pulse_msg);

void left_motor_cmd_callback( const std_msgs::Int16& cmd_msg) {
  if(cmd_msg.data > 0) {
    MotorClockwiseLeft(abs(cmd_msg.data));
  }
  else {
    MotorCounterClockwiseLeft(abs(cmd_msg.data));
  }
}

void right_motor_cmd_callback( const std_msgs::Int16& cmd_msg){
  if(cmd_msg.data > 0) {
    MotorClockwiseRight(abs(cmd_msg.data));
  }
  else {
    MotorCounterClockwiseRight(abs(cmd_msg.data));
  }
}

ros::Subscriber<std_msgs::Int16> lmotor_sub("lmotor_cmd", left_motor_cmd_callback);
ros::Subscriber<std_msgs::Int16> rmotor_sub("rmotor_cmd", right_motor_cmd_callback);

/*void servo_srv_callback(const Test::Request & req, Test::Response & res) {
  // when we had an arm, we used this, but no arm anymore
  int servo_microsecond_delay = atoi(req.input);

  gripper_servo.writeMicroseconds(servo_microsecond_delay);
}

ros::ServiceServer<Test::Request, Test::Response> servo_server("test_srv", &servo_srv_callback);*/

void setup() {
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);
  pinMode(en1_pwm, OUTPUT);
  pinMode(en2_pwm, OUTPUT);

  digitalWrite(m1_in1, LOW);
  digitalWrite(m1_in2, LOW);
  digitalWrite(m2_in1, LOW);
  digitalWrite(m2_in2, LOW);

  // for testing
  //analogWrite(en1_pwm, 80);
  //analogWrite(en2_pwm, 80);
  //MotorClockwiseRight(255);
  //MotorCounterClockwiseRight(80);
  //MotorClockwiseLeft(255);
  //MotorCounterClockwiseLeft(80);

  start_time = millis();
  //gripper_servo.attach(9);

  TCCR3B = TCCR3B & B11111000 | B00000001;    // set timer 3 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR4B = TCCR4B & B11111000 | B00000001;    // set timer 4 divisor to     1 for PWM frequency of 31372.55 Hz
  
  nh.initNode();
  nh.subscribe(lmotor_sub);
  nh.subscribe(rmotor_sub);
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);
  //nh.advertiseService(servo_server);
}

void loop() {
  long curr_time = millis() - start_time;
  long time_difference = curr_time - prev_time;
  if (time_difference > check_rpm_period) {
    long newPositionLeft = myEncLeft.read();
    long newPositionRight = myEncRight.read();

    Serial.println(newPositionLeft);

    prev_time = curr_time;

    lwheel_pulse_msg.data = newPositionLeft;
    lwheel_pub.publish(&lwheel_pulse_msg);
    rwheel_pulse_msg.data = newPositionRight;
    rwheel_pub.publish(&rwheel_pulse_msg);
    
    oldPositionLeft = newPositionLeft;
    oldPositionRight = newPositionRight;
  }

  nh.spinOnce();
}


void MotorClockwiseLeft(int power) {
  analogWrite(en2_pwm, power);
  digitalWrite(m2_in1, HIGH);
  digitalWrite(m2_in2, LOW);
}

void MotorCounterClockwiseLeft(int power) {
  analogWrite(en2_pwm, power);
  digitalWrite(m2_in1, LOW);
  digitalWrite(m2_in2, HIGH);
}

void MotorClockwiseRight(int power) {
  analogWrite(en1_pwm, power);
  digitalWrite(m1_in1, HIGH);
  digitalWrite(m1_in2, LOW);
}

void MotorCounterClockwiseRight(int power) {
  analogWrite(en1_pwm, power);
  digitalWrite(m1_in1, LOW);
  digitalWrite(m1_in2, HIGH);
}
