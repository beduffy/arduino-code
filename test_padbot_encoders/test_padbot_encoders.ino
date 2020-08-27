#include <Servo.h>
#include <Encoder.h>
#include <ros.h>
#include <ros/time.h>
#include <rosserial_arduino/Test.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

using rosserial_arduino::Test;

const int m1_in1 = 27;   // left
const int m1_in2 = 29;

const int m2_in1 = 23;  // right
const int m2_in2 = 25;

const int en1_pwm = 6;  // left
const int en2_pwm = 5;  // right

long oldPositionLeft  = 0;
long oldPositionRight  = 0;
const int check_rpm_period = 20;
const int lowest_power_value = 50;

// 8423 pulses per revolution by eye
//int pulses_per_revolution = 8423;  // for old worm gear motors
int pulses_per_revolution = 663;

Servo gripper_servo; 

// it somehow doesn't matter the order you put
//Encoder myEncLeft(2, 31);

const int PIN_ENCOD_A_MOTOR_LEFT = 2;    
volatile float pos_left = 0;       //Left motor encoder position

Encoder myEncRight(35, 3);
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
  //Serial.println("Within left Callback");
  /*digitalWrite(m1_in1, HIGH-digitalRead(m1_in1));
  digitalWrite(m1_in2, LOW-digitalRead(m1_in2));*/

  if(cmd_msg.data > 0) {
    // todo delete or duplicate
    //motor_value = constrain(motor_value, 60, 255);
    MotorClockwiseLeft(abs(cmd_msg.data));
  }
  else {
    //motor_value = constrain(motor_value, -255, -60);
    MotorCounterClockwiseLeft(abs(cmd_msg.data));
  }
}

void right_motor_cmd_callback( const std_msgs::Int16& cmd_msg){
  //Serial.println("Within right Callback");
  /*digitalWrite(m2_in1, HIGH-digitalRead(m2_in1));
  digitalWrite(m2_in2, LOW-digitalRead(m2_in2));*/

  if(cmd_msg.data > 0) {
    //motor_value = constrain(motor_value, 60, 255);
    MotorClockwiseRight(abs(cmd_msg.data));
  }
  else {
    //motor_value = constrain(motor_value, -255, -60);
    MotorCounterClockwiseRight(abs(cmd_msg.data));
  }
}

ros::Subscriber<std_msgs::Int16> rmotor_sub("lmotor_cmd", left_motor_cmd_callback);
ros::Subscriber<std_msgs::Int16> lmotor_sub("rmotor_cmd", right_motor_cmd_callback);

void servo_srv_callback(const Test::Request & req, Test::Response & res){
  //int servo_microsecond_delay = parseInt(req.input);
  int servo_microsecond_delay = atoi(req.input);

  gripper_servo.writeMicroseconds(servo_microsecond_delay);
}

ros::ServiceServer<Test::Request, Test::Response> servo_server("test_srv", &servo_srv_callback);

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

  gripper_servo.attach(9);

  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  //pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  //digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  //attachInterrupt(0, encoderLeftMotor, RISING);
  //attachInterrupt(0, encoderLeftMotor, CHANGE);
  //attachInterrupt(0, encoderLeftMotor, FALLING);
  //attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, FALLING);
  
  nh.initNode();
  //nh.getHardware()->setBaud(115200);
  nh.subscribe(lmotor_sub);
  nh.subscribe(rmotor_sub);
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);
  nh.advertiseService(servo_server);
}

void loop() {
  long curr_time = millis() - start_time;
  long time_difference = curr_time - prev_time;
  if (time_difference > check_rpm_period) {
    //long newPositionLeft = myEncLeft.read();
    long newPositionRight = myEncRight.read();

    //Serial.println(pos_left);
    //pos_left = digitalRead(PIN_ENCOD_A_MOTOR_LEFT);
    //pos_left = analogRead(A0);

    prev_time = curr_time;

    lwheel_pulse_msg.data = pos_left;
    lwheel_pub.publish(&lwheel_pulse_msg);
    rwheel_pulse_msg.data = newPositionRight;
    rwheel_pub.publish(&rwheel_pulse_msg);
    
    oldPositionLeft = pos_left;
    oldPositionRight = newPositionRight;
  }

  nh.spinOnce();
  
  //delay(1);  // todo keep or not? don't
}

void encoderLeftMotor() {
  pos_left++;
  //if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
  //else pos_left--;
}

void MotorClockwiseLeft(int power) {
  analogWrite(en1_pwm, power);
  digitalWrite(m1_in1, HIGH);
  digitalWrite(m1_in2, LOW);
}

void MotorCounterClockwiseLeft(int power) {
  analogWrite(en1_pwm, power);
  digitalWrite(m1_in1, LOW);
  digitalWrite(m1_in2, HIGH);
}

void MotorClockwiseRight(int power) {
  analogWrite(en2_pwm, power);
  digitalWrite(m2_in1, HIGH);
  digitalWrite(m2_in2, LOW);
}

void MotorCounterClockwiseRight(int power) {
  analogWrite(en2_pwm, power);
  digitalWrite(m2_in1, LOW);
  digitalWrite(m2_in2, HIGH);
}
