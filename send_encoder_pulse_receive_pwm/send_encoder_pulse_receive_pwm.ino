#include <Encoder.h>

boolean motor_direction = true;  // true is forward
const int m1_in1 = 10;
const int m1_in2 = 11;

const int m2_in1 = 5;
const int m2_in2 = 6;

const int en1_pwm = 9;
const int en2_pwm = 6;

long oldPositionLeft  = 0;
long oldPositionRight  = 0;
const int check_rpm_period = 20;

// 8423 pulses per revolution by eye
int pulses_per_revolution = 8423;

Encoder myEncLeft(2, 4);
Encoder myEncRight(3, 12);
long oldPosition  = -999;
unsigned long start_time = millis();
unsigned long last_time_for_revolution = millis();
unsigned long prev_time = start_time;

void setup() {
  Serial.begin(9600);
  //Serial.begin(19200);
  //Serial.begin(38400);
  //Serial.begin(115200);
  Serial.println("Began serial");

  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);
  pinMode(en1_pwm, OUTPUT);
  pinMode(en2_pwm, OUTPUT);

  analogWrite(en1_pwm, 0);
  analogWrite(en2_pwm, 255);

  digitalWrite(m1_in1, LOW);
  digitalWrite(m1_in2, LOW);
  digitalWrite(m2_in1, LOW);
  digitalWrite(m2_in2, LOW);

  if (motor_direction == true) {
    digitalWrite(m1_in1, HIGH);
    digitalWrite(m1_in2, LOW);
    digitalWrite(m2_in1, HIGH);
    digitalWrite(m2_in2, LOW);
  }
  else {
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, HIGH);
    digitalWrite(m2_in1, LOW);
    digitalWrite(m2_in2, HIGH);
  }

  Serial.println("Started motor");

  start_time = millis();
}

void loop() {
  if (Serial.available() > 0) {
    // todo read pwm for both wheels
    int value = Serial.parseInt();
    Serial.println(value);
    if (value != 0) {
      // if value is negative, change direction
      boolean changed_direction = false;
      if (value < 0 && motor_direction == true) {
        motor_direction = false;
        changed_direction = true;
      }
      else if (value > 0 && motor_direction == false) {
        motor_direction = true;
        changed_direction = true;
      }

      if (changed_direction) {
        if (motor_direction == true) {
          digitalWrite(m1_in1, HIGH);
          digitalWrite(m1_in2, LOW);
          digitalWrite(m2_in1, HIGH);
          digitalWrite(m2_in2, LOW);
        }
        else {
          digitalWrite(m1_in1, LOW);
          digitalWrite(m1_in2, HIGH);
          digitalWrite(m2_in1, LOW);
          digitalWrite(m2_in2, HIGH);
        }
      }

      if (value < 0) {
        value = value * -1;
      }
      analogWrite(en1_pwm, value);
      analogWrite(en2_pwm, value);
    }
  }

  long curr_time = millis() - start_time;
  long time_difference = curr_time - prev_time;
  if (time_difference > check_rpm_period) {
    long newPositionLeft = myEncLeft.read();
    long newPositionRight = myEncRight.read();

    prev_time = curr_time;

    Serial.print("newPositionLeft: ");
    Serial.print(newPositionLeft);
    Serial.print("  ");

    Serial.print("newPositionRight: ");
    Serial.println(newPositionRight);

    oldPositionLeft = newPositionLeft;
    oldPositionRight = newPositionRight;
  }
}
