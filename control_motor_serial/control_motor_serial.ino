#include <Encoder.h>
#include <PID_v1.h>

boolean motor_direction = true;  // true is forward
const int m1_in1 = 10;
const int m1_in2 = 11;

const int m2_in1 = 5;
const int m2_in2 = 6;

const int en1_pwm = 9;
const int en2_pwm = 6;

double RPM_left = 1;
double RPM_right = 1;
long oldPositionLeft  = 0;
long oldPositionRight  = 0;
int incomingByte = 0; // for incoming serial data
int delay_amt = 100;
const int check_rpm_period = 20;
//double RPM = 1;

// 8423 pulses per revolution by eye
int pulses_per_revolution = 8423;

//Encoder myEnc(8, 12);  // attempt at polling with 2 normal digital
//Encoder myEnc(3, 4); // attempt at 1 interrupt, 1 normal digital
//Encoder myEnc(2, 3);
//Encoder myEnc(12, 13);
Encoder myEncLeft(2, 4);
Encoder myEncRight(3, 12);
long oldPosition  = -999;
unsigned long start_time = millis();
unsigned long last_time_for_revolution = millis();
unsigned long prev_time = start_time;

void setup() {
  Serial.begin(9600);
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
  /*Serial.println("forward");
    digitalWrite(m1_in1, HIGH);
    digitalWrite(m1_in2, LOW);
    digitalWrite(m2_in1, HIGH);
    digitalWrite(m2_in2, LOW);*/
  //analogWrite(en1_pwm, 255);
  /*delay(2000);

    Serial.println("backwards");
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, HIGH);
    digitalWrite(m2_in1, LOW);
    digitalWrite(m2_in2, HIGH);
    //analogWrite(en1_pwm, 255);
    delay(2000);*/

  /*for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    analogWrite(en1_pwm, fadeValue);
    Serial.println(fadeValue);
    delay(delay_amt);
    }

    // fade out from max to min in increments of 5 points:
    for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
    analogWrite(en1_pwm, fadeValue);
    Serial.println(fadeValue);
    delay(delay_amt);
    }*/

  if (Serial.available() > 0) {
    // read the incoming byte:
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

  //long newPosition = myEnc.read();
  /*if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
    if (newPosition % pulses_per_revolution == 0)
    {
      int curr_time = millis();
      int time_taken_for_one_revolution = curr_time - last_time_for_revolution;
      last_time_for_revolution = curr_time;
      Serial.print("Time taken for one revolution: ");
      Serial.println(time_taken_for_one_revolution);
      double RPM = 60.0 / (time_taken_for_one_revolution / 1000.0);
      Serial.print("RPM: ");
      Serial.println(RPM);
    }*/
  long curr_time = millis() - start_time;
  long time_difference = curr_time - prev_time;
  if (time_difference > check_rpm_period) {
    long newPositionLeft = myEncLeft.read();
    long newPositionRight = myEncRight.read();

    prev_time = curr_time;

    int num_pulses_in_period_left = abs(newPositionLeft - oldPositionLeft);
    double pulses_per_second_left = (1000.0 / (double)time_difference) * (double)num_pulses_in_period_left;
    double time_for_one_revolution_left = (double)pulses_per_revolution / pulses_per_second_left;
    RPM_left = 60.0 / time_for_one_revolution_left;

    int num_pulses_in_period_right = abs(newPositionRight - oldPositionRight);
    double pulses_per_second_right = (1000.0 / (double)time_difference) * (double)num_pulses_in_period_right;
    double time_for_one_revolution_right = (double)pulses_per_revolution / pulses_per_second_right;
    RPM_right = 60.0 / time_for_one_revolution_right;

    if (newPositionLeft - oldPositionLeft < 0) {
      RPM_left = RPM_left * -1;
    }
    if (newPositionRight - oldPositionRight < 0) {
      RPM_right = RPM_right * -1;
    }

    Serial.print("RPM_left: ");
    Serial.print(RPM_left);
    Serial.print("  ");

    Serial.print("RPM_right: ");
    Serial.println(RPM_right);

    oldPositionLeft = newPositionLeft;
    oldPositionRight = newPositionRight;
  }
}
