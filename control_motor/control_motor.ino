#include <Encoder.h>
#include <PID_v1.h>

boolean motor_direction = true;  // true is forward
const int m1_in1 = 10;
const int m1_in2 = 11;

const int m2_in1 = 5;
const int m2_in2 = 6;

const int en1_pwm = 9;
const int en2_pwm = 6;

int incomingByte = 0; // for incoming serial data
int delay_amt = 100;

// 8423 pulses per revolution by eye
int pulses_per_revolution = 8423;

//Encoder myEnc(12, 13);  // attempt at polling
Encoder myEnc(2, 3);
long oldPosition  = -999;

// PID variables
boolean result;
double duration, abs_duration;//the number of the pulses
double val_output;//Power supplied to the motor PWM value.
double Setpoint;
double Kp=0.6, Ki=5, Kd=0;
PID myPID(&abs_duration, &val_output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long start_time = millis();
unsigned long last_time_for_revolution = millis();

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

  Setpoint =80;  //Set the output value of the PID
  myPID.SetMode(AUTOMATIC);//PID is set to automatic mode
  myPID.SetSampleTime(100);//Set PID sampling frequency is 100ms
  
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
    /*incomingByte = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);

    int value = atoi(incomingByte);*/
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
    }
  }

  /*analogWrite(en1_pwm, val_output);
  abs_duration = abs(duration);
  result = myPID.Compute(); //PID conversion is complete and returns 1
  if(result)
  {
    Serial.print("Plus: ");
    Serial.println(duration);
    duration = 0; //Count clear, wait for the next count
  }*/

  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
    if (newPosition % pulses_per_revolution == 0) {
      int curr_time = millis();
      int time_taken_for_one_revolution = curr_time - last_time_for_revolution;
      last_time_for_revolution = curr_time;
      Serial.print("Time taken for one revolution: ");
      Serial.println(time_taken_for_one_revolution);
      double RPM = 60.0 / (time_taken_for_one_revolution / 1000.0);
      Serial.print("RPM: ");
      Serial.println(RPM);
    }
  }
}
