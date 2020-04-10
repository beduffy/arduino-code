#include <Encoder.h>
#include <PIDController.h>

PIDController pos_pid; 
int motor_value = 255;
const int lowest_power_value = 55;
long integerValue = 0;  // Max value is 65535
char incomingByte;
const int check_rpm_period = 100;

// 8423 pulses per revolution by eye
int pulses_per_revolution = 8423;
double RPM = 1;
long oldPosition  = 0;

unsigned long start_time = millis();
unsigned long last_time_for_revolution = millis();
unsigned long prev_time = start_time;

const int en1_pwm = 9;
const int m1_in1 = 10;
const int m1_in2 = 11;

Encoder myEnc(2, 3);

void setup() {

  Serial.begin(9600);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(en1_pwm, OUTPUT);

  pos_pid.begin();    
  //pos_pid.tune(15, 0.1, 2000);   // strangely good defaults?
  pos_pid.tune(10, 3.3, 0);
  pos_pid.limit(-255, 255);
  pos_pid.setpoint(6);
}

void loop() {
    if (Serial.available() > 0) {  
        integerValue = 0;
    
        integerValue = Serial.parseInt();
        if (integerValue != 0) {
          Serial.print("New setpoint: ");
          Serial.println(integerValue);
          pos_pid.setpoint(integerValue);
        }
    }

    long newPosition = myEnc.read();
    long curr_time = millis() - start_time;
    long time_difference = curr_time - prev_time;
    if (time_difference > check_rpm_period) {
      prev_time = curr_time;
      
      int num_pulses_in_period = abs(newPosition - oldPosition);
      double pulses_per_second = (1000.0 / (double)time_difference) * (double)num_pulses_in_period;
      double time_for_one_revolution = (double)pulses_per_revolution / pulses_per_second;
      RPM = 60.0 / time_for_one_revolution;

      if (newPosition - oldPosition > 0) {
      }
      else if (newPosition - oldPosition < 0) {
        RPM = RPM * -1;
      }
      
      motor_value = pos_pid.compute(RPM);
      if(motor_value > 0) {
        MotorCounterClockwise(abs(motor_value));
      }
      else {
        MotorClockwise(abs(motor_value));
      }
      Serial.print("RPM: ");
      Serial.print(RPM);
      Serial.print("  ");
      Serial.print("motor_value: ");
      Serial.println(motor_value);
      /*Serial.print(" ");
      Serial.print("newPosition: ");
      Serial.println(newPosition);*/

      oldPosition = newPosition;
    }
    
    //delay(10);
}

void MotorClockwise(int power) {
  //if(power > lowest_power_value) {
    analogWrite(en1_pwm, power);
    digitalWrite(m1_in1, HIGH);
    digitalWrite(m1_in2, LOW);
  /*}
  else {
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, LOW);
  }*/
}

void MotorCounterClockwise(int power) {
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
