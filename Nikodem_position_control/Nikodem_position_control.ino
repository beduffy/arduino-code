#include <Encoder.h>
#include <PIDController.h>

volatile long int encoder_pos = 0;
PIDController pos_pid; 
int motor_value = 255;
const int lowest_power_value = 55;
long integerValue = 0;  // Max value is 65535
char incomingByte;


const int en1_pwm = 9;
const int m1_in1 = 10;
const int m1_in2 = 11;

Encoder myEnc(2, 3);

void setup() {

  Serial.begin(9600);
  /*pinMode(2, INPUT);
  pinMode(3, INPUT);*/
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(2), encoder, RISING);

  pos_pid.begin();    
  //pos_pid.tune(15, 0.1, 2000);   // strangely good defaults?
  pos_pid.tune(10, 0.1, 0);
  pos_pid.limit(-255, 255);
  pos_pid.setpoint(8234);
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

    encoder_pos = myEnc.read();
    motor_value = pos_pid.compute(encoder_pos);
    if(motor_value > 0) {
      MotorClockwise(motor_value);
    }
    else {
      MotorCounterClockwise(abs(motor_value));
    }
    Serial.print("motor_value: ");
    Serial.print(motor_value);
    Serial.print(" ");
    Serial.print("encoder_pos: ");
    Serial.println(encoder_pos);
    delay(10);
}

void MotorClockwise(int power) {
  if(power > lowest_power_value) {
    analogWrite(en1_pwm, power);
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, HIGH);
  }
  else {
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, LOW);
  }
}

void MotorCounterClockwise(int power) {
  if(power > lowest_power_value) {
    analogWrite(en1_pwm, power);
    digitalWrite(m1_in1, HIGH);
    digitalWrite(m1_in2, LOW);
  }
  else {
    digitalWrite(m1_in1, LOW);
    digitalWrite(m1_in2, LOW);
  }
}
