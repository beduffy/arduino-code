#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50
// our servo # counter
uint8_t servonum = 0;

int analog_pulse_width = 100;

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);  // Analog servos run at ~60 Hz updates
}

void loop() {  
  
  //pwm.setPWM(servonum, 0, pulseWidth(90));
  analog_pulse_width = 200;
  for (uint16_t servonum_i = 0; servonum_i < 5; servonum_i++) {
    pwm.setPWM(servonum_i, 0, analog_pulse_width);
    delay(2000);
  }
  /*pwm.setPWM(servonum + 1, 0, analog_pulse_width);
  pwm.setPWM(servonum + 2, 0, analog_pulse_width);*/
  /*Serial.print
  Serial.print(" ");*/
  analog_pulse_width = 300;
  for (uint16_t servonum_i = 0; servonum_i < 5; servonum_i++) {
    pwm.setPWM(servonum_i, 0, analog_pulse_width);
    delay(2000);
  }
  //Serial.println(analog_pulse_width);
  delay(2000);

  //analog_pulse_width += 10;
  /*if (analog_pulse_width > 600) {
    analog_pulse_width = 150;
  }*/
  /*pwm.setPWM(servonum, 0, pulseWidth(135));
  Serial.println("135");
  delay(2000);*/

  //servonum++;
  //if (servonum > 5) servonum = 0;
}

int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
}
