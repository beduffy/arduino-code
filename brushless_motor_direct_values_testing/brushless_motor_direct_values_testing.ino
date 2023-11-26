/*
        Arduino Brushless Motor Control
     by Dejan, https://howtomechatronics.com
*/

#include <Servo.h>

Servo ESC;     // create servo object to control the ESC

int potValue;  // value from the analog pin

void setup() {
  Serial.begin(9600);
  // Attach the ESC on pin 9
  // ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC.attach(9,1000,2000);

  ESC.writeMicroseconds(2000);
  delay(2000);
  ESC.writeMicroseconds(1000);
  delay(2000);
  ESC.writeMicroseconds(1500);
  delay(2000);
  ESC.writeMicroseconds(1000);
  delay(2000);
}

void loop() {
  potValue = 0;
  // ESC.write(potValue);    // Send the signal to the ESC
  ESC.writeMicroseconds(1000);
}