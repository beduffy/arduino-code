/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int delayAmt = 1000;
int testVal = 500;

void setup() {
  Serial.begin(9600);
  Serial.println("Began serial");
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.print("Hello");
  myservo.write(pos);
  delay(delayAmt);
}

void loop() {
  Serial.println("start of loop");
  /*for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(delayAmt);                       // waits 15ms for the servo to reach the position
  }*/
  
  
  /*delay(delayAmt);
  myservo.write(0);
  Serial.println(
  delay(delayAmt);
  myservo.write(45);
  delay(delayAmt);
  myservo.write(90);
  delay(delayAmt);
  myservo.write(135);
  delay(delayAmt);
  myservo.write(180);*/

  delay(delayAmt);
  myservo.write(75);
  delay(delayAmt);
  myservo.write(90);
  delay(delayAmt);

  
  /*myservo.writeMicroseconds(testVal);
  delay(delayAmt);
  testVal += 50;
  Serial.println(testVal);*/
  
  /*myservo.writeMicroseconds(1000);
  delay(delayAmt);
  myservo.writeMicroseconds(1500);
  delay(delayAmt);
  myservo.writeMicroseconds(2000);
  delay(delayAmt);*/
  
  /* Serial.println("Done first clockwise rotation");
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(delayAmt);                       // waits 15ms for the servo to reach the position
  }*/
  Serial.println("end of loop");
}
