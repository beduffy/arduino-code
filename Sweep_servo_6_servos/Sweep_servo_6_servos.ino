/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo3;
Servo myservo5;
Servo myservo6;
Servo myservo9;  // create servo object to control a servo
Servo myservo10;
Servo myservo11;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int delayAmt = 1000;
int testVal = 500;

void setup() {
  Serial.begin(9600);
  Serial.println("Began serial");
  myservo3.attach(3);
  myservo5.attach(5);
  myservo6.attach(6);
  myservo9.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo10.attach(10);
  myservo11.attach(11);
}

void loop() {
  Serial.println("start of loop");
  /*for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(delayAmt);                       // waits 15ms for the servo to reach the position
  }*/

  delay(delayAmt);
  pos = 60;
  myservo3.write(pos);
  myservo5.write(pos);
  myservo6.write(pos);
  myservo9.write(pos);
  myservo10.write(pos);
  myservo11.write(pos);
  delay(delayAmt);
  pos = 130;
  myservo3.write(pos);
  myservo5.write(pos);
  myservo6.write(pos);
  myservo9.write(pos);
  myservo10.write(pos);
  myservo11.write(pos);
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
