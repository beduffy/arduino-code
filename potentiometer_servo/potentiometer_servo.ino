/* Learn how to use a potentiometer to fade an LED with Arduino - Tutorial
   More info and circuit schematic: http://www.ardumotive.com/arduino-tutorials/arduino-fade-led
   Dev: Michalis Vasilakis / Date: 25/10/2014                                                   */

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
//Constants:
const int servoPin = 9;  //pin 9 has PWM function
//const int potPin = A0; //pin A0 to read analog input
const int potPin = 0;

//Variables:
int value; //save analog value


void setup() {
  Serial.begin(9600);
  Serial.println("Began serial");
  myservo.attach(servoPin);
  //Input or output?
  //pinMode(servoPin, OUTPUT);
  pinMode(potPin, INPUT); //Optional
  /*myservo.write(75);
  delay(1000);*/

}

void loop() {
  //Serial.println("loop");
  value = analogRead(potPin);          //Read and save analog value from potentiometer
  Serial.print(value);
  Serial.print(" ");
  value = map(value, 0, 1023, 0, 180); //Map value 0-1023 to 0-180 (PWM)
  Serial.println(value);
  //analogWrite(servoPin, value);          //Send PWM value to led

  
  //
  if (value > 90) {
    value = 90;
  }
  myservo.write(value);



  
  /*myservo.write(30);
  delay(1000);*/
  /*myservo.write(90);
  delay(1000);
  myservo.write(120);
  delay(1000);
  myservo.write(150);
  delay(1000);*/
  /*myservo.write(70);
  delay(1000);
  myservo.write(50);
  delay(1000);
  myservo.write(30);
  delay(1000);
  myservo.write(10);
  delay(1000);
  myservo.write(0);
  delay(1000);*/

  delay(10);                          //Small delay

}
