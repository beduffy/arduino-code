#include <Encoder.h>

int yellow_encoder = A5;
int green_encoder = A4;

//int digital_encoder_pin = 2;

Encoder myEnc(2,3);
long oldPosition  = -999;

const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = 3;//B pin -> the digital pin 3
byte encoder0PinALast;
int duration;//the number of the pulses
boolean Direction;//the rotation direction


unsigned long startTimeA = 0;    // start timing A interrupts  
unsigned long startTimeB = 0;    // start timing B interrupts  
unsigned long countIntA = 0;     // count the A interrupts  
unsigned long countIntB = 0;     // count the B interrupts  
const unsigned long INT_COUNT = 20;     // sufficient interrupts for accurate timing
unsigned long nowTime = 0;       // updated on every loop
double inputA = 0;              // input is PWM to motors  
double outputA = 0;             // output is rotational speed in Hz  
double inputB = 0;              // input is PWM to motors  
double outputB = 0;             // output is rotational speed in Hz  


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  Serial.println("Starting setup");

  //pinMode(yellow_encoder, INPUT_PULLUP);
  //pinMode(green_encoder, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(yellow_encoder), isr_A, RISING);  
  //attachInterrupt(digitalPinToInterrupt(green_encoder), isr_B, RISING);

  //attachInterrupt(digitalPinToInterrupt(yellow_encoder), isr_A, CHANGE);  

  //attachInterrupt(digitalPinToInterrupt(digital_encoder_pin), isr_A, CHANGE);  


  //Direction = true;//default -> Forward
  //pinMode(encoder0pinB, INPUT);
  //attachInterrupt(0, wheelSpeed, CHANGE);
  //attachInterrupt(0, wheelSpeed, FALLING);
}

void wheelSpeed()
{
  //Serial.println("wheelSpeed()");
  int Lstate = digitalRead(encoder0pinA);
  //Serial.print("Lstate: ");
  //Serial.println(Lstate);
  if((encoder0PinALast == LOW) && Lstate==HIGH)
  {
    //Serial.print("encoder0PinALast: ");
    //Serial.println(encoder0PinALast);
    int val = digitalRead(encoder0pinB);
    if(val == LOW && Direction)
    {
      Serial.println("direction: reverse");
      Direction = false; //Reverse
    }
    else if(val == HIGH && !Direction)
    {
      Serial.println("direction: forward");
      Direction = true;  //Forward
    }
  }
  encoder0PinALast = Lstate;

  if(!Direction) {
    duration++;
    //Serial.print("duration: ");
    //Serial.println(duration);
  }
  else {
    duration--;
  }
}

void isr_A(){  
  Serial.println("isr_A");
  // count sufficient interrupts to get accurate timing  
  // inputX is the encoder frequency in Hz  
  countIntA++;  
  if (countIntA == INT_COUNT){  
    inputA = (float) INT_COUNT * 1000 / (float)(nowTime - startTimeA);  
    startTimeA = nowTime;  
    countIntA = 0;  
  }
}  
void isr_B(){
  Serial.println("isr_B");
  // count sufficient interrupts to get accurate timing  
  // inputX is the encoder frequency in Hz  
  countIntB++;  
  if (countIntB == INT_COUNT){  
    inputB = (float) INT_COUNT * 1000 / (float)(nowTime - startTimeB);  
    startTimeB = nowTime;  
    countIntB = 0;  
  }  
}  

void loop() {
  /*int yellowSensorValue = analogRead(yellow_encoder);
  int greenSensorValue = analogRead(green_encoder);*/
  
  /*Serial.print("Yellow: ");
  Serial.print(yellowSensorValue);
  Serial.print(" Green: ");
  Serial.println(greenSensorValue);*/

  //Serial.print("Pulse: ");
  /*Serial.println(duration);
  duration = 0;*/

  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
  
  delay(100);
  //#delay(50);
}
