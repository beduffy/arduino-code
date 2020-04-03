#include <SoftwareSerial.h>
 
// Initializing communication ports
//SoftwareSerial mySerial(11, 10); // TX/RX pins
SoftwareSerial mySerial(0, 1);
 
void setup()  
{
  Serial.begin(9600);
  mySerial.begin(9600);
}
 
String getMessage(){
  String msg = "";
  char a;
  
  while(mySerial.available()) {
      a = mySerial.read();
      msg+=String(a);
  }
  return msg;
}
 
void loop()
{
    // Check if a message has been received
    String msg = getMessage();
    if(msg!=""){
      Serial.println(msg);
    }
 
    // Send a string when 'm' is sent through the Serial
    if(Serial.available()){
      if(Serial.read()=='m'){
        mySerial.println("HC-06 IS REPLYING :-)");
      }
    }
 
    delay(10);
}
