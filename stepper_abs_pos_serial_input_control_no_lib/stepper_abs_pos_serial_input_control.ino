/*   
 *   Basic example code for controlling a stepper without library
 *      
 *   by Dejan, https://howtomechatronics.com
 */

// defines pins
#define stepPin 2
#define dirPin 5 

// MS2(-)	MS1(-)	Steps(-)	Interpolation(-)	Mode(-)
// GND	VIO	1⁄2	1⁄256	stealthChop2
// VIO	GND	1⁄4	1⁄256	stealthChop2
// GND	GND	1⁄8	1⁄256	stealthChop2   // nothing connected to ms1 or 2. So 1/8 of 1.8 degrees is 1/8 * 1.8 = 0.225 degrees. so 800 below would be 180 degrees
// VIO	VIO	1⁄16	1⁄256	stealthChop2  // if i wanted max res: 1/16 of 1.8 degrees is (1/16) * 1.8 = 0.1125 degrees...

const float resolution  = 0.225; // put your step resolution here
int step_degree(float desired_degree){
    return abs(desired_degree/resolution);
}

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

float num_degrees_to_change_by;
float incoming_absolute_degrees = 45;
float new_angle_to_change_to;
float cur_abs_degrees = 0;

int calculate_how_many_steps_to_go_to_new_absolute_degrees() {

}

void setup() {
  Serial.begin(9600);
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void loop()  {
  
  recvWithEndMarker();
  if (newData == true) {
        
        newData = false;
        incoming_absolute_degrees = atof(receivedChars);

        // math for num_degrees_to_change_by. steppers are relative but we can track absolute position without skipped steps
        // we start at 0 degrees, if 100 comes in, we should go to +100, if -100 comes in we should go to -100
        
        // e.g. then we are at -100, if +100 comes in we should change by 200 degrees, if -100, then 0, 
        // if 0 comes in, we should change by +100
        // +100 - (-100) = 200
        // -100 - (-100) = 0 stay same
        // 0 - (-100) = 100



        // new_angle_to_change_to = cur_abs_degrees + incoming_absolute_degrees;
        // num_degrees_to_change_by = cur_abs_degrees + new_angle_to_change_to;

        num_degrees_to_change_by = incoming_absolute_degrees - cur_abs_degrees;
        cur_abs_degrees = incoming_absolute_degrees;

        int num_steps = step_degree(num_degrees_to_change_by);
        Serial.print("New incoming angle ");
        Serial.print(receivedChars);
        Serial.print(" New angle to change to ");
        Serial.print(num_degrees_to_change_by);
        Serial.print("  num_degrees_to_change_by: ");
        Serial.println(num_degrees_to_change_by);

        if (num_degrees_to_change_by < 0.0) {
          digitalWrite(dirPin, LOW);
        }
        else {
          digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
        }

        for(int x = 0; x < num_steps; x++) {
          digitalWrite(stepPin,HIGH); 
          delayMicroseconds(300);    // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(stepPin,LOW); 
          delayMicroseconds(300); 
        }
  }
}