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

float num_degrees = 45;
const float resolution  = 0.225; // put your step resolution here
int step_degree(float desired_degree){
    return (desired_degree/resolution);
}

void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
}
void loop()  {
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  int num_steps = step_degree(num_degrees);

  Serial.print("Num steps for ");
  Serial.print(num_degrees);
  Serial.print(" is: ");
  Serial.println(num_steps);

  for(int x = 0; x < num_steps; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(300);    // by changing this time delay between the steps we can change the rotation speed
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(300); 
  }
  delay(2500); // One second delay

}