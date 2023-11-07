/*   
 *   Basic example code for controlling a stepper without library
 *      
 *   by Dejan, https://howtomechatronics.com
 */
#include <AccelStepper.h>
 
// Define a stepper and the pins it will use
AccelStepper stepper;

// MS2(-)	MS1(-)	Steps(-)	Interpolation(-)	Mode(-)
// GND	VIO	1⁄2	1⁄256	stealthChop2
// VIO	GND	1⁄4	1⁄256	stealthChop2
// GND	GND	1⁄8	1⁄256	stealthChop2   // nothing connected to ms1 or 2. So 1/8 of 1.8 degrees is 1/8 * 1.8 = 0.225 degrees. so 800 below would be 180 degrees
// VIO	VIO	1⁄16	1⁄256	stealthChop2  // if i wanted max res: 1/16 of 1.8 degrees is (1/16) * 1.8 = 0.1125 degrees...

const float resolution  = 0.225; // put your step resolution here
int step_degree(float desired_degree){
    return desired_degree / resolution;
}

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

float num_degrees_to_change_by;
float incoming_relative_degrees = 0;
float new_angle_to_change_to;
int incoming_relative_steps = 0;
float cur_abs_degrees = 0;


void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(4000);
  stepper.setSpeed(4000);
  // stepper.setAcceleration(500);
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

void loop()
{
  
  recvWithEndMarker();
  if (newData == true) {
        
        newData = false;
        incoming_relative_degrees = atof(receivedChars);

        // math for num_degrees_to_change_by. steppers are relative but we can track absolute position without skipped steps
        // we start at 0 degrees, if 100 comes in, we should go to +100, if -100 comes in we should go to -100
        
        // e.g. then we are at -100, if +100 comes in we should change by 200 degrees, if -100, then 0, 
        // if 0 comes in, we should change by +100
        // +100 - (-100) = 200
        // -100 - (-100) = 0 stay same
        // 0 - (-100) = 100

        num_degrees_to_change_by = incoming_relative_degrees;
        incoming_relative_steps = step_degree(incoming_relative_degrees);
        // cur_abs_degrees = incoming_relative_degrees;

        // int num_steps = step_degree(num_degrees_to_change_by);
        Serial.print("New incoming angle ");
        Serial.println(receivedChars);
        // Serial.print("  num_degrees_to_change_by: ");
        // Serial.println(num_degrees_to_change_by);
        stepper.move(incoming_relative_steps);
  }


  // stepper.setCurrentPosition(incoming_relative_steps);  // homes so nope

  // TODO would it be possible to go straight from pixel horizontal dist to the relative angle? study pinhole and do experiments and calculate foV horizontal

  // incoming_relative_steps = 1000;
  // stepper.moveTo(incoming_relative_steps);
  // stepper.move(incoming_relative_steps);

  if (stepper.distanceToGo() == 0)
    {
      // stepper.setCurrentPosition(incoming_relative_steps);
  //       // Random change to speed, position and acceleration
  //       // Make sure we dont get 0 speed or accelerations
  //       delay(1000);
  //       stepper.moveTo(rand() % 1000);
    }

  

    // TODO PID position control is too jagged, should be velocity control instead?

  // int chosen_speed = 4000;
  int steps_to_go = stepper.distanceToGo();
  // Serial.println(steps_to_go);
  // if (steps_to_go > 0) {
  //   stepper.setSpeed(chosen_speed);
  // }
  // else if (steps_to_go < 0) {
  //   stepper.setSpeed(-chosen_speed);
  // }
  // else {
  //   stepper.setSpeed(0);
  // }

  stepper.setSpeed(1000);
  
  // stepper.setSpeed(1000);
  bool output = stepper.runSpeedToPosition();
  // bool output = stepper.runSpeed();
  // bool output = stepper.run();
  // Serial.println(output);
}