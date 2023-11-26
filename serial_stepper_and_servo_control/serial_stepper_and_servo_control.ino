#include <AccelStepper.h>
 
// Define a stepper and the pins it will use
AccelStepper stepper(1, 2, 5);  // TODO before it was without 1, 2,5 and degrees were wrong? Check if it still works with python. Maybe pinhole divisoon by 5 was wrong

#include <Servo.h>

Servo tilt_servo; 

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
char receivedCharsAfterFirst[numChars];

boolean newData = false;

float num_degrees_to_change_by;
float incoming_relative_degrees = 0;
float incoming_servo_degrees;
float new_angle_to_change_to;
int incoming_relative_steps = 0;
float cur_abs_degrees = 0;


void setup() {
  Serial.begin(115200);
  stepper.setMaxSpeed(4000);
  stepper.setSpeed(4000);
  tilt_servo.attach(9);
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    // TODO time everything, is the stepper slow or serial connection slow? 
    // https://arduino.stackexchange.com/questions/9666/how-to-avoid-blocking-while-loop-reading-serial
    // https://forum.arduino.cc/t/serial-input-basics/278284/5
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
        if (receivedChars[0] == 'a') {
          strcpy(receivedCharsAfterFirst, receivedChars + 1);
          incoming_relative_degrees = atof(receivedCharsAfterFirst);

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
          Serial.print("New incoming angle for stepper ");
          Serial.println(receivedCharsAfterFirst);
          // Serial.print("  num_degrees_to_change_by: ");
          // Serial.println(num_degrees_to_change_by);
          stepper.move(incoming_relative_steps);
        }
        else if (receivedChars[0] == 'b') {
          strcpy(receivedCharsAfterFirst, receivedChars + 1);
          incoming_servo_degrees = atof(receivedCharsAfterFirst);
          Serial.print("New incoming angle for servo");
          Serial.println(receivedCharsAfterFirst);
          tilt_servo.write(incoming_servo_degrees);
        }
        else {
          Serial.println("You need to begin string with a or b");
        }
  }

    // TODO PID position control is too jagged, should be velocity control instead?

  // int chosen_speed = 4000;
  int steps_to_go = stepper.distanceToGo();

  stepper.setSpeed(2000);
  
  bool output = stepper.runSpeedToPosition();
}