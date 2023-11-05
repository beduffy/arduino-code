
/*   
 *   Basic example code for controlling a stepper with the AccelStepper library
 *      
 *   by Dejan, https://howtomechatronics.com
 */

// #include <AccelStepper.h>

// // Define the stepper motor and the pins that is connected to
// AccelStepper stepper1(1, 2, 5); // (Type of driver: with 2 pins, STEP, DIR)
// // AccelStepper stepper1(1, 5, 2); 

// void setup() {
//   // Set maximum speed value for the stepper
//   stepper1.setMaxSpeed(300);
// }

// void loop() {
//   stepper1.setSpeed(1);
//   // Step the motor with a constant speed previously set by setSpeed();
//   stepper1.runSpeed();
// }


// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
 
#include <AccelStepper.h>
 
AccelStepper stepper(1, 2, 5); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
 
void setup()
{  
   stepper.setMaxSpeed(1000);
   stepper.setSpeed(300);
}
 
void loop()
{  
   stepper.runSpeed();
}