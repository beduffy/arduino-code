// -*- mode: C++ -*-
//
// Make a single stepper follow the analog value read from a pot or whatever
// The stepper will move at a constant speed to each newly set posiiton, 
// depending on the value of the pot.
//
// Copyright (C) 2012 Mike McCauley
// $Id: ProportionalControl.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
 
#include <AccelStepper.h>
 
// Define a stepper and the pins it will use
AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
 
// This defines the analog input pin for reading the control voltage
// Tested with a 10k linear pot between 5v and GND
#define ANALOG_IN A0
 
void setup()
{  
  stepper.setMaxSpeed(1000);
}
 
// TODO actually connect potentiometer!! and do this for speed control too
// TODO actually store current angle, and then build script that is always going towards desired angle. 
// TODO connect webcam pan with hack to stepper and make it rotate 360 degrees and track me very fast.
// TODO modulus degrees so 360 and 1 degree are 1 degree apart
// TODO print coupling plate for webcam
// TODO flask to jetson to serial? Or is there another way?
// TODO HOw to make acceleration and actual PID with stepper motors?
// TODO how to find all bottle necks, remove them and make things extremely fast?

void loop()
{
  // Read new position
  // int analog_in = analogRead(ANALOG_IN);
  int analog_in = 1000;
  stepper.moveTo(analog_in);
  stepper.setSpeed(100);
  stepper.runSpeedToPosition();
}
