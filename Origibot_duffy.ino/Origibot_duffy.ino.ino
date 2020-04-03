

 /*******************************************************************************************
* File Name          : Origibot20.ino
* Author             : Richard D. Laboris, Origin Robotics, Inc.
* Date               : 4/4/2019
* Description        : Origibot2 Robot Interface for Control via BlueTooth
* 
* 24 Commands Defined:  ,B,C,D,E,F,G,H,I,J,K,L, ,N,O,P,Q,R,S,T,U,V,W,X,Y,Z
*
*   S: Chassis Stop
*   L: Chassis Left
*   K: Chassis Forward+Left
*   F: Chassis Forward
*   J: Chassis Forward+Right
*   R: Chassis Right
*   Q: Chassis Backward+Right
*   B: Chassis Backward
*   E: Chassis Backward+Left
*   N: Neck Up
*   V: Neck Down
*   X: Neck Stop
*   C: Arm Up -- Was 'A' but caused bug on startup?
*   Z: Arm Down
*   I: Arm Stop
*   G: Gripper Open
*   T: Gripper Close
*   Y: Gripper Stop
*   W: Wrist Rotate Left
*   O: Wrist Rotate Right
*   P: Wrist Rotate Stop
*   U: Wrist Tilt Up
*   H: Wrist Tilt Down
*   D: Wrist Tilt Stop
*
* Select Arduino Genuino Uno Board
********************************************************************************************/
/********************************************************************************************
* Servo Range based on Arduino Min_Pulse_Width=544; Max_Pulse_Width=2400 
* Corresponding to Angles of 0 and 180 deg respectively
********************************************************************************************/
#include <MeOrion.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

//Makeblock Module Port Initialization
MeDCMotor RightMotor(M1);                     //Right Motor
MeDCMotor LeftMotor(M2);                      //Left Motor
MeBluetooth bluetooth(PORT_5);                //Bluetooth Module
MePort ArmRelay(PORT_7);                      //Arm Servo Power Relay
MePort ServosRelay(PORT_8);                   //Other Servos Power Relays
MePort ServoPort3(PORT_3);                    //Servo Port 3
MePort ServoPort4(PORT_4);                    //Servo Port 4
MePort ServoPort6(PORT_6);                    //Servo Port 6
Servo NeckServo;                              //Neck Servo
Servo ArmServo;                               //Arm Servo
Servo WristTiltServo;                         //Wrist Tilt Servo
Servo GripperServo;                           //Gripper Servo
Servo WristServo;                             //Wrist Servo
int16_t ServoPort3Pin1 =  ServoPort3.pin1();  //attaches PORT_3 Pin1
int16_t ServoPort3Pin2 =  ServoPort3.pin2();  //attaches PORT_3 Pin2
int16_t ServoPort4Pin2 =  ServoPort4.pin2();  //attaches PORT_4 Pin2
int16_t ServoPort6Pin1 =  ServoPort6.pin1();  //attaches PORT_6 Pin1
int16_t ServoPort6Pin2 =  ServoPort6.pin2();  //attaches PORT_6 Pin2

//Chassis constants/variables
//int DriveSpeed = -150;                   //Max=-255
int DriveSpeed = -45;
//int DriveSpeed = -165;              //Blue Robot has 125RPM motor vs 80RMP, Max=-255

int LeftDirection = 0;                   //LastDirection of Left motor travel, 1=Forward, -1=Backward, 0=Other
int RightDirection = 0;                  //LastDirection of Right motor travel, 1=Forward, -1=Backward, 0=Other
float TurnSpeedMultiplier = 1;           //TurnSpeed is this multiple of DriveSpeed
float MovingTurnSpeedMultiplier = .5;   //During Forward+Turn, turning side wheel will spin at this multiple of DriveSpeed

//Neck Servo constants/variables
int NeckMinPos = 0;                      //Neck tilted back
int NeckMaxPos = 180;                    //Neck tilted forward
int NeckCenterPos = 90;                  //Neck at center
int NeckCurrPos = NeckCenterPos;
int NeckTargetPos = NeckCurrPos;
int NeckServoChangeDelay = 30;           //ms delay for slowing change in servo angle

//Arm Servo constants/variables           //These are different due to discrepancy in Servo extreme position values
int ArmMinPos = 75;                       //Highest Arm Position - Red Robot - Ben Duffy
int ArmMaxPos = 130;                      //Lowest Arm Position  - Red Robot - Ben Duffy

int ArmCurrPos = ArmMaxPos; 
int ArmTargetPos = ArmCurrPos;
int ArmServoChangeDelay = 100;            //ms delay for slowing change in servo angle

//Wrist Tilt constants/variables
int WristTiltMinPos = 10;                //Wrist tilted up
int WristTiltMaxPos = 160;               //Wrist tilted down
int WristTiltCenterPos = 40; //was 36    //Wrist at horizontal positiong
int WristTiltCurrPos = WristTiltCenterPos;
int WristTiltTargetPos = WristTiltCurrPos;
int WristTiltServoChangeDelay = 15;      //ms delay for slowing change in servo angle

//Wrist Rotation Servo constants/variables
int WristMinPos = 0;                     //Wrist rotated counter-clockwise
int WristMaxPos = 180;                   //Wrist rotated clockwise
int WristCenterPos = 80;                 //Wrist at center position
int WristCurrPos = WristCenterPos;
int WristTargetPos = WristCurrPos;
int WristServoChangeDelay = 15;          //ms delay for slowing change in servo angle

//Gripper Servo constants/variables
int GripperMinPos = 50;                  //Gripper Fully Opened
int GripperMaxPos = 165;                 //Gripper Fully Closed
int GripperCurrPos = GripperMinPos;      //?
int GripperTargetPos = GripperCurrPos;
int GripperServoChangeDelay = 15;        //ms delay for slowing change in servo angle

//Miscellaneous variables
char inDat = 0;                          //Bluetoothe data read
int CharsAvail;                          //Number of Characters in Bluetooth Buffer
int AddServoChangeDelay = 0;             //Amount of delay to be added during loop
int ServoEnabled = 0;                    //Flag to indicate Servos have been enabled
int SoftDelay = 100; //was 100            //Amount of delay (ms) between iterations of soft starts/stops

//Variables used for auto-adjusting Wrist in response to Arm movement -- tilt adjust varies depending on arm angle

int ArmTotalSweep = ArmMaxPos-ArmMinPos;                      //Total Arm Servo sweep -- Approx = 55
int WristTiltTotalSweep = WristTiltMaxPos-WristTiltMinPos-20; //Total WristTilt Servo sweep -- Approx = 160-10-20=130
float WristTiltArmAdjust = WristTiltTotalSweep/ArmTotalSweep; //was 2.5; //Keeps Wrist level while Arm moves, set to 0 for non-leveling action -- Approx 130/55 = 2.36
int LoopsSinceArmDirectionChanged = 0;                        //Only level wrist after a few Arm Change Iterations to prevent wrist over-compenstation
int ArmWristAdjustTopBuffer = 0;  //5;                              //Wrist Tilt position is not adjusted below ArmMinPos+ArmWristAdjustTopBuffer
int ArmWristAdjustBottomBuffer = 10;                           //Wrist Tilt position is not adjusted above ArmMaxPos-ArmWristAdjustBottomBuffer
int ArmWristDownLoopsBuffer = 0;  //2;                            //Number of loops of Arm change before wrist will begin compensation
int ArmWristUpLoopsBuffer = 0; //7;                               //Number of loops of Arm change before wrist will begin compensation

//Heartbeat signal variables used to prevent collisions after remote disconnect
unsigned long LastHeartBeat = 0;         //Milliseconds since last HeartBeat signal was detected
unsigned long HeartBeatInterval = 1500;  //HeartBeat signal must be received within this many milliseconds
unsigned long currentMillis = 0;         //Store current clock time
bool heartbeatactive = false;           //Only activate heartbeat function if triggered by initial heartbeat '@' from remote to allow backwards compatibility

void setup()
{
  bluetooth.begin(115200);               //Set baud rate
  WristServo.attach(ServoPort6Pin1);     //Attach to Wrist Rotation Servo  
  GripperServo.attach(ServoPort6Pin2);   //Attach to Gripper Servo
  ArmServo.attach(ServoPort3Pin1);       //Attach to Arm Servo
  NeckServo.attach(ServoPort3Pin2);      //Attach to Neck Servo  
  WristTiltServo.attach(ServoPort4Pin2); //Attach to Wrist Tilt Servo
  NeckServo.write(NeckCurrPos);          //Set startup values
  ArmServo.write(ArmCurrPos);
  WristTiltServo.write(WristTiltCurrPos);
  GripperServo.write(GripperCurrPos);
  WristServo.write(WristCurrPos);
  LeftMotor.run(0);                      //Stop Left Motor
  RightMotor.run(0);                     //Stop Right Motor  
  LastHeartBeat = millis();              //Initialize LastHeartBeat to current millis
}

 void loop()                             //Loop continuously
 {
    
  currentMillis = millis();              //Used to determine when to check for HeartBeat signal
  if(currentMillis < 5000){//Disable Motors while booting (4 seconds)
    LeftMotor.run(0);                      //Stop Left Motor
    RightMotor.run(0);                     //Stop Right Motor
  }
  CharsAvail = bluetooth.available();    //Check for data in bluetooth buffer
  if(CharsAvail > 0){
    inDat = bluetooth.read();            //Read one byte of data from bluetooth buffer
      switch (inDat) {

        case 64:           //HeartBeat Signal Received
          heartbeatactive = true;           //Activate the heartbeat function
          LastHeartBeat = millis();         //Set LastHeartBeat var
          break;      
  
        //Chassis Commands:      
        case 'S':                           //Chassis Stop
          // Add Soft Stop
          //*          
          for(int x = 9; x >= 1; x--){      //10 Loops with Softdelay between each
            LeftMotor.run(LeftDirection*DriveSpeed*(x*.1));
            RightMotor.run(RightDirection*DriveSpeed*(x*.1));
            delay(SoftDelay);
          }
          //*/           
          LeftMotor.run(0);
          RightMotor.run(0);
          LeftDirection=0;
          RightDirection=0;
          break;
        case 'L':                           //Chassis Left
          // Add Soft Start
          /*
          for(int x = 1; x <= 9; x++){
            LeftMotor.run(-DriveSpeed*TurnSpeedMultiplier*(x*.1));
            RightMotor.run(DriveSpeed*TurnSpeedMultiplier*(x*.1));
            delay(SoftDelay);
          }
          //*/ 
          LeftMotor.run(-DriveSpeed*TurnSpeedMultiplier);
          RightMotor.run(DriveSpeed*TurnSpeedMultiplier);
          LeftDirection=-1;
          RightDirection=1;
          break;
        case 'K':                           //Chassis Forward+Left
          /*
          // Add Soft Start
          for(int x = 1; x <= 9; x++){
            LeftMotor.run(DriveSpeed*TurnSpeedMultiplier*(x*.1));
            RightMotor.run(DriveSpeed*(x*.1));
            delay(SoftDelay);
          }
          //*/        
          LeftMotor.run(DriveSpeed*MovingTurnSpeedMultiplier);
          RightMotor.run(DriveSpeed);
          LeftDirection=1;
          RightDirection=1;               
          break;         
        case 'F':                           //Chassis Forward
          /*
          // Add Soft Start
          for(int x = 1; x <= 9; x++){
            LeftMotor.run(DriveSpeed*(x*.1));
            RightMotor.run(DriveSpeed*(x*.1));
            delay(SoftDelay);
          }
          //*/
          LeftMotor.run(DriveSpeed);
          RightMotor.run(DriveSpeed);
          LeftDirection=1;
          RightDirection=1;
          break;
        case 'J':                           //Chassis Forward+Right
          /*
          // Add Soft Start
          for(int x = 1; x <= 9; x++){
            LeftMotor.run(DriveSpeed*(x*.1));
            RightMotor.run(DriveSpeed*TurnSpeedMultiplier*(x*.1));
            delay(SoftDelay);
          }
          //*/        
          LeftMotor.run(DriveSpeed);
          RightMotor.run(DriveSpeed*MovingTurnSpeedMultiplier);
          LeftDirection=1;
          RightDirection=1;   
          break;
        case 'R':                           //Chassis Right
          /*
          // Add Soft Start
          for(int x = 1; x <= 9; x++){
            LeftMotor.run(DriveSpeed*TurnSpeedMultiplier*(x*.1));
            RightMotor.run(-DriveSpeed*TurnSpeedMultiplier*(x*.1));
            delay(SoftDelay);
          }
          //*/      
          LeftMotor.run(DriveSpeed*TurnSpeedMultiplier);
          RightMotor.run(-DriveSpeed*TurnSpeedMultiplier);
          LeftDirection=1;
          RightDirection=1;        
          break;
        case 'Q':                           //Chassis Backward+Right
          /*
          // Add Soft Start
          for(int x = 1; x <= 9; x++){
            LeftMotor.run(-DriveSpeed*(x*.1));
            RightMotor.run(-DriveSpeed*TurnSpeedMultiplier*(x*.1));
            delay(SoftDelay);
          }
          //*/        
          LeftMotor.run(-DriveSpeed);
          RightMotor.run(-DriveSpeed*MovingTurnSpeedMultiplier);
          LeftDirection=-1;
          RightDirection=-1;     
          break;
        case 'B':                           //Chassis Backward
          /*        
          // Add Soft Start
          for(int x = 1; x <= 9; x++){
            LeftMotor.run(-DriveSpeed*(x*.1));
            RightMotor.run(-DriveSpeed*(x*.1));
            delay(SoftDelay);
          }
          //*/                   
          LeftMotor.run(-DriveSpeed);
          RightMotor.run(-DriveSpeed);
          LeftDirection=-1;
          RightDirection=-1;
          break;
        case 'E':                           //Chassis Backward+Left
          /*
          // Add Soft Start
          for(int x = 1; x <= 9; x++){
            LeftMotor.run(-DriveSpeed*TurnSpeedMultiplier*(x*.1));
            RightMotor.run(-DriveSpeed*(x*.1));
            delay(SoftDelay);
          }
          //*/        
          LeftMotor.run(-DriveSpeed*MovingTurnSpeedMultiplier);
          RightMotor.run(-DriveSpeed);
          LeftDirection=-1;
          RightDirection=-1;       
          break;
         
        //Servo Commands:        
        //(Servo position changes handled after end of switch block)
        
        case 'N':                           //Neck Up
          NeckTargetPos = NeckMinPos;
          if (ServoEnabled==0){ // If Servos have not yet been enabled, enable them
              ServosRelay.aWrite2(0);                        // Turn other Servos ON
              ServoEnabled = 1;
          }          
          break; 
        case 'V':                           //Neck Down
          NeckTargetPos = NeckMaxPos;
          if (ServoEnabled==0){ // If Servos have not yet been enabled, enable them
              ServosRelay.aWrite2(0);                        // Turn other Servos ON
              ServoEnabled = 1;
          }          
          break;
         case 'X':                          //Neck Stop
          NeckTargetPos = NeckCurrPos;
          break;               
        case 'C':                           //Arm Up
          ArmTargetPos = ArmMinPos;
          LoopsSinceArmDirectionChanged = 1;
          break;
        case 'Z':                           //Arm Down
          ArmTargetPos = ArmMaxPos;
          LoopsSinceArmDirectionChanged = 1;
          break;
        case 'I':                           //Arm Stop
          ArmTargetPos = ArmCurrPos;
          LoopsSinceArmDirectionChanged = 0;
          break;        
        case 'T':                           //Gripper Close
          GripperTargetPos = GripperMinPos;
          if (ServoEnabled==0){ // If Servos have not yet been enabled, enable them
              ServosRelay.aWrite2(0);                        // Turn other Servos ON
              ServoEnabled = 1;
          }          
          break;
        case 'G':                           //Gripper Open
          GripperTargetPos = GripperMaxPos;
          if (ServoEnabled==0){ // If Servos have not yet been enabled, enable them
              ServosRelay.aWrite2(0);                        // Turn other Servos ON
              ServoEnabled = 1;
          }          
          break;
        case 'Y':                           //Gripper Stop
          GripperTargetPos = GripperCurrPos;
          break;      
        case 'W':                           //Wrist Rotate Left
          WristTargetPos = WristMinPos;
          if (ServoEnabled==0){ // If Servos have not yet been enabled, enable them
              ServosRelay.aWrite2(0);                        // Turn other Servos ON
              ServoEnabled = 1;
          }          
          break;
        case 'O':                           //Wrist Rotate Right
          WristTargetPos = WristMaxPos;
          if (ServoEnabled==0){ // If Servos have not yet been enabled, enable them
              ServosRelay.aWrite2(0);                        // Turn other Servos ON
              ServoEnabled = 1;
          }          
          break; 
        case 'P':                           //Wrist Rotate Stop
          WristTargetPos = WristCurrPos;
          break;         
        case 'U':                           //Wrist Tilt Up
          WristTiltTargetPos = WristTiltMinPos;
          if (ServoEnabled==0){ // If Servos have not yet been enabled, enable them
              ServosRelay.aWrite2(0);                        // Turn other Servos ON
              ServoEnabled = 1;
          }          
          break;
        case 'H':                           //Wrist Tilt Down
          WristTiltTargetPos = WristTiltMaxPos;
          if (ServoEnabled==0){ // If Servos have not yet been enabled, enable them
              ServosRelay.aWrite2(0);                        // Turn other Servos ON
              ServoEnabled = 1;
          }          
          break; 
        case 'D':                           //Wrist Tilt Stop
          WristTiltTargetPos = WristTiltCurrPos;
          break;
        default:                           //nothing matched, ignore
          break;
      }  //End switch (inDat)
  }    //End if(CharsAvail > 0)
  
    //If heartbeatactive, Check if HeartBeat has been received since Interval
    
  if(heartbeatactive && (currentMillis - LastHeartBeat > HeartBeatInterval)) { //Expected HeartBeat was not received, stop Robot
    LeftMotor.run(0);
    RightMotor.run(0);
    //LastDirection=0;     
    NeckTargetPos = NeckCurrPos;            //Stop Neck in case it is moving
    ArmTargetPos = ArmCurrPos;              //Stop Arm in case it is moving
    GripperTargetPos = GripperCurrPos;      //Stop Gripper in case it is moving
    WristTargetPos = WristCurrPos;          //Stop Wrist Rotation in case it is moving
    WristTiltTargetPos = WristTiltCurrPos;  //Stop Wrist Tilt in case it is moving
    LastHeartBeat = millis();               //Set LastHeartBeat so check isn't triggered right away
  }
  
  //Now, handle on-going servo position changes
  //Use delay to slow servo position change

  //***************************************************************************
  //                                NECK  
  //***************************************************************************
  if (NeckTargetPos > NeckCurrPos){         //Neck needs to move down
    NeckCurrPos ++;                         //Increase one step per loop         
    NeckServo.write(NeckCurrPos);
    AddServoChangeDelay = NeckServoChangeDelay;
  }else if (NeckTargetPos < NeckCurrPos) {  //Neck needs to move up
    NeckCurrPos --;                         //Decrease one step per loop
    NeckServo.write(NeckCurrPos);
    AddServoChangeDelay = NeckServoChangeDelay;
  }
  //***************************************************************************
  //                                   ARM  
  //***************************************************************************  
  if (ArmTargetPos > ArmCurrPos){           //Arm needs to move down
    //ArmTargetPos = ArmMaxPos;
    ArmCurrPos ++;                          //Increase one step per loop
    ArmServo.write(ArmCurrPos);
    AddServoChangeDelay = ArmServoChangeDelay;
    LoopsSinceArmDirectionChanged ++;
    if ((ArmCurrPos > (ArmMinPos+ArmWristAdjustTopBuffer))&&(ArmCurrPos < (ArmMaxPos-ArmWristAdjustBottomBuffer))){ //Arm is NOT above or below the ClearanceWatch position, adjust WristTilt position
     //ArmCurrPos > 75 && ArmCurrPos < 121
     //int ArmMinPos = 75;                       //Highest Arm Position - Black Robot
     //int ArmMaxPos = 131;                      //Lowest Arm Position  - Black Robot
     //int ArmWristAdjustTopBuffer = 0;
     //int ArmWristAdjustBottomBuffer = 10;
       if (LoopsSinceArmDirectionChanged > ArmWristUpLoopsBuffer){ 
        //LoopsSinceArmDirectionChanged = 2; //Whever Arm moves Up or Down
        //int ArmWristDownLoopsBuffer = 0;
        //int ArmWristUpLoopsBuffer = 0; 
        WristTiltTargetPos = WristTiltTargetPos-WristTiltArmAdjust;  //Adjust Wrist Tilt to keep it at relatively same angle
        WristTiltCurrPos = WristTiltTargetPos;
        WristTiltServo.write(WristTiltCurrPos);
      }
    }
    if (ArmCurrPos==ArmMaxPos){             //Arm is at bottom, Relay is ON, turn it OFF
      delay(2000);                          //Wait a little while before shutting off servo to allow arm to actually reach the bottom
      ArmRelay.aWrite2(255);                //Turn OFF Arm Relay
    }    
  }else if (ArmTargetPos < ArmCurrPos) {    //Arm needs to move up
      //ArmTargetPos = ArmMinPos;
      ArmRelay.aWrite2(0);                  //Turn ON Arm Relay -- It's OK to Set Arm Relay to On each time - Don't check for it being off
      ArmCurrPos --;                        //Decrease one step per loop
      ArmServo.write(ArmCurrPos);
      AddServoChangeDelay = ArmServoChangeDelay;
      LoopsSinceArmDirectionChanged ++; 
    if ((ArmCurrPos > (ArmMinPos+ArmWristAdjustTopBuffer))&&(ArmCurrPos < (ArmMaxPos-ArmWristAdjustBottomBuffer))){ //Arm is NOT above or below the ClearanceWatch position, adjust WristTilt position
     //ArmCurrPos > 75 && ArmCurrPos < 121
     //int ArmMinPos = 75;                       //Highest Arm Position - Black Robot
     //int ArmMaxPos = 131;                      //Lowest Arm Position  - Black Robot
     //int ArmWristAdjustTopBuffer = 0;
     //int ArmWristAdjustBottomBuffer = 10;
      if (LoopsSinceArmDirectionChanged > ArmWristDownLoopsBuffer){
        //LoopsSinceArmDirectionChanged = 2; //Whever Arm moves Up or Down
        //int ArmWristDownLoopsBuffer = 0;
        //int ArmWristUpLoopsBuffer = 0;        
        WristTiltTargetPos=WristTiltTargetPos+WristTiltArmAdjust; 
        WristTiltCurrPos = WristTiltTargetPos;
        WristTiltServo.write(WristTiltCurrPos);
      }    
    }  
  }
  //***************************************************************************
  //                               WRIST ROTATE
  //***************************************************************************  
  if (WristTargetPos > WristCurrPos){        //Wrist needs to move Clockwise
    WristCurrPos ++;                         //Increase one step per loop
    WristServo.write(WristCurrPos);
    AddServoChangeDelay = WristServoChangeDelay;
  }else if (WristTargetPos < WristCurrPos) { //Wrist needs to move Counter-Clockwise
    WristCurrPos --;                         //Decrease one step per loop
    WristServo.write(WristCurrPos);
    AddServoChangeDelay = WristServoChangeDelay;          
  }
  //***************************************************************************
  //                             WRIST TILT  
  //***************************************************************************  
    if (WristTiltTargetPos > WristTiltCurrPos){       //Wrist needs to move Up
    WristTiltCurrPos ++;                              //Increase one step per loop
    WristTiltServo.write(WristTiltCurrPos);
    if (AddServoChangeDelay < WristTiltServoChangeDelay){ //Only change delay if it is not set already (When Arm is moving, it is already set)
      AddServoChangeDelay = WristTiltServoChangeDelay;
    }
  }else if (WristTiltTargetPos < WristTiltCurrPos) { //Wrist needs to move Down
    WristTiltCurrPos --;                             //Decrease one step per loop
    WristTiltServo.write(WristTiltCurrPos);
    if (AddServoChangeDelay < WristTiltServoChangeDelay){ //Only change delay if it is not set already (When Arm is moving, it is already set)
      AddServoChangeDelay = WristTiltServoChangeDelay;
    }          
  }  
  //***************************************************************************
  //                              GRIPPER  
  //***************************************************************************  
  if (GripperTargetPos > GripperCurrPos){         //Gripper needs close
    GripperCurrPos ++;                            //Increase one step per loop
    GripperServo.write(GripperCurrPos);
    AddServoChangeDelay = GripperServoChangeDelay;
  }else if (GripperTargetPos < GripperCurrPos) { //Gripper needs to open
    GripperCurrPos --;                           //Decrease one step per loop
    GripperServo.write(GripperCurrPos);
    AddServoChangeDelay = GripperServoChangeDelay;          
  }  
  //***************************************************************************
  //                              DELAY & SERVO ENABLE
  //*************************************************************************** 
  if (AddServoChangeDelay > 0) {                     //Only delay if at least one servo was moved
    delay(AddServoChangeDelay);
    AddServoChangeDelay = 0;                         //Reset
  }
  
  if (ServoEnabled==0){
    if (millis()>2000){                              // Only if at least 2 seconds has elapsed since program started
      //ServosRelay.aWrite2(0);                        // Turn other Servos ON
      ServosRelay.aWrite2(255);                        // Turn other Servos OFF
      //ServoEnabled = 1;
      ServoEnabled = 0;
    }
  }
  
} //End Loop
