/*Title: Festoon Machine
 *
 * Objective:
 *    To control the placement of fiber into the container
 *
 * Description:
 *    The container has one axis of movement (x).
 *    The pinch roller controls the speed in which the fiber is placed
 *    into the container
 *    The ramp is under the pinch roller and controls the fiber in the y 
 *    direction 
 *
 * Inputs:
 * 1. Start button
 * 2. Stop button
 * 3. Pinch Roller Speed Control Knob (Quadrature 16PPR Encoder)
 * Outputs:
 * 1. pinch roller motor Enable I/O:2
 * 2. fiber container motor (x) 
 * 3. ramp motor (y)
 * 4. status indicator light - when machine is running
 ________________________________________________________________________________
 
 Motor Setup for fiber container motor (x) AND  ramp motor (y)
 ________________________________________________________________________________
 * Title: 16PositionsHomeToHardStop
 * Objective:
 *    This example demonstrates control of the ClearPath-MCPV operational mode
 *    Move To Absolute Position, 16 Positions (Home to Hard Stop)
 *
 * Description:
 *    This example enables, homes, and then moves a ClearPath motor between
 *    preprogrammed absolute positions as defined in the MSP software. During
 *    operation, various move statuses are written to the USB serial port.
 *
 * Requirements:
 * 1. A ClearPath motor must be connected to Connector M-0.
 * 2. The connected ClearPath motor must be configured through the MSP software
 *    for Move To Absolute Position, 16 Positions (Home to Hard Stop) mode (In
 *    MSP select Mode>>Position>>Move to Absolute Position, then with "16
 *    Positions (Home to Hard Stop)" selected hit the OK button).
 * 3. The ClearPath motor must be set to use the HLFB mode "ASG-Position
 *    w/Measured Torque" with a PWM carrier frequency of 482 Hz through the MSP
 *    software (select Advanced>>High Level Feedback [Mode]... then choose
 *    "ASG-Position w/Measured Torque" from the dropdown, make sure that 482 Hz
 *    is selected in the "PWM Carrier Frequency" dropdown, and hit the OK
 *    button).
 * 4. The ClearPath must have defined Absolute Position Selections through
 *    the MSP software (On the main MSP window fill in the textboxes labeled
 *    1-16 found under "Position Selection Setup (cnts)").
 * 5. Homing must be configured in the MSP software for your mechanical
 *    system (e.g. homing direction, torque limit, etc.). To configure, click
 *    the "Setup..." button found under the "Homing" label on the MSP's main
 *    window.
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 * ** ClearPath Manual (DC Power): https://www.teknic.com/files/downloads/clearpath_user_manual.pdf
 * ** ClearPath Manual (AC Power): https://www.teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf
 *
 *
 * Copyright (c) 2020 Teknic Inc. This work is free to use, copy and distribute under the terms of
 * the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 * 
 _____________________________________________________________________________
 
 Motor Setup for Pinch Roller
 ________________________________________________________________________________
 * Title: ManualVelocity
 *
 * Objective:
 *    This example demonstrates control of the ClearPath-MC operational mode
 *    Manual Velocity Control.
 *
 * Description:
 *    This example enables a ClearPath motor and executes a repeating pattern of
 *    bidirectional velocity moves. During operation, various move statuses are
 *    written to the USB serial port.
 *
 * Requirements:
 * 1. A ClearPath motor must be connected to Connector M-0.
 * 2. The connected ClearPath motor must be configured through the MSP software
 *    for Manual Velocity Control mode (In MSP select Mode>>Velocity>>Manual
 *    Velocity Control, then hit the OK button).
 * 3. In the MSP software:
 *    * Define a Max Clockwise and Counter-Clockwise (CW/CCW) Velocity (On the
 *      main MSP window fill in the textboxes labeled "Max CW Velocity (RPM)"
 *      and "Max CCW Velocity (RPM)"). Any velocity commanded outside of this
 *      range will be rejected.
 *    * Set the Velocity Resolution to 2 (On the main MSP window check the
 *      textbox labeled "Velocity Resolution (RPM per knob count)" 2 is
 *      default). This means the commanded velocity will always be a multiple
 *      of 2. For finer resolution, lower this value and change
 *      velocityResolution in the sketch below to match.
 *    * Set Knob Direction to As-Wired, and check the Has Detents box (On the
 *      main MSP window check the dropdown labeled "Knob Direction" and the
 *      checkbox directly below it labeled "Has Detents").
 *    * On the main MSP window set the dropdown labeled "On Enable..." to be
 *      "Zero Velocity".
 *    * Set the HLFB mode to "ASG-Velocity w/Measured Torque" with a PWM carrier
 *      frequency of 482 Hz through the MSP software (select Advanced>>High
 *      Level Feedback [Mode]... then choose "ASG-Velocity w/Measured Torque" 
 *      from the dropdown, make sure that 482 Hz is selected in the "PWM Carrier
 *      Frequency" dropdown, and hit the OK button).
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 * ** ClearPath Manual (DC Power): https://www.teknic.com/files/downloads/clearpath_user_manual.pdf
 * ** ClearPath Manual (AC Power): https://www.teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf
 *
 * Copyright (c) 2020 Teknic Inc. This work is free to use, copy and distribute under the terms of
 * the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */
#include "ClearCore.h"

// Input pins
#define inputPin6 DI6 //Start Button
#define inputPin7 DI7 //Pause Button

//Motor 2 Encoder input pins
#define encoderPinA 4
#define encoderPinB 5

// The INPUT_A_B_FILTER must match the Input A, B filter setting in
// MSP (Advanced >> Input A, B Filtering...)
#define INPUT_A_B_FILTER 20

// Defines the motor's connector as ConnectorM0
#define motor1 ConnectorM0 //Left to Right movement of box
#define motor2 ConnectorM1 //Front to back movement of ramp

// Declares our user-defined helper function, which is used to send move
// commands. The definition/implementation of this function is at the bottom of
// the sketch
bool MoveToPositionM1(int positionNum);
void checkButtonStatus();
void checkEncoder();
void changeRampMotor2Speed();
bool StopButton = 0;
bool StartButton = 0;
bool motorsEnabled = 0;
bool containerPos = 0;
int caseStep = 1;
bool pathDirection = 0; //forward = 0, reverse = 1
int pos = 0;

// Define Variables for Encoder 
int counter = 0; 
int currentStateCLK;
int previousStateCLK;
String encdir ="";
 
void setup() {
// Put your setup code here, it will only run once:
    // Sets up serial communication and wait up to 5 seconds for a port to open
    // Serial communication is not required for this example to run
    Serial.begin(9600);
    uint32_t timeout = 5000;
    uint32_t startTime = millis();
    while (!Serial && millis() - startTime < timeout) {
        continue;
    }
    Serial.println("Setup Started");
    
    //Define pins   
    pinMode(IO1, OUTPUT); //Status Light
    pinMode(IO2, OUTPUT); //Motor 3 Enable

    // Set encoder pins as inputs  
    pinMode (encoderPinA,INPUT);
    pinMode (encoderPinB,INPUT);

    // Read the initial state of encoderPinA
    // Assign to previousStateCLK variable
    previousStateCLK = digitalRead(encoderPinA);
    
    // Sets all motor connectors to the correct mode for Absolute Position mode
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_A_DIRECT_B_DIRECT);

    // Set the motor's HLFB mode to bipolar PWM
    motor1.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motor2.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    
    // Set the HFLB carrier frequency to 482 Hz
    motor1.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motor2.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    // Enforces the state of the motor's A and B inputs before enabling the motor
    motor1.MotorInAState(false);
    motor1.MotorInBState(false);

    motor2.MotorInAState(false);
    motor2.MotorInBState(false);

    // Enable Motor 1 when start button is pushed; homing will then begin automatically
    while (digitalRead(inputPin6) != 1) {
      continue;
    }
    motor1.EnableRequest(true);
    digitalWrite(IO1, true); //turn on status light
    
    // Waits for HLFB to assert (waits for homing to complete if applicable)
    Serial.println("Waiting for HLFB...");
    while (motor1.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    //Enable Motor 2 and Motor 3
    motor2.EnableRequest(true);
    digitalWrite(IO2, true); //Enable pinch roller motor3
    motorsEnabled = 1;
    Serial.println("Motors Enabled and Ready");
}
void loop() {
  checkButtonStatus();
  checkEncoder();
  changeRampMotor2Speed();
  if (motorsEnabled == 1) {
      Serial.print("Case Step:");
      Serial.println(caseStep);
      switch (caseStep) {
      case 1:
        MoveToPositionM1(1);
        Serial.println("Pos 1");
        caseStep = 2;
        break;
      case 2:
        MoveToPositionM1(2); 
        Serial.println("Pos 2");
        caseStep = 1;
        break;
      }
   }
}   
/*------------------------------------------------------------------------------
 * MoveToPosition
 *
 *    Move to position number positionNum (defined in MSP)
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    position)
 *
 * Parameters:
 *    int positionNum  - The position number to command (defined in MSP)
 *
 * Returns: True/False depending on whether the position was successfully
 * commanded.
 */
bool MoveToPositionM1(int positionNum) {
  checkButtonStatus();
  if (motorsEnabled == 1) {
    // Check if an alert is currently preventing motion
    if (motor1.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor1 status: 'In Alert'. Move Canceled.");
        return false;
    }

    Serial.print("M1 Moving to position: ");
    Serial.println(positionNum);

    if (positionNum < 17 && positionNum > 0) {
        // Sends pulses on Input B based on positionNum
        for (int i = 0; i < positionNum; i++) {
            motor1.MotorInBState(true);
            delay(INPUT_A_B_FILTER);
            motor1.MotorInBState(false);
            delay(INPUT_A_B_FILTER);
        }

        // Triggers the command
        motor1.MotorInAState(true);
        delay(INPUT_A_B_FILTER);
        motor1.MotorInAState(false);
    }
    else {
        // If an invalid positionNum has been entered, returns a failure
        return false;
    }

    // Ensures this delay is at least 2ms longer than the Input A, B filter
    // setting in MSP
    delay(2 + INPUT_A_B_FILTER);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    Serial.println("M1 Moving.. Waiting for HLFB");
    while (motor1.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        checkEncoder();
        changeRampMotor2Speed();
        checkButtonStatus();
        continue;  
    }

    Serial.println("M1 Move Done");
    return true;
  }
}

//Function to check on button status to Enable/Disable motors
void checkButtonStatus(){
    // Read the state of the input connectors.
    StopButton = digitalRead(inputPin7);
    StartButton = digitalRead(inputPin6);

    // Disable Motors if stop button is pushed
    if (StopButton == 1) {
        motor1.EnableRequest(false);
        motor2.EnableRequest(false);
        digitalWrite(IO2, false); //Disable pinch roller motor3
        motorsEnabled = 0;
        Serial.println("Motors Disabled - Stop Button");
        digitalWrite(IO1, false); // turn light off
    }
    // Enable Motors if start button is pushed
    if (StopButton == 0 && StartButton) {
        motor1.EnableRequest(true);
        motor2.EnableRequest(true);
        digitalWrite(IO2, true); //Enable pinch roller motor3
        motorsEnabled = 1;
        Serial.println("Motors Enabled - Start Button");
        digitalWrite(IO1, true); //turn on light
    }
}
// Test program for Serial Readout of the digital rotary encoder
void checkEncoder(){
   // Read the current state of encoderPinA
   currentStateCLK = digitalRead(encoderPinA);
    
   // If the previous and the current state of the encoderPinA are different then a pulse has occured
   if (currentStateCLK != previousStateCLK){ 
    
     // If the encoderPinB state is different than the encoderPinA state then 
     // the encoder is rotating counterclockwise
     if (digitalRead(encoderPinB) != currentStateCLK) { 
       counter --;
       encdir ="CCW";
     } else {
       // Encoder is rotating clockwise
       counter ++;
       encdir ="CW";
     }
     Serial.print("Direction: ");
     Serial.print(encdir);
     Serial.print(" -- Value: ");
     Serial.println(counter);
   } 
   // Update previousStateCLK with the current state
   previousStateCLK = currentStateCLK; 
  
}

void changeRampMotor2Speed(){
  //Sends the digital rotary encoder inputs to Motor 2 A/B inputs
 
  if (digitalRead(encoderPinA) == 1){
    motor2.MotorInAState(true);
    delay(INPUT_A_B_FILTER);
  }
  else{
    motor2.MotorInAState(false);
    delay(INPUT_A_B_FILTER);
  }
  
  if (digitalRead(encoderPinB)==1) {
    motor2.MotorInBState(true);
    delay(INPUT_A_B_FILTER);
  }
  else{
    motor2.MotorInBState(false);
    delay(INPUT_A_B_FILTER);
  }
}
