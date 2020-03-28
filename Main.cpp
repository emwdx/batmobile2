// Servo - Version: 1.1.4
#include <Servo.h>

//Updated to use the Arduino Mega as the main processor for the Batmobile

//MARK TO-DO: 
//readRadioPWM(pin) - reading PWM value from the length of a pulse sent through the radio
//interrupt handler for left encoder
//interrupt handler for right encoder
//functions to calculate velocity from the right and left encoders


#define LINEAR_ACTUATOR_MID 566//1980
#define LINEAR_ACTUATOR_MAX_RIGHT 269//820
#define LINEAR_ACTUATOR_MAX_LEFT 784//3139
#define LINEAR_ACTUATOR_TICKS_PER_ONE_JOYSTICK 10
#define LINEAR_ACTUATOR_JOYSTICK_THRESHOLD 10
#define LINEAR_ACTUATOR_POT_THRESHOLD 30

#define BRAKE_POT_MIN 490
#define BRAKE_POT_MAX 875
#define GAS_POT_MIN 490
#define GAS_POT_MAX 875
#define STEERING_POT_MIN 364  
#define STEERING_POT_MAX 764
#define RADIO_STEER_MIN 1220
#define RADIO_STEER_MAX 1880

#define GAS_PRESS_THRESHOLD 500
#define BRAKE_PRESS_THRESHOLD 500

#define SHIFT_REVERSE_GEAR 0
#define SHIFT_LOW_GEAR 1
#define SHIFT_HIGH_GEAR 2
#define SHIFT_REVERSE_HIGH_GEAR 3

#define CONTROL_SCHEME_1 0
#define CONTROL_SCHEME_2 1
#define CONTROL_SCHEME_3 2
#define CONTROL_SCHEME_4 3

#define KID_CONTROL 0
#define PARENT_CONTROL 1

#define BOOST_OFF 0
#define BOOST_ON 1

#define DRIFT_OFF 0
#define DRIFT_ON 1

#define REVERSE_GEAR_MAX_SPEED -64
#define LOW_GEAR_MAX_SPEED 64
#define HIGH_GEAR_MAX_SPEED 127
#define REVERSE_HIGH_GEAR_MAX_SPEED -127

#define BOOST_REVERSE_GEAR_MAX_SPEED -64
#define BOOST_LOW_GEAR_MAX_SPEED 64
#define BOOST_HIGH_GEAR_MAX_SPEED 127
#define BOOST_REVERSE_HIGH_GEAR_MAX_SPEED -127

#define DRIFT_REAR_WHEEL_SPEED -15

#define THROTTLE_SCALE_FACTOR 25//25

#define CONTROL_MODE_LIGHT_PERIOD 13//

#define DEBUG_REPORT_PERIOD 1

#define REMOTE_ENABLE_PWM_THRESHOLD 127

#define RADIO_PWM_DEADBAND 15

bool revertToParentMode = false;

bool btn8LLastPressed = false;
bool carHeadlightInLastPressed = false;

bool btn8ULastPressed = false;



const int brakePedalPot = A0;
const int gasPedalPot = A1;
const int shiftStickPosition = A2; //Not being used at this time
const int steeringPot = A3;
const int linearActuatorPot = A4; //Not being used at this time

const int localSystemEnablePin = 4; //switch or jumper on-board the batmobile to control whether the system will respond to user inputs
const int headlightControlPin = 5; //SPIKE
//Not sure if we want to use these, but they are set to match legacy code
const int shiftGreenOut = 47;
const int shiftRedOut = 48;
const int shiftBlueOut = 49;
const int controlModeLightOut = 51; //SPIKE
const int auxiliaryLightingTwo = 52; //SPIKE
//End legacy pins

const int systemEnabledSignalLight = 13; //This is the LED on the Arduino. It will indicate whether or not the system is enabled

const int driftControlPin = 8; //Drift control input pin on the batmobile (not from the radio)

const int rightFrontMotorPin = 9; // PWM control for the right front motor
const int leftFrontMotorPin = 10; // PWM control for the left front motor
const int rearMotorPin = 11;  // PWM control for the rear motor
const int turnLinearActuatorPin = 12; //PWM for steer linear actuator

//The next group is the set of inputs being added for v2.

//Encoders
const int rightEncoderIndexPin = 18; //Arduino Interrupt 5
const int leftEncoderIndexPin = 19; //Arduino Interrupt 4
const int rearEncoderIndexPin = 3; //Arduino Interrupt 1
const int rightEncoderAnglePin = 17; //direction pin on right encoder
const int leftEncoderAnglePin = 16; //direction pin on left encoder
const int rearEncoderAnglePin = 15; //direction pin on rear encoder

//Note: leaving interrupts 20 and 21 open as these are the I2C pins on the Mega

//Radio inputs - these are for reading PWM values over the radio. These are digital inputs.
const int radioThrottlePin = 22; //Y from trigger throttle on controller
const int radioSteeringPin = 24; //X from wheel on controller
const int radioDriftModePin = 26; //remote drift button
const int radioSystemEnablePin = 28; //radio remote enable
const int radioLightingPin = 30;

//Set up PWM outputs for the motors (and legacy turn linear actuator)
Servo rightFrontMotor;// 9
Servo leftFrontMotor;// 10
Servo rearMotor;// 11
Servo turnLinearActuator;// 12


void setup(){
  
//Set up digital outputs
pinMode(localSystemEnablePin, INPUT);
pinMode(driftControlPin, INPUT);
pinMode(systemEnabledSignalLight,OUTPUT);
pinMode(radioLightingPin, INPUT);

pinMode(shiftGreenOut, OUTPUT);
pinMode(shiftRedOut, OUTPUT);
pinMode(shiftBlueOut, OUTPUT);
pinMode(controlModeLightOut, OUTPUT);
pinMode(auxiliaryLightingTwo, OUTPUT);


//Set up analog inputs
pinMode(brakePedalPot, INPUT_PULLUP);
pinMode(gasPedalPot, INPUT_PULLUP);
pinMode(shiftStickPosition, INPUT_PULLUP);
pinMode(steeringPot, INPUT_PULLUP);
pinMode(linearActuatorPot, INPUT_PULLUP);


//Set up encoders
pinMode(rightEncoderIndexPin, INPUT);
pinMode(leftEncoderIndexPin, INPUT);
pinMode(rearEncoderIndexPin, INPUT);
pinMode(rightEncoderAnglePin, INPUT);
pinMode(leftEncoderAnglePin, INPUT);
pinMode(rearEncoderAnglePin, INPUT);


//Set up radios
pinMode(radioThrottlePin, INPUT);
pinMode(radioSteeringPin, INPUT);
pinMode(radioDriftModePin, INPUT);
pinMode(radioSystemEnablePin, INPUT);

//Set up motors
rightFrontMotor.attach(rightFrontMotorPin);
leftFrontMotor.attach(leftFrontMotorPin);
rearMotor.attach(rearMotorPin);
turnLinearActuator.attach(turnLinearActuatorPin);

//Set up dashboard
Serial.begin(115200);
}

int readRadioInput(int inputPin){
  
  unsigned long inputPulseDuration =  pulseIn(inputPin, HIGH);
    //if there is a pulse being read of some kind that matches the duration we are looking for:
    if (inputPulseDuration > 875 && inputPulseDuration < 2085){
      int mappedValue = map(inputPulseDuration,1220,1880,-127,127);
    if (mappedValue>127){
      return 127;
    }
    else if(mappedValue < -127){
      return -127;
    }
    else if(mappedValue > 0 && mappedValue < RADIO_PWM_DEADBAND){
      return 0;
    }
    else if(mappedValue < 0 && mappedValue > -RADIO_PWM_DEADBAND){
      return 0;
    }
    else{
      return mappedValue;
    }
    }
    //otherwise return 0
    else{
      return 0;
    }
    
    
 
  
}

int clipMotorOutput(int output, int max) {
  if(output > 0 && output > max) {
    output = max;
  }
  else if(output < 0 && output < max) {
    output = max;
  }
  //else if(output == 0) {
  //  output = max;
  //}

  return output;
}

int rampDown(int currentSpeed) {
  if(currentSpeed > 0) {
    currentSpeed -= THROTTLE_SCALE_FACTOR;
  }
  else if(currentSpeed < 0) {
    currentSpeed += THROTTLE_SCALE_FACTOR;
  }

  return currentSpeed;
}

void driveLinearActuator(int steer) {
  int linearActuatorPotValue = analogRead(linearActuatorPot);
  int linearActuatorSetpointFromCenter = abs(steer) * LINEAR_ACTUATOR_TICKS_PER_ONE_JOYSTICK;
  int linearActuatorSetpoint;

  if(steer > LINEAR_ACTUATOR_JOYSTICK_THRESHOLD) {
    linearActuatorSetpoint = LINEAR_ACTUATOR_MID - linearActuatorSetpointFromCenter;
    if(linearActuatorSetpoint < LINEAR_ACTUATOR_MAX_RIGHT) {
      linearActuatorSetpoint = LINEAR_ACTUATOR_MAX_RIGHT;
    }
  }
  else if(steer < -LINEAR_ACTUATOR_JOYSTICK_THRESHOLD) {
    linearActuatorSetpoint = LINEAR_ACTUATOR_MID + linearActuatorSetpointFromCenter;
    if(linearActuatorSetpoint > LINEAR_ACTUATOR_MAX_LEFT) {
      linearActuatorSetpoint = LINEAR_ACTUATOR_MAX_LEFT;
    }
  }
  else {
    linearActuatorSetpoint = LINEAR_ACTUATOR_MID;
  }

  if(linearActuatorPotValue - linearActuatorSetpoint > LINEAR_ACTUATOR_POT_THRESHOLD) {
    motorControl(turnLinearActuator,127); //75
  }
  else if(linearActuatorPotValue - linearActuatorSetpoint < -LINEAR_ACTUATOR_POT_THRESHOLD) {
    motorControl(turnLinearActuator,-127); //-75
  }
  else {
    motorControl(turnLinearActuator,0);
  }
}



int getStickShiftPosition() {
  // Shifter
  int shiftStickValue = analogRead(shiftStickPosition);
  int retVal = SHIFT_LOW_GEAR;
//shifter high 975 -835
//shifter low 1050-975

  // LED values on the digital outs are reversed for the shifter, 0 turns a color on,
  // 1 turns it off.
  if(800 > shiftStickValue && shiftStickValue > 700) {
    // shifter in reverse
    digitalWrite(shiftRedOut,0);
    digitalWrite(shiftBlueOut,1);
    digitalWrite(shiftGreenOut,1);

    retVal = SHIFT_REVERSE_GEAR;
  }
  else if(975 > shiftStickValue && shiftStickValue > 835) {
    // shifter in high
    digitalWrite(shiftBlueOut,0);
    digitalWrite(shiftGreenOut,1);
    digitalWrite(shiftRedOut,1);

    retVal = SHIFT_HIGH_GEAR;
  }
  else if(1050 > shiftStickValue && shiftStickValue > 975) {
    // shifter in low
    digitalWrite(shiftGreenOut,0);
    digitalWrite(shiftBlueOut,1);
    digitalWrite(shiftRedOut,1);

    retVal = SHIFT_LOW_GEAR;
  }

  return retVal;
}

bool isKidDrivingEnabled(){

  int KidEnabled = pulseIn(radioSystemEnablePin, HIGH);
  if (KidEnabled > 1700) { 
    return true;
  }
 else return false;

}
 
  //return (readRadioInput (radioSystemEnablePin)==HIGH );


   //WP Changes Below
bool radioControlIsEnabled(){ 
    int RadioEnabled = pulseIn(radioSystemEnablePin, HIGH);
  if (1700 > RadioEnabled && RadioEnabled < 1400) { 
    return true;
  }
 else return false;
  

}


int getSteeringWheelPosition(int steeringPot) {
  int retVal;

  float steeringPotValue = analogRead(steeringPot);
  //float centeredValue = steeringPotValue - 0.5*(STEERING_POT_MAX + STEERING_POT_MIN);
  float centeredValue = steeringPotValue - 0.5*(STEERING_POT_MAX + STEERING_POT_MIN);
  float scaledSteeringPot = (STEERING_POT_MAX - STEERING_POT_MIN)/255;
  if(fabs(centeredValue) < 100) {
    retVal = 0;
  }
  else {
    retVal = centeredValue / scaledSteeringPot;
  }

  return round(retVal);
}

int getRadioSteeringPosition(int radioSteeringPin) {
  int retVal;

  float radioPotValue = pulseIn(radioSteeringPin, HIGH);
  //float centeredValue = steeringPotValue - 0.5*(STEERING_POT_MAX + STEERING_POT_MIN);
  float centeredValue = radioPotValue - 0.5*(RADIO_STEER_MAX + RADIO_STEER_MIN);
  float scaledRadioSteerPot = (RADIO_STEER_MAX - RADIO_STEER_MIN)/255;
  if(fabs(centeredValue) < 100) {
    retVal = 0;
  }
  else {
    retVal = centeredValue / scaledRadioSteerPot;
  }

  return round(retVal);
}


void driveMotorsRadio(int speed, int boostMode, int driftMode, int stickShiftPosition) {
  int threshold = 10;   // helps to eliminate 'noise' from a joystick that isn't perfectly at (0,0)
  // feel free to change this to match your needs.

  switch(boostMode) {
  case BOOST_OFF:
    switch(stickShiftPosition) {
    case SHIFT_LOW_GEAR:
      speed = clipMotorOutput(speed, LOW_GEAR_MAX_SPEED);
      break;
    case SHIFT_HIGH_GEAR:
      speed = clipMotorOutput(speed, HIGH_GEAR_MAX_SPEED);
      break;
    case SHIFT_REVERSE_GEAR:
      speed = clipMotorOutput(speed, REVERSE_GEAR_MAX_SPEED);
      break;
    case SHIFT_REVERSE_HIGH_GEAR:
      speed = clipMotorOutput(speed, REVERSE_HIGH_GEAR_MAX_SPEED);
      break;
    }
    break;
  case BOOST_ON:
    switch(stickShiftPosition) {
    case SHIFT_LOW_GEAR:
      speed = clipMotorOutput(speed, BOOST_LOW_GEAR_MAX_SPEED);
      break;
    case SHIFT_HIGH_GEAR:
      speed = clipMotorOutput(speed, BOOST_HIGH_GEAR_MAX_SPEED);
      break;
    case SHIFT_REVERSE_GEAR:
      speed = clipMotorOutput(speed, BOOST_REVERSE_GEAR_MAX_SPEED);
    case SHIFT_REVERSE_HIGH_GEAR:
      speed = clipMotorOutput(speed, BOOST_REVERSE_HIGH_GEAR_MAX_SPEED);
    }
    break;
  }
  
  int currentSpeed = map(leftFrontMotor.readMicroseconds(),1000,2000,-127,127); //all motors should be at the same speed unless we're drifting
  

  if(abs(speed) > threshold) {
    if(speed < 0) {
      
      speed = clipMotorOutput(speed, currentSpeed - THROTTLE_SCALE_FACTOR);
      
    }
    else if(speed > 0) {
      speed = clipMotorOutput(speed, currentSpeed + THROTTLE_SCALE_FACTOR);
      
      
    }
    
    motorControl(leftFrontMotor,speed);
    motorControl(rightFrontMotor,speed);
    
    int rearSpeed = speed;
    //This drift mode very much can be something more sophisticated, but this is it for now.
    //Right now the rear motor speed is arbitrarily set to be the same as the left motor speed. 
    switch(driftMode) {
    case DRIFT_OFF:
      break;
    case DRIFT_ON:
      rearSpeed = clipMotorOutput(rearSpeed, DRIFT_REAR_WHEEL_SPEED);
      break;
    }
    motorControl(rearMotor,rearSpeed);
    
  }
  else {
    speed = rampDown(currentSpeed);

    if(abs(speed) < threshold) {
      speed = 0;
    }

    motorControl(leftFrontMotor,speed);
    motorControl(rightFrontMotor,speed);
    motorControl(rearMotor,speed);
    
  }
}

void driveMotors(int speed, int boostMode, int driftMode, int stickShiftPosition) {
  int threshold = 5;   // helps to eliminate 'noise' from a joystick that isn't perfectly at (0,0)
  // feel free to change this to match your needs.

  switch(boostMode) {
  case BOOST_OFF:
    switch(stickShiftPosition) {
    case SHIFT_LOW_GEAR:
      speed = clipMotorOutput(speed, LOW_GEAR_MAX_SPEED);
      break;
    case SHIFT_HIGH_GEAR:
      speed = clipMotorOutput(speed, HIGH_GEAR_MAX_SPEED);
      break;
    case SHIFT_REVERSE_GEAR:
      speed = clipMotorOutput(speed, REVERSE_GEAR_MAX_SPEED);
      break;
    case SHIFT_REVERSE_HIGH_GEAR:
      speed = clipMotorOutput(speed, REVERSE_HIGH_GEAR_MAX_SPEED);
      break;
    }
    break;
  case BOOST_ON:
    switch(stickShiftPosition) {
    case SHIFT_LOW_GEAR:
      speed = clipMotorOutput(speed, BOOST_LOW_GEAR_MAX_SPEED);
      break;
    case SHIFT_HIGH_GEAR:
      speed = clipMotorOutput(speed, BOOST_HIGH_GEAR_MAX_SPEED);
      break;
    case SHIFT_REVERSE_GEAR:
      speed = clipMotorOutput(speed, BOOST_REVERSE_GEAR_MAX_SPEED);
    case SHIFT_REVERSE_HIGH_GEAR:
      speed = clipMotorOutput(speed, BOOST_REVERSE_HIGH_GEAR_MAX_SPEED);
    }
    break;
  }

  int currentSpeed = map(leftFrontMotor.readMicroseconds(),1000,2000,-127,127); //all motors should be at the same speed unless we're drifting
  

  if(abs(speed) > threshold) {
    if(speed < 0) {
      speed = clipMotorOutput(speed, currentSpeed - THROTTLE_SCALE_FACTOR);
    }
    else if(speed > 0) {
      speed = clipMotorOutput(speed, currentSpeed + THROTTLE_SCALE_FACTOR);
      
    }

    motorControl(leftFrontMotor,speed);
    motorControl(rightFrontMotor,speed);
    
  
    int rearSpeed = speed;
    //This drift mode very much can be something more sophisticated, but this is it for now.
    //Right now the rear motor speed is arbitrarily set to be the same as the left motor speed. 
    switch(driftMode) {
    case DRIFT_OFF:
      break;
    case DRIFT_ON:
      rearSpeed = clipMotorOutput(rearSpeed, DRIFT_REAR_WHEEL_SPEED);
      break;
    }
    motorControl(rearMotor,rearSpeed);
    
  }
  else {
    speed = rampDown(currentSpeed);

    if(abs(speed) < threshold) {
      speed = 0;
    }

    motorControl(leftFrontMotor,speed);
    motorControl(rightFrontMotor,speed);
    motorControl(rearMotor,speed);
    
  }
}


/*
void driveControlSchemeOne(int boostMode, int driftMode) {
  //steering Ch4
  //throttle 8R
  //brake 8D
  int steer = vexRT[Ch4];
  int speed = 0;
  if(vexRT[Btn8R] == 1) {
    speed = 127;
  }
  else if(vexRT[Btn8D] == 1) {
    speed = -127;
  }
  else {
    speed = 0;
  }

  driveMotors(speed, boostMode, driftMode, speed >= 0 ? SHIFT_HIGH_GEAR : SHIFT_REVERSE_HIGH_GEAR);
  driveLinearActuator(steer);
}
*/

/*
void driveControlSchemeTwo(int boostMode, int driftMode) {
  //steering Ch1
  //throttle Ch3
  int steer = vexRT[Ch1];
  int speed = vexRT[Ch3];

  driveMotors(speed, boostMode, driftMode, speed >= 0 ? SHIFT_HIGH_GEAR : SHIFT_REVERSE_HIGH_GEAR);
  driveLinearActuator(steer);
}
*/
/*
void driveControlSchemeThree(int boostMode, int driftMode) {
  //steering Ch4
  //throttle Ch3
  int steer = vexRT[Ch4];
  int speed = vexRT[Ch3];

  driveMotors(speed, boostMode, driftMode, speed >= 0 ? SHIFT_HIGH_GEAR : SHIFT_REVERSE_HIGH_GEAR);
  driveLinearActuator(steer);
}
*/

/*
void driveControlSchemeFour(int boostMode, int driftMode) {
  //steering Ch4
  //throttle Ch2
  int steer = vexRT[Ch4];
  int speed = vexRT[Ch2];

  driveMotors(speed, boostMode, driftMode, speed >= 0 ? SHIFT_HIGH_GEAR : SHIFT_REVERSE_HIGH_GEAR);
  driveLinearActuator(steer);
}
*/

void driveControlSchemeRadio(int stickShiftPosition, int boostMode, int driftMode){
  //float brakePedalValue = float(readRadioInput(radioThrottlePin));
  //float gasPedalValue = float(readRadioInput(radioThrottlePin));
  int steer = getRadioSteeringPosition(radioSteeringPin);
  int speed = 0;
  //float potScalingFactor = (BRAKE_POT_MAX - BRAKE_POT_MIN)/127;//3.03
  int throttle = readRadioInput(radioThrottlePin);
;/*
  if(brakePedalValue > BRAKE_PRESS_THRESHOLD) {
    // brake is pressed
    int desiredSpeed = round((gasPedalValue - BRAKE_POT_MIN) / potScalingFactor);
    if(stickShiftPosition == SHIFT_REVERSE_GEAR) {
      speed = -desiredSpeed + round((brakePedalValue - BRAKE_POT_MIN) / potScalingFactor);
      if(speed > 0) speed = 0;
      } else {
      speed = desiredSpeed - round((brakePedalValue - BRAKE_POT_MIN) / potScalingFactor);
      if(speed < 0) speed = 0;
    }
  }
  */
  if(abs(throttle) > RADIO_PWM_DEADBAND ) {
    // gas is pressed
    speed = throttle;
    
  }
  else {
    // neither is pressed
    speed = 0;
  }

  if(radioControlIsEnabled()){
    
    driveMotorsRadio(speed, boostMode, driftMode, stickShiftPosition);
    {
    //if(abs(steer) > RADIO_PWM_DEADBAND){
      driveLinearActuator(steer);
      digitalWrite(systemEnabledSignalLight,1);
      //motorControl(turnLinearActuator,steer);
    }
  //}    else{
    //motorControl(turnLinearActuator,0);
   // }
  }
  else{
    driveMotors(0, boostMode, driftMode, stickShiftPosition);
   driveLinearActuator(0);
    digitalWrite(systemEnabledSignalLight,0);
  }


}

void driveControlSchemeKid(int stickShiftPosition, int boostMode, int driftMode){
  float brakePedalValue = float(analogRead(brakePedalPot));
  float gasPedalValue = float(analogRead(gasPedalPot));
  int steer = getSteeringWheelPosition(steeringPot);
  int speed = 0;
  float potScalingFactor = (BRAKE_POT_MAX - BRAKE_POT_MIN)/127;//21.26;
  

  if(brakePedalValue > BRAKE_PRESS_THRESHOLD) {
    // brake is pressed
    int desiredSpeed = round((gasPedalValue - BRAKE_POT_MIN) / potScalingFactor);
    if(stickShiftPosition == SHIFT_REVERSE_GEAR) {
      speed = -desiredSpeed + round((brakePedalValue - BRAKE_POT_MIN) / potScalingFactor);
      if(speed > 0) speed = 0;
      } else {
      speed = desiredSpeed - round((brakePedalValue - BRAKE_POT_MIN) / potScalingFactor);
      if(speed < 0) speed = 0;
    }
  }
  
  else if(gasPedalValue > GAS_PRESS_THRESHOLD) {
    // gas is pressed
    speed = round((gasPedalValue - GAS_POT_MIN) / potScalingFactor);
    
    if(stickShiftPosition == SHIFT_REVERSE_GEAR) {
      speed = -speed;
    }
  }
  else {
    // neither is pressed
    speed = 0;
  }

  if(isKidDrivingEnabled()){
    
    driveMotors(speed, boostMode, driftMode, stickShiftPosition);
    //driveMotors(50, boostMode, driftMode, stickShiftPosition);
    driveLinearActuator(steer);
    digitalWrite(systemEnabledSignalLight,1);
  }
  else{
    driveMotors(0, boostMode, driftMode, stickShiftPosition);
    driveLinearActuator(0);
    digitalWrite(systemEnabledSignalLight,0);
  }

}


void move(int controlMode, int controlScheme
, int stickShiftPosition, int boostMode
, int driftMode) {
  switch(controlMode) {
   
  case PARENT_CONTROL:
    
    driveControlSchemeRadio(HIGH_GEAR_MAX_SPEED, BOOST_ON, driftMode);
    break;
    
  case KID_CONTROL:
    driveControlSchemeKid(stickShiftPosition, boostMode, driftMode);
    //driveControlSchemeDebug(stickShiftPosition, boostMode, driftMode);
    
    break;
  }
}

/*
int getConfiguredControlScheme(){
  return 3 - (SensorValue[controlSchemeBitOne] * 1 + SensorValue[controlSchemeBitTwo] * 2);
}
*/


int getControlMode (int currentControlMode)
{
  //remote control - 7L or 7D
  //onboard control - 7U
  //onboard control - 5U or 5D held
  int retVal;

  if(isKidDrivingEnabled()){
    retVal = KID_CONTROL;
  }
  else{
    retVal = PARENT_CONTROL;
  }

  // if(vexRT[Btn7L] == 1 || vexRT[Btn7D] == 1) {
  //  retVal = PARENT_CONTROL;
  //  revertToParentMode = false;
  // }
  // else if(vexRT[Btn7U] == 1) {
  //  retVal = KID_CONTROL;
  //  revertToParentMode = false;
  // }
  // else if(vexRT[Btn5U] == 1 || vexRT[Btn5D] == 1) {
  //  retVal = KID_CONTROL;
  //  //revert to PARENT_CONTROL regardless of what state we were in when we started
  //  //holding the momentary.
  //  revertToParentMode = true;
  // }
  // else if(revertToParentMode == true) {
  //  retVal = PARENT_CONTROL;
  //  revertToParentMode = false;
  // }
  // else {
  //  retVal = currentControlMode;
  // }

  return retVal;
}

int getBoostMode() {
  //boost - 6U
  int retVal;

  if(digitalRead(driftControlPin)==1){
//  if(vexRT[Btn6U] == 1) {
    retVal = BOOST_ON;
  }
  else {
    retVal = BOOST_OFF;
  }

  return retVal;
}

int getDriftMode() {
  //drift - 6D
  int retVal;
if (pulseIn(radioDriftModePin, HIGH) > 1700){
  //if(vexRT[Btn6D] == 1) {
    retVal = DRIFT_ON;
  }
  else {
    retVal = DRIFT_OFF;
  }

  return retVal;
}

void motorControl(Servo &selectedMotor,int speed){
  selectedMotor.writeMicroseconds(map(speed,-127,127,1000,2000));
}



void setControlModeLight(int controlMode, int loopCount) {
  switch(controlMode) {
  case KID_CONTROL:
    /*if((loopCount % CONTROL_MODE_LIGHT_PERIOD) == 0) {
      //SensorValue[controlModeLightOut] ^= 1;
      bool controlModeLightOutValue = !digitalRead(controlModeLightOut);
      digitalWrite(controlModeLightOut,controlModeLightOutValue);
    }*/
      digitalWrite(controlModeLightOut, HIGH); // sets the digital pin 13 on
      delay(500);            // waits for a second
      digitalWrite(controlModeLightOut, LOW);  // sets the digital pin 13 off
      delay(500);
    break;
  case PARENT_CONTROL:
    digitalWrite(controlModeLightOut,0);
    break;
  default:
    digitalWrite(controlModeLightOut,1);
    break;
  }
}

/*
void setAuxiliaryLightingOne(){
  if(pulseIn( || (SensorValue[carHeadlightIn] == 0 && !carHeadlightInLastPressed)) {
    SensorValue[auxiliaryLightingOne] ^= 1;
    btn8LLastPressed = true;
    carHeadlightInLastPressed = true;
  }

  if(vexRT[Btn8L] == 0) {
    btn8LLastPressed = false;
  }

  if(SensorValue[carHeadlightIn] == 1) {
    carHeadlightInLastPressed = false;
  }
}
*/
void setAuxiliaryLightingTwo(){
  if(pulseIn(radioLightingPin, HIGH) < 1600) {
    digitalWrite(auxiliaryLightingTwo, 0);
  }
  else {digitalWrite(auxiliaryLightingTwo, HIGH);
  }
}


//+++++++++++++++++++++++++++++++++++++++++++++| MAIN |+++++++++++++++++++++++++++++++++++++++++++++++


  //start with auxiliary lighting on
//  SensorValue[auxiliaryLightingOne] = 1;
//  SensorValue[auxiliaryLightingTwo] = 1;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  int controlScheme;
  int stickShiftPosition;
  int controlMode = PARENT_CONTROL;
  int boostMode;
  int driftMode;
  int loopCount = 0;


void dashboard(){
  
  String brakePedalPotString = "Brake Pedal Pot: " +  String(analogRead(brakePedalPot));
  String gasPedalPotString = "Gas Pedal Pot:" + String(analogRead(gasPedalPot));
  String steeringPotString = "Steering Pot:" + String(analogRead(steeringPot));
  String localSystemEnablePinString = "Kid Control Enabled (Pin 28) : " + String(pulseIn(radioSystemEnablePin, HIGH));
  String radioThrottleString = "Radio Throttle PWM: " + String(readRadioInput(radioThrottlePin));
  String radioSteeringString = "Radio Steering PWM: " + String(readRadioInput(radioSteeringPin));

  String rightFrontMotorString = "RightFrontMotor: " +  String(map(rightFrontMotor.readMicroseconds(),1000,2000,-127,127));
  String leftFrontMotorString = "LeftFrontMotor:" + String(map(leftFrontMotor.readMicroseconds(),1000,2000,-127,127));
  String rearMotorString = "RearMotor:" + String(map(rearMotor.readMicroseconds(),1000,2000,-127,127));
  String turnLinearActuatorPinString = "Turn Linear Actuator:" + String(map(turnLinearActuator.readMicroseconds(),1000,2000,-127,127));

  Serial.println("********************************");
  Serial.println("Inputs:");
  Serial.println(brakePedalPotString);
  Serial.println(gasPedalPotString);
  Serial.println(steeringPotString);
  Serial.println(localSystemEnablePinString);
  Serial.println(radioThrottleString);
  Serial.println(radioSteeringString);
  Serial.println("********************************");
  Serial.println("Outputs:");
  Serial.println(rightFrontMotorString);
  Serial.println(leftFrontMotorString);
  Serial.println(rearMotorString);
  Serial.println(turnLinearActuatorPinString);
  
  
  
}



int steerPotDebug = 500;
int brakePedalPotDebug = BRAKE_PRESS_THRESHOLD;
int gasPedalPotDebug = BRAKE_PRESS_THRESHOLD;
int testDirection = 1;


void loop() {
    controlMode = getControlMode(controlMode);

    setControlModeLight(controlMode, loopCount);
  //  setAuxiliaryLightingOne();
    setAuxiliaryLightingTwo();

    //controlScheme = getConfiguredControlScheme();
    stickShiftPosition = getStickShiftPosition();
    boostMode = getBoostMode();
    driftMode = getDriftMode();
    //move(controlMode, controlScheme, stickShiftPosition, boostMode, driftMode);
    
    move(controlMode, controlScheme, stickShiftPosition, BOOST_ON, driftMode);

    if((loopCount % ( DEBUG_REPORT_PERIOD * 10)) == 0) {
      loopCount = 0;
      dashboard();
    }
    loopCount++;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             

    //sleep for 30ms to approximate a periodic tick-through
    //wait1Msec(30);
    delay(10);
    
  }
  
