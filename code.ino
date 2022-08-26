#include <Arduino.h>
#include <AccelStepper.h>
#include <PIDController.h>
#include <Keypad.h>
#include <RotaryEncoder.h>

//// Pins ////
// Stepper Driver
const byte stepper1EnablePin = 4;
const byte stepper1StepPin = 2;
const byte stepper1DirectionPin = 3;
const byte endstop1Pin = 8;

// Ultrasonic Sensor
const byte ultrasonicTriggerPin = 10;
const byte ultrasonicEchoPin = 11;

// Rotary Knob
const byte rotaryKnobDTPin = 7;
const byte rotaryKnobCLKPin = 6;

// Keypad
const byte keypadRows = 4;
const byte keypadColumns = 4;
byte keypadRowPins[keypadRows] = {13, A4, A3, A2};
byte keypadColumnPins[keypadColumns] = {12, 9, A1, A5};

// Joystick
const byte joystickYPin = A0;
const byte joystickSWPin = 5;


//// Constants (User Configured) ////
volatile int stepper1Speed = 500; // microsteps/s
const int stepper1Acceleration = 2000; // microsteps/s^2
const int stepper1HomingSpeed = 100; // microsteps/s
const int stepper1HomingBumpDiviser = 2; // amount to divide "stepper1HomingSpeed" by for the second homing bump
const int stepper1HomingBumpDistance = 20; // number of full steps moved away from the endstop after the first homing bump
const int stepper1HomeOffset = 0; // number of full steps to move away from the endstop after the homing sequency completes
const int stepper1DriverMinimumPulseWidth = 1; // minimum pulse width required by the stepper driver on the step input
const int stepper1Microstepping = 8; // microstepping mode set on the stepper driver
const int stepper1HomedMaxLimit = 1150; // maximum full step limit after homing has completed
const int stepper1HomedMinLimit = 0; // minimum full step limit after homing has completed
const unsigned long stepperDisableTime = 30000; // time (ms) that the stepper motor remains powered while not in motion
const unsigned long endstopPressTimeForProgramRun = 2000; // time (ms) that the endstop must remain pressed to execute the program
const unsigned long ultrasonicDistanceUpdatePeriod = 25; // time (ms) between updates of the distance measured by the ultrasonic sensor
const unsigned long serialUpdatePeriod = 100; // time (ms) between updates of data sent using Serial.print()

// PID Controller
const float P = 20; // PID controller proportional gain
const float I = 0; // PID controller integral gain
const float D = 0; // PID controller derivative gain
const float updatePeriod = 1.0/976.5625; // time (s) between updates of the PID controller (PID interrupt occurs at 976.5625Hz)
const float outputMin = -1000; // minimum output from the PID controller
const float outputMax = 1000; // maximum output from the PID controller
const bool outputDirectionReverse = true; // reverses the direction of the PID controller output
const float integralWindupMin = -500; // minimum integral term windup (unused)
const float integralWindupMax = 500; // maximum integral term windup (unused)

// Keypad
char keys[keypadRows][keypadColumns] = { // keypad layout
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
const unsigned long passwordEnterTimeout = 4000; // timeout (ms) between key presses when entering password

// Joystick
const int joystickAnalogValueDeadRange = 5; // analog value range around joystick center where stage does not move
unsigned long joystickUpdatePeriod = 10; // time (ms) between updates of the stage speed from the joystick analog value


//// Variables ////
byte stepper1Enable = 0;
unsigned long lastStepperMove = 0; // timestamp of the last stepper movement (ms)
volatile float ultrasonicDistanceReading = 0; // distance measured by the ultrsonic sensor (mm)
unsigned long lastUltrasonicDistanceUpdate = 0; // timestamp of the last update of the ultrasonic sensor distance (ms)
unsigned long lastSerialUpdate = 0; // timestamp of the last update of the serial data (ms)
float ultrasonicDistanceReadingSetpoint = 150; // setpoint for the distance measured by the ultrasonic sensor (mm)
unsigned long passwordEnterStart = 0; // timestamp of the most recent keypad press (ms)
int operationMode = 0; // sets the operation mode of the stage (0: joystick control,  1: closed loop control)
float joystickAnalogValue = 0; // variable for the analog value read from the ADC for the joystick
int joystickAnalogValueCenter = 0; // analog value of the joystick center position
unsigned long lastJoystickUpdate = 0; // timestamp of the last update of the joystick data (ms)

//// Constructs ////
AccelStepper Stepper1(AccelStepper::DRIVER, stepper1StepPin, stepper1DirectionPin, false);
PIDController UltrasonicPID(P, I, D, updatePeriod, outputMin, outputMax, outputDirectionReverse, integralWindupMin, integralWindupMax);
RotaryEncoder RotaryKnob(rotaryKnobDTPin, rotaryKnobCLKPin, RotaryEncoder::LatchMode::FOUR0);
Keypad keypad =  Keypad(makeKeymap(keys), keypadRowPins, keypadColumnPins, keypadRows, keypadColumns);


//// Function Prototypes ////
void MoveSteppers(long int stepper1Position);
void HomeSteppers();
float GetUltrasonicDistanceReading();
void Stepper1InterruptEnable();
void Stepper1InterruptDisable();
bool PasswordChecker();


//// Setup ////
void setup() {
  pinMode(endstop1Pin, INPUT_PULLUP); // use internal pullup on the endstop pin, the endstop is wired normally closed between the endstop pin and GND

  // Stepper Driver Setup
  Stepper1.setPinsInverted(false, false, true);
  Stepper1.setEnablePin(stepper1EnablePin);
  Stepper1.disableOutputs();
  Stepper1.setMinPulseWidth(stepper1DriverMinimumPulseWidth);
  Stepper1.setMaxSpeed(stepper1Speed*stepper1Microstepping);
  Stepper1.setAcceleration(stepper1Acceleration*stepper1Microstepping);

  // Stepper Interrrupt Setup (this interrupt handles updating of the PID controller and stepper movment)
  OCR0A = 100; // interrupt will occur when timer 0 counter and OCR0A (compare register A for timer 0) are equal
  Stepper1InterruptDisable(); // disable the interrrupt for now

  // Ultrasonic Sensor
  pinMode(ultrasonicTriggerPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);
  ultrasonicDistanceReading = GetUltrasonicDistanceReading(); // get inital ultrasonic sensor distance measurement

  // Joystick
  pinMode(joystickSWPin, INPUT_PULLUP);
  joystickAnalogValueCenter = analogRead(joystickYPin); // get analog center value for the joystick

  Serial.begin(9600);
}


//// Main Loop ////
void loop() {
  if(PasswordChecker()) { // if password has been entered, run program
    delay(1000);
    // Main Program
    HomeSteppers(); // home the carriage
    ultrasonicDistanceReading = GetUltrasonicDistanceReading(); // update ultrasonic distance reading before enabling PID control
    Stepper1InterruptEnable(); // enable interrupt for PID control and stepper movement
    while(1) {
      if(operationMode == 1) { // closed loop control mode
        if(millis()-lastUltrasonicDistanceUpdate >= ultrasonicDistanceUpdatePeriod) {
          ultrasonicDistanceReading = 0.95*ultrasonicDistanceReading + 0.05*GetUltrasonicDistanceReading(); // ultrasonic sensor exponential low-pass filter
          long rotaryKnobPosition = RotaryKnob.getPosition(); // update position of the rotary encoder knob
          if(rotaryKnobPosition < -75) {
            rotaryKnobPosition = -75;
            RotaryKnob.setPosition(-75);
          }
          ultrasonicDistanceReadingSetpoint = 125 - rotaryKnobPosition; // calculate setpoint from rotary knob position (default setpoint is 125mm)
          lastUltrasonicDistanceUpdate = millis(); // timestamp
        }
        if(millis()-lastSerialUpdate >= serialUpdatePeriod) {
          //Serial.print("Ultrasonic Distance: ");
          Serial.print(ultrasonicDistanceReading);
          Serial.print(",");
          //Serial.println("mm");
          //Serial.print("Stepper Speed: ");
          //Serial.print(stepper1Speed);
          //Serial.println("steps/s");
          //Serial.println();
          Serial.println(ultrasonicDistanceReadingSetpoint);
          lastSerialUpdate = millis(); // timestamp
        }
      }
      else if(operationMode == 0) { // manual control mode
        if(millis()-lastJoystickUpdate >= joystickUpdatePeriod) {
          joystickAnalogValue = analogRead(joystickYPin); // get joystick ADC value
          if((joystickAnalogValue > joystickAnalogValueCenter+joystickAnalogValueDeadRange) ||
             (joystickAnalogValue < joystickAnalogValueCenter-joystickAnalogValueDeadRange)) {
            stepper1Speed = -(joystickAnalogValue-joystickAnalogValueCenter)*2; // calculate stepper speed based on joystick deflection from center
          }
          else stepper1Speed = 0;
          lastJoystickUpdate = millis(); // timestamp
        }
      }
      if(digitalRead(joystickSWPin) == 0) { // change modes if joystick is pressed
      operationMode++;
      operationMode %= 2;
      delay(50);
      }
    }
  }
  if(stepper1Enable==1 && millis()-lastStepperMove>=stepperDisableTime) { // if it has been longer than "stepperDisableTime" since the last stepper movement then
    Stepper1.disableOutputs();                                            // disable the stepper motor
  }
}


//// Motion ////
void MoveSteppers(long int stepper1Position) {
  // Maximum travel limit software endstop
  if(stepper1Position > stepper1HomedMaxLimit*stepper1Microstepping) { // limit the movement to the maximum travel limit
    Stepper1.moveTo(stepper1HomedMaxLimit*stepper1Microstepping);
  }
  // Mimimum travel limit software endstop
  else if(stepper1Position < stepper1HomedMinLimit*stepper1Microstepping) { // limit the movement to the minimum travel limit
    Stepper1.moveTo(stepper1HomedMinLimit*stepper1Microstepping);
  }
  else {
    Stepper1.moveTo(stepper1Position);
  }

  while(Stepper1.distanceToGo() != 0) { // move to the requested position with travel limits applied
    Stepper1.run();
  }
  lastStepperMove = millis(); // store when the most recent stepper movement occured
}


//// Homing ////
void HomeSteppers() {
  byte stepper1Homed = 0; // variable to keep track of homing progress

  // Stepper 1 Homing
  if(stepper1Enable==0) { // make sure the stepper is powered
    Stepper1.enableOutputs();
    stepper1Enable = 1;
  }
  Stepper1.setCurrentPosition(0); // set position to zero
  Stepper1.moveTo((-stepper1HomedMaxLimit-100)*stepper1Microstepping); // set the stepper to move the carriage towards the endstop
  Stepper1.setMaxSpeed(stepper1HomingSpeed*stepper1Microstepping); // set the stepper speed to the homing speed
  while(stepper1Homed==0) { // move the carriage towards the endstop until the endstop is triggered
    Stepper1.run();
    if(digitalRead(endstop1Pin)==1) stepper1Homed=1; // first homing bump is complete
  }
  Stepper1.setCurrentPosition(0); // set position to zero
  MoveSteppers(stepper1HomingBumpDistance*stepper1Microstepping); // bounce off the endstop
  Stepper1.moveTo((-stepper1HomingBumpDistance-25)*stepper1Microstepping); // set the stepper to move the carriage towards the endstop
  Stepper1.setMaxSpeed((stepper1HomingSpeed*stepper1Microstepping)/stepper1HomingBumpDiviser); // set stepper speed to the homing speed with bump divisor applied
  while(stepper1Homed==1) { // move the carriage towards the endstop until the endstop is triggered
    Stepper1.run();
    if(digitalRead(endstop1Pin)==1) stepper1Homed=2; // second homing bump is complete
  }
  Stepper1.setCurrentPosition(stepper1HomeOffset*stepper1Microstepping); // set the home position
  Stepper1.setMaxSpeed(stepper1Speed*stepper1Microstepping); // revert the stepper speed to "stepper1Speed"
}

//// Ultrasonic Sensor ////
float GetUltrasonicDistanceReading() {
  digitalWrite(ultrasonicTriggerPin, LOW);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTriggerPin, HIGH);
  delayMicroseconds(10);
  return pulseIn(ultrasonicEchoPin, HIGH)/5.8; // ultrasonic ulse width divided by 5.8 gives distance in mm
}


//// PasswordChecker ////
bool PasswordChecker () { // this is an unsecure way to check for a password, if any real security is needed another method should be used
  if(keypad.getKey() == '6') { // check for key press
    passwordEnterStart = millis(); // key press timestamp
    while(keypad.getKey() != '9') { // check for key press
      if(millis()-passwordEnterStart > passwordEnterTimeout) return 0; // timeout check
    }
    passwordEnterStart = millis(); // key press timestamp
    while(keypad.getKey() != '6') { // check for key press
      if(millis()-passwordEnterStart > passwordEnterTimeout) return 0; // timeout check
    }
    passwordEnterStart = millis(); // key press timestamp
    while(keypad.getKey() != '9') { // check for key press
      if(millis()-passwordEnterStart > passwordEnterTimeout) return 0; // timeout check
    }
    passwordEnterStart = millis(); // key press timestamp
    while(keypad.getKey() != '#') { // check for key press
      if(millis()-passwordEnterStart > passwordEnterTimeout) return 0; // timeout check
    }
    return 1; // password correct
  }
  else return 0; // password incorrect or button press timeout
}


//// Interrupts ////
void Stepper1InterruptEnable() {
  TIMSK0 |= _BV(OCIE0A); // enable interrupt by setting bit in timer 0 interrupt mask register
}

void Stepper1InterruptDisable() {
  TIMSK0 &= ~_BV(OCIE0A); // disable interrupt by clearing bit in timer 0 interrupt mask register
}

SIGNAL(TIMER0_COMPA_vect) { // interrupt
  RotaryKnob.tick(); // update rotary knob
  if(operationMode == 1) stepper1Speed = UltrasonicPID.calculate(ultrasonicDistanceReadingSetpoint-ultrasonicDistanceReading); // update PID controller
  if(stepper1Speed > 0) { // update stepper motion
    Stepper1.moveTo(stepper1HomedMaxLimit*stepper1Microstepping);
    Stepper1.setMaxSpeed(stepper1Speed);
    Stepper1.run();
  }
  else if(stepper1Speed < 0) { // update stepper motion
    Stepper1.moveTo(stepper1HomedMinLimit*stepper1Microstepping);
    Stepper1.setMaxSpeed(abs(stepper1Speed));
    Stepper1.run();
  }
}