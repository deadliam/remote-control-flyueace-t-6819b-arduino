/*
  Arduino RC Controller Input
  ---------------------------
  Lesson 1: Read RC Controller Input and outputs the values to the plotter

  Read more about this example on the blog post at https://www.andyvickers.net/

  Author: Andy Vickers
  Downloaded From: https://github.com/andyman198/ArduinoRCControllerInput
  
*/
#include <SimpleKalmanFilter.h>
#include <Servo.h>

SimpleKalmanFilter simpleKalmanFilter1(2, 2, 0.007);
SimpleKalmanFilter simpleKalmanFilter2(2, 2, 0.003);
SimpleKalmanFilter simpleKalmanFilter3(2, 2, 0.01);

// USED PINS 2,3,4,5,6,7,9,12,13
const int IGNITION_RELAY_PIN = 7;
// const int HIGH_LOW_SPEED_PIN = ;
const int FORWARD_GEARBOX_PIN = 13;
const int BACKWARD_GEARBOX_PIN = 12;
const int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
const int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

bool isOnboardControlEnabled = false;
int onBoardGoForwardSwitchState = 1;              // Inverted value. 0 - true, 1 - false
int onBoardGoBackwardSwitchState = 1;             // Inverted value. 0 - true, 1 - false
int onBoardHighLowSpeedSwitchState = 1;           // Inverted value. 0 - true, 1 - false

// Variables for movement with onboarding control
int motorSpeed = 511;         // Current motor speed (0-255)
const int speedStep = 10;    // Step increment for speed
const int speedStepStopping = 20;    // Step increment for speed
const unsigned long interval = 50; // Time interval for ramp-up (ms)
unsigned long lastUpdate = 0; // Tracks the last time motor speed was updated
int MAX_SPEED_FORWARD = 200; // 0 - maximum, 511 - stop
int MAX_SPEED_BACKWARD = 600; // 1023 - maximum, 511 - stop
int STOP_VALUE = 511;

const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

Servo servoMotor1;
int posServo1 = 90;
int previousPosServo1 = 0;

int moveCarValue = 512;

int deltaSteeringAngle = 0;
int deltaMovement = 0;

enum position_channel7 {INVALID, RIGHT, LEFT, CENTER};
position_channel7 switchPosition = INVALID;
position_channel7 previousSwitchPosition = INVALID; 

bool ignition = false;

// Set the port speed for host communication
#define SERIAL_PORT_SPEED 115200

// Set the size of the arrays (increase for more channels)
#define RC_NUM_CHANNELS 2

// Set up our receiver channels - these are the channels from the receiver
#define RC_CH1  0 
#define RC_CH2  1 

// Set up our channel pins - these are the pins that we connect to the receiver
#define RC_CH1_INPUT  2 // receiver pin 2
#define RC_CH2_INPUT  3 // receiver pin 3
#define RC_CH7_INPUT  4 // receiver pin 4
#define SERVOMOTOR_PIN 9

int RC_VALUE_CH7 = 0;

// Set up some arrays to store our pulse starts and widths
uint16_t RC_VALUES[RC_NUM_CHANNELS];
uint32_t RC_START[RC_NUM_CHANNELS];
volatile uint16_t RC_SHARED[RC_NUM_CHANNELS];

bool isMovingForward = false;
bool isMovingBackward = false;


// Setup our program
void setup() {
  
  // Set the speed to communicate with the host PC
  Serial.begin(SERIAL_PORT_SPEED);

  // Set our pin modes to input for the pins connected to the receiver
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH7_INPUT, INPUT);

  // Attach interrupts to our pins
  attachInterrupt(digitalPinToInterrupt(RC_CH1_INPUT), READ_RC1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH2_INPUT), READ_RC2, CHANGE);

  servoMotor1.attach(SERVOMOTOR_PIN);

  pinMode(RPWM_Output, INPUT_PULLUP);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, INPUT_PULLUP);
  pinMode(LPWM_Output, OUTPUT);
  pinMode(IGNITION_RELAY_PIN, INPUT_PULLUP);
  pinMode(IGNITION_RELAY_PIN, OUTPUT);

  // pinMode(HIGH_LOW_SPEED_PIN, INPUT_PULLUP);
  pinMode(FORWARD_GEARBOX_PIN, INPUT_PULLUP);
  pinMode(BACKWARD_GEARBOX_PIN, INPUT_PULLUP);

  initialState();
}

void initialState() {
  stopMovement();
}
//==============================================

void loop() {

  onBoardGoForwardSwitchState = digitalRead(FORWARD_GEARBOX_PIN);
  onBoardGoBackwardSwitchState = digitalRead(BACKWARD_GEARBOX_PIN);
  // onBoardHighLowSpeedSwitchState = digitalRead(HIGH_LOW_SPEED_PIN);
  
  // read the values from our RC Receiver
  rc_read_values();

  float estimatedRCValue1 = simpleKalmanFilter1.updateEstimate(RC_VALUES[RC_CH1]);
  float estimatedRCValue2 = simpleKalmanFilter2.updateEstimate(RC_VALUES[RC_CH2]);
  float rcValue7 = simpleKalmanFilter3.updateEstimate(pulseIn(RC_CH7_INPUT, HIGH));
  
  // ------------------------------------------
  // Read CHANNEL 7

  if (rcValue7 > 1790) {
    switchPosition = LEFT;

  } else if ((rcValue7 > 1490) && (rcValue7 < 1495)) {
    switchPosition = CENTER;
    
  } else if (rcValue7 < 1201) {
    switchPosition = RIGHT;
  }

  if (switchPosition != previousSwitchPosition)
  {
    previousSwitchPosition = switchPosition;
    switch (switchPosition)
    {
      case LEFT: 
        // LEFT: Disable movement
        servoMotor1.detach();
        estimatedRCValue1 = 0;
        estimatedRCValue2 = 0;
        ignition = false;
        stopMovement();
        ignition_car(0);
        delay(50); break;
      case CENTER:
        // CENTER: RC Controlled
        ignition = true;
        isOnboardControlEnabled = false;
        ignition_car(1);
        servoMotor1.attach(SERVOMOTOR_PIN);
        delay(50); break;
      case RIGHT: 
        // RIGHT: Onboard controlled
        ignition = true;
        isOnboardControlEnabled = true;
        ignition_car(1);
        servoMotor1.detach();
        delay(50); break;
      
      case INVALID: break; 
    }
  }

  // ------------------------------------------

  // output our values to the serial port in a format the plotter can use
  // Serial.print("STEERING: ");
  // Serial.print(estimatedRCValue1);  Serial.print(" : ");

  // ------------------------------------------
  posServo1 = map(estimatedRCValue1, 1525, 1610, 0, 180);
  if (posServo1 >= 180) {
    posServo1 = 180;
  }
  if (posServo1 <= 0) {
    posServo1 = 0;
  }
  // Serial.print(posServo1);  Serial.print(" ----- ");

  // ------------------------------------------
  // Steering
  servoMotor1.write(posServo1 + deltaSteeringAngle);

  // ------------------------------------------
  // Movement

  if (isOnboardControlEnabled == false) {
    Serial.print(" ");
    Serial.print("RC CONTROL");  Serial.print(" ");

    moveCarValue = map(estimatedRCValue2, 1520, 1620, 0, 1023);
    if (moveCarValue > 1023) {
      moveCarValue = 1023;
    }
    if (moveCarValue <= 0) {
      moveCarValue = 0;
    }

    if (ignition == true) {
      if ((estimatedRCValue2 >= 1565) && (estimatedRCValue2 <= 1575)) {
        stopMovement();

      } else if (estimatedRCValue2 > 1575) {
        // reverse rotation
        moveBackward(); 

      } else if ((estimatedRCValue2 <= 1565) && (moveCarValue != 0)) {
        // forward rotation
        moveForward();
      }
    }
  } else {
    // Onboard controls movement
    Serial.print(" ");
    Serial.print("ONBOARD");  Serial.print(" ");
    if (ignition == true) {
      if (onBoardGoForwardSwitchState == 0) {
        unsigned long currentTime = millis(); // Get the current time
        if (currentTime - lastUpdate >= interval) { // Check if interval has passed
          lastUpdate = currentTime; // Update the last update time
          if (motorSpeed > MAX_SPEED_FORWARD) {
            motorSpeed -= speedStep; // Increment motor speed
          } else {
            motorSpeed = MAX_SPEED_FORWARD;
          }
        }
        moveCarValue = motorSpeed;
        moveForward();
      } else if (onBoardGoForwardSwitchState == 1 && onBoardGoBackwardSwitchState == 1 && motorSpeed != STOP_VALUE) {
        // Slowly stop
        unsigned long currentTime = millis(); // Get the current time
        if (currentTime - lastUpdate >= interval) { // Check if interval has passed
          lastUpdate = currentTime; // Update the last update time
          if (motorSpeed <= STOP_VALUE) {
            motorSpeed += speedStepStopping; // Increment motor speed
          } else {
            motorSpeed = STOP_VALUE;
          }
        }
        if (motorSpeed > STOP_VALUE) {
          motorSpeed = STOP_VALUE;
        }
        moveCarValue = motorSpeed;
        moveForward();

      } else if (onBoardGoBackwardSwitchState == 0) {
        unsigned long currentTime = millis(); // Get the current time
        if (currentTime - lastUpdate >= interval) { // Check if interval has passed
          lastUpdate = currentTime; // Update the last update time
          if (motorSpeed < MAX_SPEED_BACKWARD) {
            motorSpeed += speedStep; // Increment motor speed
          } else {
            motorSpeed = MAX_SPEED_BACKWARD;
          }
        }
        moveCarValue = motorSpeed;
        moveBackward();
      } else {
        stopMovement();
        motorSpeed = STOP_VALUE;
      }
    }
  }

  // ------------------------------------------
  refresh_time = millis() + SERIAL_REFRESH_TIME;

  previousPosServo1 = posServo1;
  Serial.println("");
}

void stopMovement() {
  Serial.print("STOP");
  analogWrite(LPWM_Output, 0);
  analogWrite(RPWM_Output, 0);
  isMovingForward = false;
  isMovingBackward = false;
}

void moveForward() {
  Serial.print("FORWARD: ");
  int forwardPWM = -(moveCarValue - 511) / 2;
  analogWrite(LPWM_Output, 0);
  analogWrite(RPWM_Output, forwardPWM);
  isMovingForward = true;
  isMovingBackward = false;
  Serial.print(forwardPWM);
}

void moveBackward() {
  Serial.print("BACKWARD: ");
  int reversePWM = (moveCarValue - 512) / 2;
  analogWrite(LPWM_Output, reversePWM);
  analogWrite(RPWM_Output, 0);
  isMovingBackward = true;
  isMovingForward = false;
  Serial.print(reversePWM);
}

void ignition_car(int state) {
  if (state == 1) {
    digitalWrite(IGNITION_RELAY_PIN, HIGH);
  } else {
    digitalWrite(IGNITION_RELAY_PIN, LOW);
  }
}

// Thee functions are called by the interrupts. We send them all to the same place to measure the pulse width
void READ_RC1() { 
   Read_Input(RC_CH1, RC_CH1_INPUT); 
}
void READ_RC2() { 
   Read_Input(RC_CH2, RC_CH2_INPUT);
}

// This function reads the pulse starts and uses the time between rise and fall to set the value for pulse width
void Read_Input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    RC_START[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - RC_START[channel]);
    RC_SHARED[channel] = rc_compare;
  }
}

// this function pulls the current values from our pulse arrays for us to use. 
void rc_read_values() {
  noInterrupts();
  memcpy(RC_VALUES, (const void *)RC_SHARED, sizeof(RC_SHARED));
  interrupts();
}