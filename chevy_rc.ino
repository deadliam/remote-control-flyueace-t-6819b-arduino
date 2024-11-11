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

const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

Servo servoMotor1;
int posServo1 = 90;
int previousPosServo1 = 0;

int moveCarValue = 512;

int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

int deltaSteeringAngle = 9;
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
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);

  initialState();
}

void initialState() {
  stopMovement();
}
//==============================================

void loop() {
  
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
        // LEFT
        servoMotor1.detach();
        estimatedRCValue1 = 0;
        estimatedRCValue2 = 0;
        ignition = false;
        stopMovement();
        delay(50); break;
      case CENTER:
        // CENTER
        ignition = true;
        servoMotor1.attach(SERVOMOTOR_PIN);
        delay(50); break;
      case RIGHT: 
        // RIGHT
        ignition = true;
        servoMotor1.detach();
        delay(50); break;
      
      case INVALID: break; 
    }
  }

  // ------------------------------------------

  // output our values to the serial port in a format the plotter can use
  Serial.print("STEERING: ");
  Serial.print(estimatedRCValue1);  Serial.print(" : ");

  // ------------------------------------------
  posServo1 = map(estimatedRCValue1, 1525, 1610, 0, 180);
  if (posServo1 >= 180) {
    posServo1 = 180;
  }
  if (posServo1 <= 0) {
    posServo1 = 0;
  }
  Serial.print(posServo1);  Serial.print(" ----- ");

  moveCarValue = map(estimatedRCValue2, 1520, 1620, 0, 1023);
  if (moveCarValue > 1023) {
    moveCarValue = 1023;
  }
  if (moveCarValue <= 0) {
    moveCarValue = 0;
  }

  // ------------------------------------------
  // Steering
  servoMotor1.write(posServo1 + deltaSteeringAngle);

  // ------------------------------------------
  // Movement
  if (ignition == true) {
    if ((estimatedRCValue2 >= 1565) && (estimatedRCValue2 <= 1575)) {
      stopMovement();

    } else if (estimatedRCValue2 > 1575) {
      // reverse rotation
      moveBackward(); 

    } else if (estimatedRCValue2 <= 1565) {
      // forward rotation
      moveForward();
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