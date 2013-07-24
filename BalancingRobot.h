#ifndef _balancingrobot_h_
#define _balancingrobot_h_

#include <stdint.h> // Needed for uint8_t

char stringBuf[30];

bool sendData;
bool sendPIDValues;

#define PWM_FREQUENCY 20000 // The motor driver can handle a pwm frequency up to 20kHz
#define PWMVALUE F_CPU/PWM_FREQUENCY/2 // Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no prescaling so frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

/* Used for the Communication and motor functions */
uint8_t lastCommand; // This is used set a new targetPosition
enum Command {
  stop,
  forward,
  backward,
  left,
  right,
  joystick,
};

/* These are used to read and write to the port registers - see http://www.arduino.cc/en/Reference/PortManipulation 
 I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/ */
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/* Left motor */
#define rightPort PORTB
#define rightPortDirection DDRB
#define rightA PINB3
#define rightB PINB4

#define rightPwmPortDirection DDRB
#define rightPWM PINB2

/* Right motor */
#define leftPort PORTD
#define leftPortDirection DDRD
#define leftA PIND7
#define leftB PIND6

#define leftPwmPortDirection DDRB
#define leftPWM PINB1

/* Encoders */
#define rightEncoder1 2
#define rightEncoder2 4
#define leftEncoder1 3
#define leftEncoder2 5

volatile int32_t leftCounter = 0;
volatile int32_t rightCounter = 0;

/* IMU */
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

uint8_t i2cBuffer[14]; // Buffer for I2C data

#define ledPin 13

// Results
double accAngle, gyroRate, gyroAngle;
double pitch;

/* PID variables */
double Kp = 20.0;
double Ki = 0.0;
double Kd = 2.0;
double targetAngle = 182.5;

double lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

double lastError; // Store last angle error
double integratedError; // Store integrated error

double error;
double pTerm, iTerm, dTerm;
double PIDValue, PIDLeft, PIDRight;

/* Used for timing */
#define STD_LOOP_TIME 7000 // Fixed time loop of 10 milliseconds
uint32_t loopStartTime;

uint32_t kalmanTimer; // Timer used for the Kalman filter
uint32_t pidTimer; // Timer used for the PID loop
uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
uint32_t dataTimer; // This is used so it doesn't send data to often

bool steerForward;
bool steerBackward;
bool steerStop = true; // Stop by default
bool steerLeft;
bool steerRight;

bool stopped; // This is used to set new target position after breaking

bool layingDown = true; // Use to indicate if the robot is laying down

double targetOffset = 0; // Offset for going forward and backward
double turningOffset = 0; // Offset for turning left and right

double sppData1 = 0;
double sppData2 = 0;

int32_t lastWheelPosition;
int32_t wheelVelocity;
int32_t targetPosition;

const uint16_t zoneA = 8000;
const uint16_t zoneB = 4000;
const uint16_t zoneC = 1000;
const double positionScaleA = 600; // One resolution is 928 pulses per encoder
const double positionScaleB = 800;
const double positionScaleC = 1000;
double positionScaleD = 5000;
const double velocityScaleMove = 70;
double velocityScaleStop = 1000;
const double velocityScaleTurning = 70;

#endif
