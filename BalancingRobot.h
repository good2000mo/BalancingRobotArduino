#ifndef _balancingrobot_h_
#define _balancingrobot_h_

#include <stdint.h> // Needed for uint8_t

char stringBuf[30];

bool sendData;
bool sendPIDValues;

#define PWM_FREQUENCY 20000 // The motor driver can handle a pwm frequency up to 20kHz
#define PWMVALUE F_CPU/PWM_FREQUENCY/2 // Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no prescaling so frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

/* Used for the Communication and motor functions */
int lastCommand; // This is used set a new targetPosition
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
#define leftPort PORTC
#define leftPortDirection DDRC
#define leftA PINC0
#define leftB PINC1

#define leftPwmPortDirection DDRB
#define leftPWM PINB2

/* Right motor */
#define rightPort PORTC
#define rightPortDirection DDRC
#define rightPwmPortDirection DDRB

#define rightA PINC2
#define rightB PINC3
#define rightPWM PINB1

/* Encoders */
#define leftEncoder1 2
#define leftEncoder2 4
#define rightEncoder1 3
#define rightEncoder2 5

volatile long leftCounter = 0;
volatile long rightCounter = 0;

/* IMU */
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
//int16_t accY;
//int16_t accZ;
//int16_t gyroX;

uint8_t i2cBuffer[14]; // Buffer for I2C data

#define ledPin 13

// Zero voltage values for the sensors - gyroX
double gyroX_offset = 0;

// Results
double accAngle;
double gyroRate;
double gyroAngle;
double pitch;

/* PID variables */
double Kp = 7;
double Ki = 0;
double Kd = 8;
double targetAngle = 180;

double lastError; // Store position error
double iTerm; // Store integral term

/* Used for timing */
unsigned long timer;

#define STD_LOOP_TIME 10000 // Fixed time loop of 10 milliseconds
unsigned long loopStartTime;

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

uint8_t loopCounter = 0; // Used to update wheel velocity
long wheelPosition;
long lastWheelPosition;
long wheelVelocity;
long targetPosition;
int zoneA = 8000;
int zoneB = 4000;
double positionScaleA = 500; // One resolution is 464 pulses
double positionScaleB = 1000; 
double positionScaleC = 2000;
double velocityScaleMove = 70;
double velocityScaleStop = 60;
double velocityScaleTurning = 70;
#endif
