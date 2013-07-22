#include "BalancingRobot.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

#include "Kalman.h" // Kalman filter library see: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
Kalman kalman; // See https://github.com/TKJElectronics/KalmanFilter for source code

void setup() {
  Serial.begin(115200);

  /* Setup encoders */
  pinMode(leftEncoder1,INPUT);
  pinMode(leftEncoder2,INPUT);
  pinMode(rightEncoder1,INPUT);
  pinMode(rightEncoder2,INPUT); 
  attachInterrupt(0,leftEncoder,RISING); // pin 2
  attachInterrupt(1,rightEncoder,RISING); // pin 3

  /* Setup motor pins to output */
  sbi(leftPwmPortDirection,leftPWM);
  sbi(leftPortDirection,leftA);
  sbi(leftPortDirection,leftB);
  sbi(rightPwmPortDirection,rightPWM);
  sbi(rightPortDirection,rightA);
  sbi(rightPortDirection,rightB);  

  /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8025.pdf page 128-135 */
  // Set up PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1
  TCCR1A = 0;
  TCCR1B = _BV(WGM13) | _BV(CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1H = (PWMVALUE >> 8); // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz
  ICR1L = (PWMVALUE & 0xFF);

  /* Enable PWM on pin 9 (OC1A) & pin 10 (OC1B) */
  // Clear OC1A/OC1B on compare match when up-counting
  // Set OC1A/OC1B on compare match when downcountin
  TCCR1A = _BV(COM1A1) | _BV(COM1B1);
  setPWM(leftPWM,0); // Turn off pwm on both pins
  setPWM(rightPWM,0);

  stopAndReset();

  /* Setup pin for buzzer to beep when finished calibrating */
  pinMode(ledPin,OUTPUT);  

  /* Setup IMU Inputs */
  setupIMU();

  /* Calibrate the gyro and accelerometer relative to ground */
  calibrateSensors();

  /* Setup timing */
  loopStartTime = micros();
  timer = loopStartTime;
  encoderTimer = loopStartTime;
  dataTimer = millis();
}

void loop() {
  /* Calculate pitch */
  updateIMUsensor();
  accAngle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  gyroRate = (double)(gyroX-gyroX_offset)/131.0; // Convert to deg/s
  gyroAngle += gyroRate*((double)(micros()-timer)/1000000); // Gyro angle is only used for debugging
  if (gyroAngle < 0 || gyroAngle > 360)
    gyroAngle = pitch; // Reset the gyro angle when it has drifted too much

  // See my guide for more info about calculation the angles and the Kalman filter: http://arduino.cc/forum/index.php/topic,58048.0.htm
  pitch = kalman.getAngle(accAngle, gyroRate, (double)(micros() - timer)/1000000); // Calculate the angle using a Kalman filter
  timer = micros();  

  /* Drive motors */
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if((layingDown && (pitch < 170 || pitch > 190)) || (!layingDown && (pitch < 135 || pitch > 225))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  } 
  else {
    layingDown = false; // It's no longer laying down
    PID(targetAngle,targetOffset,turningOffset);       
  }

  /* Update wheel velocity every 100ms */
  if (micros() - encoderTimer >= 100000) { // Update encoder values every 100ms
    encoderTimer = micros();
    wheelPosition = readLeftEncoder() + readRightEncoder();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;
    if (abs(wheelVelocity) <= 20 && !stopped) { // Set new targetPosition if braking
      targetPosition = wheelPosition;
      stopped = true;
    }
  }

  /* Read the SPP connection */
  readSPP();    
  
  if (millis() - dataTimer > 50) {  // Only send data every 50ms
    if(sendPIDValues) {
      sendPIDValues = false;
      dataTimer = millis(); // Reset the timer, to prevent it from sending data in the next loop

      Serial.print("P,");
      Serial.print(Kp);
      Serial.print(',');
      Serial.print(Ki);
      Serial.print(',');
      Serial.print(Kd);
      Serial.print(',');
      Serial.println(targetAngle);
    } else if(sendData) {
      dataTimer = millis();
      
      Serial.print("V,");
      Serial.print(accAngle);
      Serial.print(',');
      Serial.print(gyroAngle);
      Serial.print(',');
      Serial.println(pitch);
    }
  }

  /* Use a time fixed loop */
  while((micros() - loopStartTime) < STD_LOOP_TIME);
  loopStartTime = micros();
}
void PID(double restAngle, double offset, double turning) {
  /* Steer robot */
  if (steerForward) {
    if (wheelVelocity < 0)
      offset += (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed - wheel velocity is negative when driving forward
    restAngle -= offset;
  }
  else if (steerBackward) {
    if (wheelVelocity > 0)
      offset -= (double)wheelVelocity/velocityScaleMove; // Scale down offset at high speed - wheel velocity is positive when driving backward
    restAngle += offset;
  }
  /* Brake */
  else if (steerStop) {
    long positionError = wheelPosition - targetPosition;
    if (abs(positionError) > zoneA) // Inside zone A
      restAngle -= (double)positionError/positionScaleA;
    else if (abs(positionError) > zoneB) // Inside zone B
      restAngle -= (double)positionError/positionScaleB;
    else // Inside zone C
      restAngle -= (double)positionError/positionScaleC;

    restAngle -= (double)wheelVelocity/velocityScaleStop;
    
    if (restAngle < targetAngle-10) // Limit rest Angle
      restAngle = targetAngle-10;
    else if (restAngle > targetAngle+10)
      restAngle = targetAngle+10;
  }
  /* Update PID values */
  double error = (restAngle - pitch);
  double pTerm = Kp * error;
  iTerm += Ki * error;
  double dTerm = Kd * (error - lastError);
  lastError = error;
  double PIDValue = pTerm + iTerm + dTerm;

  /* Steer robot sideways */
  double PIDLeft;
  double PIDRight;
  if (steerLeft) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue-turning;
    PIDRight = PIDValue+turning;
  }
  else if (steerRight) {
    turning -= abs((double)wheelVelocity/velocityScaleTurning); // Scale down at high speed
    if(turning < 0)
      turning = 0;
    PIDLeft = PIDValue+turning;
    PIDRight = PIDValue-turning;
  }
  else {
    PIDLeft = PIDValue;
    PIDRight = PIDValue;
  }

  //PIDLeft *= 0.95; // compensate for difference in the motors

  /* Set PWM Values */
  if (PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, PIDLeft * -1);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, PIDRight * -1);
}
void readSPP() {
  if(Serial.available()) {
    char input[30];
    uint8_t i = 0;
    while (1) {
      input[i] = Serial.read();
      if(input[i] == -1) // Error while reading the string
        return;
      if (input[i] == ';') // Keep reading until it reads a semicolon
        break;
      i++;
      if (i >= sizeof(input)/sizeof(input[0])) // String is too long
          return;
    }      

    if(input[0] == 'A') { // Abort
      stopAndReset();
      while(Serial.read() != 'C');
    } 
    
    else if (input[0] == 'G') { // The Processing/Android application sends when it needs the PID, settings or info
      if (input[1] == 'P') // Get PID Values
        sendPIDValues = true;
    }

    else if (input[0] == 'S') { // Set different values     
      /* Set PID and target angle */
      if (input[1] == 'P') {
        strtok(input, ","); // Ignore 'P'
        Kp = atof(strtok(NULL, ";"));
      } else if (input[1] == 'I') {
        strtok(input, ","); // Ignore 'I'
        Ki = atof(strtok(NULL, ";"));
      } else if (input[1] == 'D') {
        strtok(input, ","); // Ignore 'D'
        Kd = atof(strtok(NULL, ";"));
      } else if (input[1] == 'T') { // Target Angle
        strtok(input, ","); // Ignore 'T'
        targetAngle = atof(strtok(NULL, ";"));
      }
    }

    else if (input[0] == 'I') { // IMU transmitting states
      if (input[1] == 'B') // Begin sending IMU values
        sendData = true; // Send output to Processing/Android application
      else if (input[1] == 'S') // Stop sending IMU values
        sendData = false; // Stop sending output to Processing/Android application
    }
    
    else if (input[0] == 'C') { // Commands
      if (input[1] == 'S') // Stop
        steer(stop);
      else if (input[1] == 'J') { // Joystick
        strtok(input, ","); // Ignore 'J'
        sppData1 = atof(strtok(NULL, ",")); // x-axis
        sppData2 = atof(strtok(NULL, ";")); // y-axis
        steer(joystick);
      }
    }
  }
}
void steer(Command command) {
  // Set all false
  steerForward = false;
  steerBackward = false;
  steerStop = false;
  steerLeft = false;
  steerRight = false;
  if(command == joystick) {    
    if(sppData2 > 0) {
      targetOffset = scale(sppData2,0,1,0,7);        
      steerForward = true;
    } else if(sppData2 < 0) {
      targetOffset = scale(sppData2,0,-1,0,7);
      steerBackward = true;
    } 
    if(sppData1 > 0) {
      turningOffset = scale(sppData1,0,1,0,20);        
      steerRight = true;
    } else if(sppData1 < 0) {
      turningOffset = scale(sppData1,0,-1,0,20);
      steerLeft = true;     
    }
  }
  else if(command == stop) {
    steerStop = true;    
    if(lastCommand != stop) { // Set new stop position
      targetPosition = wheelPosition;
      stopped = false;
    }
  }
  lastCommand = command;
}
double scale(double input, double inputMin, double inputMax, double outputMin, double outputMax) { // Like map() just returns a double
  double output;
  if(inputMin < inputMax)
    output = (input-inputMin)/((inputMax-inputMin)/(outputMax-outputMin));              
  else
    output = (inputMin-input)/((inputMin-inputMax)/(outputMax-outputMin));
  if(output > outputMax)
    output = outputMax;
  else if(output < outputMin)
    output = outputMin;
  return output;
}
void stopAndReset() {
  stopMotor(left);
  stopMotor(right);  
  lastError = 0;
  iTerm = 0;
  targetPosition = wheelPosition;
}
void calibrateSensors() {
  int32_t accY_avg=0, accZ_avg=0, gyroX_avg=0;
  for (uint8_t i = 0; i < 100; i++) { // Take the average of 100 readings
    updateIMUsensor();
    gyroX_offset += (int)gyroX;
    accY_avg += (int)accY;
    accZ_avg += (int)accZ;
    delay(10);
  }
  gyroX_avg /= 100; // Gyro X-axis
  accY_avg /= 100; // Accelerometer Y-axis
  accZ_avg /= 100; // Accelerometer Z-axis

  gyroX_offset = gyroX_avg;
  accAngle = (atan2(accY_avg, accZ_avg) + PI) * RAD_TO_DEG;
  
  kalman.setAngle(accAngle);                                  // Set starting angle
  gyroAngle = accAngle;

  digitalWrite(ledPin,LOW);
  delay(500);  
  digitalWrite(ledPin,HIGH);
}
void moveMotor(Command motor, Command direction, double speedRaw) { // Speed is a value in percentage 0-100%
  if(speedRaw > 100)
    speedRaw = 100;
  int speed = speedRaw*((double)PWMVALUE)/100; // Scale from 100 to PWMVALUE
  if (motor == left) {
    setPWM(leftPWM,speed); // Left motor pwm
    if (direction == forward) {
      cbi(leftPort,leftA);
      sbi(leftPort,leftB);
    } 
    else if (direction == backward) {
      sbi(leftPort,leftA);
      cbi(leftPort,leftB);
    }
  } 
  else if (motor == right) {
    setPWM(rightPWM,speed); // Right motor pwm
    if (direction == forward) {
      sbi(rightPort,rightA);
      cbi(rightPort,rightB);
    } 
    else if (direction == backward) {
      cbi(rightPort,rightA);
      sbi(rightPort,rightB);
    }
  }
}
void stopMotor(Command motor) {  
  if (motor == left) {
    setPWM(leftPWM,PWMVALUE); // Set high
    sbi(leftPort,leftA);
    sbi(leftPort,leftB);
  } 
  else if (motor == right) {
    setPWM(rightPWM,PWMVALUE); // Set high
    sbi(rightPort,rightA);
    sbi(rightPort,rightB);
  }
}

void setPWM(uint8_t pin, int dutyCycle) { // dutyCycle is a value between 0-ICR
  if(pin == leftPWM) {
    OCR1AH = (dutyCycle >> 8); 
    OCR1AL = (dutyCycle & 0xFF);
  } else if (pin == rightPWM) {
    OCR1BH = (dutyCycle >> 8);
    OCR1BL = (dutyCycle & 0xFF);    
  }
}

/* Interrupt routine and encoder read functions - I read using the port registers for faster processing */
void leftEncoder() { 
  if(PIND & _BV(PIND4)) // read pin 4
    leftCounter--;
  else
    leftCounter++;    
}
void rightEncoder() {
  if(PIND & _BV(PIND5)) // read pin 5
    rightCounter++;
  else
    rightCounter--;  
}
long readLeftEncoder() { // The encoders decrease when motors are traveling forward and increase when traveling backward
  return leftCounter;
}
long readRightEncoder() {
  return rightCounter;
}

void setupIMU()
{
  // initialize device
  accelgyro.initialize();
  accelgyro.setRate(0);
  accelgyro.setDLPFMode(0);
  accelgyro.setTempSensorEnabled(false);

  // verify connection
  Serial.println(accelgyro.testConnection() ? "success" : "failed");

  /* Setup IMU */
  /*
  Wire.begin();
  i2cBuffer[0] = 19; // Set the sample rate to 400Hz - 8kHz/(19+1) = 400Hz
  i2cBuffer[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cBuffer[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cBuffer[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cBuffer, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x09, true)); // PLL with X axis gyroscope reference, disable temperature sensor and disable sleep mode
  
  while (i2cRead(0x75, i2cBuffer, 1));
  if (i2cBuffer[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1); // Halt
  }

  delay(100); // Wait for the sensor to get ready
  */
}

void updateIMUsensor()
{
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

  /* Update all the values, 讀取 MPU6050 的變量 */  
  /*
  while (i2cRead(0x3D, i2cBuffer, 8));
  accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  gyroX = ((i2cBuffer[6] << 8) | i2cBuffer[7]);
  */
}

/*
const char* doubleToString(double input, uint8_t digits) {
    char output[10];
    char buffer[10];
    if(input < 0) {
        strcpy(output,"-");
        input = -input;
    }
    else
        strcpy(output,"");
    
    // Round correctly
    double rounding = 0.5;
    for (uint8_t i=0; i<digits; i++)
        rounding /= 10.0;
    input += rounding;
    
    unsigned long intpart = (unsigned long)input;
    itoa(intpart,buffer,10); // Convert to string
    strcat(output,buffer);
    strcat(output,".");
    double fractpart = (input-(double)intpart);
    fractpart *= pow(10,digits);
    for(uint8_t i=1;i<digits;i++) { // Put zeroes in front of number
        if(fractpart < pow(10,digits-i)) {
            strcat(output,"0");
        }
    }
    itoa((unsigned long)fractpart,buffer,10); // Convert to string
    strcat(output,buffer);
    return output;
}
*/

/*
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress,&data,1,sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode;
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode;
  }
  Wire.requestFrom(IMUAddress, nbytes,(uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
*/
