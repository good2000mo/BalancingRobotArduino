#include "BalancingRobot.h"

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 accelgyro;

#include "Kalman.h"
Kalman kalman;

void setup() {
  Serial.begin(115200);

  /* Setup encoders */
  pinMode(leftEncoder1,INPUT);
  pinMode(leftEncoder2,INPUT);
  pinMode(rightEncoder1,INPUT);
  pinMode(rightEncoder2,INPUT); 
  attachInterrupt(0,leftEncoder,RISING);
  attachInterrupt(1,rightEncoder,RISING);

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

  //
  updateIMUsensor();
  accAngle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  
  kalman.setAngle(accAngle);                                  // Set starting angle
  gyroAngle = accAngle;

  digitalWrite(ledPin,LOW);
  delay(500);  
  digitalWrite(ledPin,HIGH);

  /* Setup timing */
  loopStartTime = micros();
  kalmanTimer = loopStartTime;
  pidTimer = loopStartTime;
  encoderTimer = loopStartTime;
  dataTimer = millis();
}

void loop() {
  /* Calculate pitch */
  updateIMUsensor();
  accAngle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  gyroRate = (double)(gyroX-gyroX_offset)/131.0; // Convert to deg/s
  gyroAngle += gyroRate*((double)(micros()-kalmanTimer)/1000000.0); // Gyro angle is only used for debugging
  if (gyroAngle < 0 || gyroAngle > 360)
    gyroAngle = pitch; // Reset the gyro angle when it has drifted too much

  // See my guide for more info about calculation the angles and the Kalman filter: http://arduino.cc/forum/index.php/topic,58048.0.htm
  pitch = kalman.getAngle(accAngle, gyroRate, (double)(micros() - kalmanTimer)/1000000.0); // Calculate the angle using a Kalman filter
  kalmanTimer = micros();  

  /* Drive motors */
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if((layingDown && (pitch < targetAngle-10 || pitch > targetAngle+10)) || (!layingDown && (pitch < targetAngle-45 || pitch > targetAngle+45))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  } 
  else {
    layingDown = false; // It's no longer laying down
    PID(targetAngle,targetOffset,turningOffset,(double)(micros()-pidTimer)/1000000.0);       
  }
  pidTimer = micros();

  /* Update wheel velocity every 100ms */
  if (micros() - encoderTimer >= 100000) { // Update encoder values every 100ms
    encoderTimer = micros();
    int32_t wheelPosition = getWheelPosition();
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
  //Serial.println((micros() - loopStartTime));
  loopStartTime = micros();
}
void PID(double restAngle, double offset, double turning, double dt) {
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
    int32_t wheelPosition = getWheelPosition();
    int32_t positionError = wheelPosition - targetPosition;

    if (abs(positionError) < zoneC)
      restAngle -= (double)positionError/positionScaleD;
    else
      targetPosition = wheelPosition;

    restAngle -= (double)wheelVelocity/velocityScaleStop;

    if (restAngle < targetAngle-10) // Limit rest Angle
      restAngle = targetAngle-10;
    else if (restAngle > targetAngle+10)
      restAngle = targetAngle+10;
  }

  if (restAngle - lastRestAngle > 1) // Don't change restAngle with more than 1 degree in each loop
    restAngle = lastRestAngle+1;
  else if (restAngle - lastRestAngle < -1)
    restAngle = lastRestAngle-1;
  lastRestAngle = restAngle;

  /* Update PID values */
  error = (restAngle - pitch);
  pTerm = Kp * error;
  integratedError += error*dt;
  integratedError = constrain(integratedError, -1.0, 1.0); // Limit the integrated error
  iTerm = (Ki*100.0) * integratedError;
  dTerm = (Kd/100.0) * (error - lastError)/dt;
  lastError = error;
  PIDValue = pTerm + iTerm + dTerm;
  
  /* Steer robot sideways */
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
    moveMotor(left, backward, -PIDLeft);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, -PIDRight);
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
      targetPosition = getWheelPosition();
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
  integratedError = 0;
  targetPosition = getWheelPosition();
  lastRestAngle = targetAngle;
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
int32_t getWheelPosition() {
  return leftCounter + rightCounter;
}

void setupIMU()
{
  // initialize device
  accelgyro.initialize();
  accelgyro.setRate(19);
  accelgyro.setDLPFMode(0);
  accelgyro.setTempSensorEnabled(false);

  // verify connection
  Serial.println(accelgyro.testConnection() ? "success" : "failed");
}

void updateIMUsensor()
{
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
}
