/*
 * This is the controller code for the reaction wheel satellite model published
 * at https://charleslabs.fr/en/project-Reaction+Wheel+Attitude+Control
 *
 * The electronics diagram and the mechanical build is described in this article.
 *
 * You can set the PID controller terms and various other parameters hereunder.
 *
 * Charles GRASSIN, 2021
 * MIT License
 */

#define SERIAL_DEBUG_ENABLE 1 /* 0 = disable, 1 = enable */
#define MPU6050_CALIBRATION 0 /* 0 = disable, 1 = enable */
#define CONTROLLER_MODE 0 /* 0 = Speed stabilization only, 1 = Speed and Attitude stabilization, 2 = same as 1, but change set point every N secondes */

// -------PINS-------
#define PIN_DIR    2
#define PIN_STEP   3
#define PIN_SLEEP  4
#define PIN_LED    13
// ------------------

// -----STEPPER------
#include "AccelStepper.h" // This is a hacked version of this library to allow
                          // speed control instead of position control.
AccelStepper myStepper(1, PIN_STEP, PIN_DIR);
double motorSpeed = 0;
#define MICROSTEPPING 4 /* 1 or 2 or 4 or 8 or 16 or 32 */
#define ACCELERATION 1750 /* Steps per s */
#define MAX_SPEED (600 * MICROSTEPPING)
// ------------------

// -------PID--------
#include "PID.h"
const double P_TERM = 0.050 * MICROSTEPPING;
const double I_TERM = 0.000 * MICROSTEPPING;
const double D_TERM = 0.017 * MICROSTEPPING; 
PIDController pidSpeed(P_TERM, I_TERM, D_TERM);
PIDAngleController pidAttitude(2.5, 0, 400);
// ------------------

// ------MPU6050-----
#include <Wire.h>
#define MPU_ADDRESS 0x68   // I2C address of the MPU-6050
double GYRO_ERROR = 61.96; // rollingAvg error compensation
double yawAngle=0, yawAngularSpeed=0;
// ------------------

// ------GENERAL-----
long timeCur, timePrev, timeStart; 
const int numReadings= 5;
double readings[numReadings];
int readIndex = 0;
double total = 0, rollingAvg = 0;
double targetAttitude = 0;
// ------------------

void setup() {
  #if SERIAL_DEBUG_ENABLE == 1
    Serial.begin(115200);
    delay(500);
    Serial.println("Attitude Speed");
  #endif
  
  // Gyro setup
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // wakes up the MPU-6050
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x10); // 1000dps full scale
  Wire.endTransmission(true);

  // Gyro cal (only if selected, otherwise use saved value)
  #if MPU6050_CALIBRATION == 1
    calibrateMPU();
  #endif

  // LED
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,1);
  
  // Initial stepper parameters
  myStepper.setEnablePin (PIN_SLEEP);
  myStepper.setAcceleration(ACCELERATION);
  setSpeedStepper(0);
  
  timeCur = millis();
  timeStart = timeCur;
}

// FSM variables
byte controllerState = 0;
int counts = 0;

// Main loop
void loop() {
  // Stop control after 40s
  // if(millis() - timeStart > 40000){
  //   digitalWrite(PIN_LED,0);
  //   myStepper.disableOutputs ();
  // }
  
  // Pulse stepper
  myStepper.run();

  // Every 10ms, read MPU and call controllers
  if(millis() - timeCur > 10) {
    timePrev = timeCur;
    timeCur = millis();

    // Measure Gyro value
    yawAngularSpeed = ((double)readMPU()-GYRO_ERROR) / 32.8;
    yawAngle += (yawAngularSpeed * (timeCur - timePrev) / 1000.0);
    // Put angle between -180 and 180
    while (yawAngle <= -180) yawAngle += 360; 
    while (yawAngle > 180)   yawAngle -= 360;

    // Low Pass Filter the angular speed (https://www.arduino.cc/en/Tutorial/BuiltInExamples/Smoothing)
    total = total - readings[readIndex]; 
    readings[readIndex] = yawAngularSpeed;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    if (readIndex >= numReadings)
      readIndex = 0;
    rollingAvg = total / numReadings; 

    // Compute controller output
    #if CONTROLLER_MODE == 0
      // Detumbling only
      motorSpeed += pidSpeed.compute(0,rollingAvg,timeCur - timePrev);
    #else if CONTROLLER_MODE == 1 || CONTROLLER_MODE == 2
      // Change set point
      #if CONTROLLER_MODE == 2
        counts ++;
        if(counts == 250) {
          counts = 0;
          if(targetAttitude == 0)
            targetAttitude = 180;
          else
            targetAttitude = 0;
        }
      #endif
      
      // FSM transition
      if(controllerState == 1 &&  fabs(rollingAvg) > 360 /* °/s */){
        controllerState = 0;digitalWrite(PIN_LED,0);
      }
      else if(controllerState == 0 && fabs(rollingAvg) < 45 /* °/s */)
        controllerState = 1;
      
      //FSM action
      if(controllerState == 0){
        motorSpeed += pidSpeed.compute(0,rollingAvg,timeCur - timePrev);
        pidAttitude.compute(targetAttitude,yawAngle,timeCur - timePrev);
      }
      else
        motorSpeed += pidSpeed.compute(pidAttitude.compute(targetAttitude,yawAngle,timeCur - timePrev),rollingAvg,timeCur - timePrev);
    #endif

    // Constrain speed to valid interval (saturation)
    if(motorSpeed > MAX_SPEED) motorSpeed = MAX_SPEED;
    else if (motorSpeed < -MAX_SPEED) motorSpeed = -MAX_SPEED;
    
    setSpeedStepper(motorSpeed);

    // Report attitude and speed
    #if SERIAL_DEBUG_ENABLE == 1
      Serial.print(yawAngle);
      Serial.print(" ");
      Serial.println(rollingAvg);
    #endif
  }
}

// Set the current speed and direction of the motor
void setSpeedStepper(double targetSpeed){
  if(targetSpeed > 0)
    myStepper.moveTo(1000000);
  else 
    myStepper.moveTo(-1000000);

  myStepper.setMaxSpeed(fabs(targetSpeed));
}

// Read a yaw angular speed value
int16_t readMPU(){
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS,2,true);
  return Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

// Calibrate the gyro by doing CALIBRATION_MEASUREMENTS_COUNT measurements
#define CALIBRATION_MEASUREMENTS_COUNT 200
void calibrateMPU(){
  GYRO_ERROR = 0;
  for(int i=0;i<CALIBRATION_MEASUREMENTS_COUNT;i++){
    GYRO_ERROR += readMPU();
    delay(20);
  }
  GYRO_ERROR = GYRO_ERROR/(double)CALIBRATION_MEASUREMENTS_COUNT;
  #if SERIAL_DEBUG_ENABLE == 1
    Serial.println(GYRO_ERROR);
  #endif
}
