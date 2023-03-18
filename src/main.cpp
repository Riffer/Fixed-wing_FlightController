/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 8/24/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//  2019-07-08 - Added Auto Calibration routine
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
//#include <MS5611.h>
#include <Simple_Wire.h>
#include <Simple_MPU6050.h>
#include <jm_CPPM.h>

#include "main.h"

#define MPU6050_ADDR 0x68 // Address of MPU-6050
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_ACCEL_XOUT_H 0x3B
#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno
#define CALIBRATE_DPM_OFFSET 6
#define MILLI_SEC 1000
#define FS_SEL_0_GYRO 131.0

// Set Offset, scaled for min sensitivity
#define GYRO_OFFSET_X 120
#define GYRO_OFFSET_Y -22
#define GYRO_OFFSET_Z -4
#define ACC_OFFSET_X -2927
#define ACC_OFFSET_Y -339
#define ACC_OFFSET_Z 1836

#define KP 1.0
#define KI 0.2
#define KD 1.0
#define CENTER_OF_SERVO 90

#define DEADZONE 50
#define AILERON_CHANNEL_OFFSET PWM_MID
#define ELEVATOR_CHANNEL_OFFSET PWM_MID
#define YAW_CHANNEL_OFFSET PWM_MID

#define FLIGHT_MODE_0 PWM_MAX
#define FLIGHT_MODE_1 PWM_MID
#define FLIGHT_MODE_2 PWM_MIN

const float DEGREE_PER_PI = 180 / M_PI;


#include <ServoTimer2.h>
//#define ServoWrite(servo, degree) servo.write(mapT((double)degree, (double)0, (double)CENTER_OF_SERVO * 2, (double)PWM_MIN, (double)PWM_MAX))

#define ServoWrite(servo, degree) servo.write(mapf(degree, 0, (CENTER_OF_SERVO * 2), PWM_MIN, PWM_MAX))
#define ServoWriteMicroseconds(servo, ms) servo.write(ms)

Simple_MPU6050 mpu;

ServoTimer2 servoRudder, servoAileron, servoElevator, servoAileron2;

RPY Sensor; // Data of Axis from MPU6050
RPY Channel; // Data of Axis from Receiver
dRPY PIDFiltered; // Data of Axis from PID Function


// MS5611 ms5611;
// long realPressure;                                            // Pressure value
// double referencePressure;                                     // Reference P
// float absoluteAltitude, relativeAltitude;                     // Altitude


VectorInt16 gy;            // Raw data of Gyro Z



Pid rollPID, pitchPID, yawPID;

void initMPU6050();
bool verifyConnection();
void setGyroAccOffset();
void calibrateDMP();
void initServo();
void initPID();
void getChannelInput();
void getDmpYPR(int16_t *gyro, int16_t *accel, int32_t *quat);
int getSystemSignal();
void manualFlightControl();
void setAutoYPR();
void relativeLeveling();
void setAutoPID();
void pidLeveling();
void debugPrint();
void readGyroData();

void setup()
{
  Serial.begin(115200); // Initialize serial communication
  while (!Serial) delay(30); // Wait for Leonardo enumeration, others continue immediately
  
  CPPM.begin(); // setup CPPM - will be called in loop

   Serial.println(F("Initializing DMP..."));
  initMPU6050();      // Initialize MPU6050 for I2C
  setGyroAccOffset(); // Supply your own gyro offsets here, scaled for min sensitivity

  //  ms5611.begin();
  //  referencePressure = ms5611.readPressure();
  //  Serial.println(ms5611.getOversampling());

  initServo();
  initPID();
}

void loop()
{
  getChannelInput(); // get channel input from CPPM (se jm_CPPM.h "CPPM_ICP1" for PIN)

  // getAltitude();

  mpu.dmp_read_fifo(false); // can be replaced by interrupt on pin "interruptPin" from MPU (but maybe overflowing CPU)

  switch (getSystemSignal()) // use flight mode
  {
  case 0: // Manual Mode
    manualFlightControl();
    break;

  default: // case 1:   //Relative Control Mode
    setAutoYPR();
    relativeLeveling();
    break;

  case 2: // PID Control Mode
    yawPID.setpoint = 0;
    pitchPID.setpoint = 0;
    rollPID.setpoint = 0;
    setAutoPID();
    pidLeveling();
    break;
  }
  debugPrint();
}

/*==================================================================
                                Setup
===================================================================*/

void initMPU6050()
{
  Serial.println(F("Initializing mpu..."));
  mpu.begin();
  mpu.CalibrateMPU(); // Calibrates the MPU.
  mpu.load_DMP_Image();
  mpu.on_FIFO(getDmpYPR);
}

bool verifyConnection()
{
  bool test = mpu.TestConnection();
  // Verify connection
  Serial.println(F("Testing device connections...\nMPU6050 connection "));
  Serial.println(test ? F("successful") : F("failed"));
  return test;
}

void setGyroAccOffset()
{
  // Supply your own gyro offsets here, scaled for min sensitivity
  mpu.setOffset(ACC_OFFSET_X, ACC_OFFSET_Y, ACC_OFFSET_Z, GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z);
}

void calibrateDMP()
{
  // Calibration Time: generate offsets and calibrate MPU6050
  mpu.CalibrateAccel(CALIBRATE_DPM_OFFSET);
  mpu.CalibrateGyro(CALIBRATE_DPM_OFFSET);
  mpu.PrintActiveOffsets();
}

// Attach pin to servo, And set to middle
void initServo()
{
  serial_printlnF("setting DIGITAL PIN 4, 5, 6, 7 as OUTPUTS");

  servoAileron.attach(4); // FakeServo sets pinMode internal
  servoElevator.attach(5);
  servoAileron2.attach(6);
  servoRudder.attach(7);

  servoAileron.write(90);
  servoElevator.write(90);
  servoAileron2.write(90);
  servoRudder.write(90);
}

// Set PID dT
void initPID()
{
  uint16_t now = millis();
  rollPID.lastTime = now - rollPID.sampleTime;
  pitchPID.lastTime = now - pitchPID.sampleTime;
  yawPID.lastTime = now - yawPID.sampleTime;
}

/*==================================================================
                                Loop
===================================================================*/

// Calculate PID of target
void computePID(Pid *target, float input, int limitMin, int limitMax)
{
  unsigned long now = millis();
  unsigned long timeChange = now - target->lastTime;

  if (timeChange >= target->sampleTime)
  {
    float error = target->setpoint - input;
    float dInput = input - target->lastInput;

    target->total += KI * error;
    target->total = constrain(target->total, limitMin, limitMax);

    float output = KP * error;
    output += target->total - KD * dInput;
    target->output = constrain(output, limitMin, limitMax);

    target->lastInput = input;
    target->lastTime = now;
  }
}

// Setting Flight Control mode
// simplified version from ChatGPT using abs()
int getSystemSignal()
{
  if (abs(Channel.aux - FLIGHT_MODE_0) <= DEADZONE) // prüfe, ob Channel.aux in der Nähe von FLIGHT_MODE_0 liegt
    return 0;
  else if (abs(Channel.aux - FLIGHT_MODE_1) <= DEADZONE) // prüfe, ob Channel.aux in der Nähe von FLIGHT_MODE_1 liegt
    return 1;
  else if (abs(Channel.aux - FLIGHT_MODE_2) <= DEADZONE) // prüfe, ob Channel.aux in der Nähe von FLIGHT_MODE_2 liegt
    return 2;

  return 0;
}

// Get Channel PWM value
void getChannelInput()
{
  CPPM.cycle(); // update some variables and check for timeouts...

  if (CPPM.synchronized()) // only in sync at specific timespans (TODO: more buffer?)
  {
    Channel.pitch = CPPM.read_us(PITCH); // Receiver Pitch
    Channel.roll = CPPM.read_us(ROLL);   // Receiver Roll
    Channel.yaw = CPPM.read_us(YAW);     // Receiver Yaw
    Channel.aux = CPPM.read_us(AUX);   // Intensity Knob
  }
}

float gyroFiltered = 0.0f; // Gyro Z
float ypr[3];              // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
void getDmpYPR(int16_t *gyro, int16_t *accel, int32_t *quat)
{
  float dt, cTime;
  static float pTime = millis();    // Timer for Gyro Z
  Quaternion q;                     // [w, x, y, z]         quaternion container
  VectorFloat gravity;              // [x, y, z]            gravity vector
  float gyroZ;                      // Gyro Z

  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);

  // Calculate dt
  cTime = millis();
  dt = (cTime - pTime) / MILLI_SEC;
  pTime = cTime;

  gyroZ = (gyro[2] / FS_SEL_0_GYRO) * dt; // TODO: check is this really Z?!
  gyroFiltered = 0.98 * gyroFiltered + 0.02 * gyroZ;

}

// Get Alititude from ms5611, This function is currently not use.
// void getAltitude()
// {
//   realPressure = ms5611.readPressure();
//   absoluteAltitude = ms5611.getAltitude(realPressure);
//   relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);
// }

// Set value for Relative Control
void setAutoYPR()
{
  // Get sensor value
  Sensor.yaw = gyroFiltered * 1000;
  Sensor.pitch = ypr[1] * DEGREE_PER_PI;
  Sensor.roll = ypr[2] * DEGREE_PER_PI;

  // Conversion angle overflowed
  if (abs(Sensor.roll) > 90 && Sensor.pitch >= 0)
    Sensor.pitch = 180 - Sensor.pitch;
  else if (abs(Sensor.roll) > 90 && Sensor.pitch < 0)
    Sensor.pitch = -(180 + Sensor.pitch);
  else if (abs(Sensor.pitch) > 90 && Sensor.roll >= 0)
    Sensor.roll = 180 - Sensor.roll;
  else if (abs(Sensor.pitch) > 90 && Sensor.roll < 0)
    Sensor.roll = -(180 + Sensor.roll);

  // Conversion sensor angle to servo angle
  Sensor.yaw = 90 + Sensor.yaw;
  Sensor.pitch = 90 + Sensor.pitch;
  Sensor.roll = 90 - Sensor.roll;

  // Limits the angle
  Sensor.yaw = constrain(Sensor.yaw, 0, 180);
  Sensor.pitch = constrain(Sensor.pitch, 0, 180);
  Sensor.roll = constrain(Sensor.roll, 0, 180);
}

// Set value for PID Control mode
void setAutoPID()
{
  // Get sensor value
  Sensor.yaw = gyroFiltered * 1000;
  Sensor.pitch = ypr[1] * DEGREE_PER_PI;
  Sensor.roll = ypr[2] * DEGREE_PER_PI;

  // PID Control
  computePID(&rollPID, Sensor.roll, -90, 90);
  computePID(&pitchPID, Sensor.pitch, -90, 90);
  computePID(&yawPID, Sensor.yaw, -90, 90);

  // Change to Servo Value
  PIDFiltered.yaw = 90 + yawPID.output;
  PIDFiltered.roll = 90 - rollPID.output;
  PIDFiltered.pitch = 90 + pitchPID.output;

  // Limits the angle
  PIDFiltered.yaw = constrain(PIDFiltered.yaw, 0, 180);
  PIDFiltered.roll = constrain(PIDFiltered.roll, 0, 180);
  PIDFiltered.pitch = constrain(PIDFiltered.pitch, 0, 180);
}

// Relative Level Control
void relativeLeveling()
{
  if ((Channel.roll > AILERON_CHANNEL_OFFSET - DEADZONE) && (Channel.roll < AILERON_CHANNEL_OFFSET + DEADZONE))
  {
    ServoWrite(servoAileron, Sensor.roll);
    ServoWrite(servoAileron2, Sensor.roll); // inverted value for opposite one
  }
  else
  {
    ServoWriteMicroseconds(servoAileron, Channel.roll);
    ServoWriteMicroseconds(servoAileron2, Channel.roll);
  }

  if ((Channel.pitch > ELEVATOR_CHANNEL_OFFSET - DEADZONE) && (Channel.pitch < ELEVATOR_CHANNEL_OFFSET + DEADZONE))
  {
    ServoWrite(servoElevator, Sensor.pitch);
  }
  else
  {
    ServoWriteMicroseconds(servoElevator, Channel.pitch);
  }

  if ((Channel.yaw > YAW_CHANNEL_OFFSET - DEADZONE) && (Channel.yaw < YAW_CHANNEL_OFFSET + DEADZONE))
  {
    ServoWrite(servoRudder, Sensor.yaw);
  }
  else
  {
    ServoWriteMicroseconds(servoRudder, Channel.yaw);
  }
}

// PID Level Control
void pidLeveling()
{
  if ((Channel.roll > AILERON_CHANNEL_OFFSET - DEADZONE) && (Channel.roll < AILERON_CHANNEL_OFFSET + DEADZONE))
  {
    ServoWrite(servoAileron, PIDFiltered.roll);
    ServoWrite(servoAileron2, PIDFiltered.roll);
  }
  else
  {
    ServoWriteMicroseconds(servoAileron, Channel.roll);
    ServoWriteMicroseconds(servoAileron2, Channel.roll);
  }

  if ((Channel.pitch > ELEVATOR_CHANNEL_OFFSET - DEADZONE) && (Channel.pitch < ELEVATOR_CHANNEL_OFFSET + DEADZONE))
  {
    ServoWrite(servoElevator, PIDFiltered.pitch);
  }
  else
  {
    ServoWriteMicroseconds(servoElevator, Channel.pitch);
  }

  if ((Channel.yaw > YAW_CHANNEL_OFFSET - DEADZONE) && (Channel.yaw < YAW_CHANNEL_OFFSET + DEADZONE))
  {
    ServoWrite(servoRudder, PIDFiltered.yaw);
  }
  else
  {
    ServoWriteMicroseconds(servoRudder, Channel.yaw);
  }
}

// Manual Control
void manualFlightControl()
{
  ServoWriteMicroseconds(servoAileron, Channel.roll);
  ServoWriteMicroseconds(servoAileron2, Channel.roll);
  ServoWriteMicroseconds(servoElevator, Channel.pitch);
  ServoWriteMicroseconds(servoRudder, Channel.yaw);
}

void debugPrint()
{
  static unsigned int lastTime = millis();

  if (millis() - lastTime < 1000)
    return;

  lastTime = millis();
  // Print DMP Data
  String sStatus = "";
  switch (getSystemSignal())
  {
    case 0: // Manual Mode
      sStatus = "manual ";
      break;

    default: // case 1:   //Relative Control Mode
      sStatus = "relative ";
      break;

    case 2: // PID Control Mode
      sStatus = "PID ";
      break;
  }
  
  //Serial.print("status: ");
  Serial.print(sStatus);
  Serial.print(" Yaw:");
  Serial.print(Sensor.yaw);
  Serial.print(" Roll: ");
  Serial.print(Sensor.roll);
  Serial.print(" Pitch: ");
  Serial.print(Sensor.pitch);
  Serial.print(" Knob: ");
  Serial.print(Channel.aux);
  Serial.print(" Ail: ");
  Serial.print(Channel.roll);
  Serial.print(" Eler: ");
  Serial.print(Channel.pitch);
  Serial.print(" Rud: ");
  Serial.print(Channel.yaw);
  Serial.print(" PIDYaw: ");
  Serial.print(PIDFiltered.yaw);
  Serial.print(" PIDRoll: ");
  Serial.print(PIDFiltered.roll);
  Serial.print(" PIDPitch: ");
  Serial.println(PIDFiltered.pitch);
  // Serial.print("\tabAlt: ");
  // Serial.print(absoluteAltitude);
  // Serial.print("\trelAlt: ");
  // Serial.println(relativeAltitude);
}

// This function is currently not use
// Failsafe function
// void emergencyLevelling()
//{
//  rollPID.setpoint = 10;
//  pitchPID.setpoint = 10;
//  // TODO: need to apply throttle
//}

