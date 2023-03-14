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
#include <MS5611.h>
#include <Simple_Wire.h>
#include <Simple_MPU6050.h>
#include <jm_CPPM.h>

#include "main.h" 
#include "fakeservo.h" // simple thin fake servo class

#define MPU6050_ADDR            0x68            //Address of MPU-6050
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_ACCEL_XOUT_H    0x3B
#define INTERRUPT_PIN           2               // use pin 2 on Arduino Uno
#define CALLIBRATE_DPM_OFFSET   6
#define MILLI_SEC               1000
#define FS_SEL_0_GYRO           131.0

// Set Offset, scaled for min sensitivity
#define GYRO_OFFSET_X           120
#define GYRO_OFFSET_Y           -22
#define GYRO_OFFSET_Z           -4
#define ACC_OFFSET_X            -2927
#define ACC_OFFSET_Y            -339
#define ACC_OFFSET_Z            1836

#define KP 1.0
#define KI 0.1
#define KD 1.0
#define CENTER_OF_SERVO 90

#define DEADZONE 50
#define AILERON_CHANNEL_OFFSET 1504
#define ELEVATOR_CHANNEL_OFFSET 1464
#define YAW_CHANNEL_OFFSET 1500

#define FLIGHT_MODE_0 PWM_MAX
#define FLIGHT_MODE_1 PWM_MID
#define FLIGHT_MODE_2 PWM_MIN

const float DEGREE_PER_PI = 180 / M_PI; 

Simple_MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;          // set true if DMP init was successful
uint8_t mpuIntStatus;           // holds actual interrupt status byte from MPU
uint8_t devStatus;              // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;            // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;             // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         // FIFO storage buffer

Quaternion q;                   // [w, x, y, z]         quaternion container
VectorFloat gravity;            // [x, y, z]            gravity vector
float ypr[3];                   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//int systemStatus = 0;
FakeServo servoRudder, servoAileron, servoElevator, servoAileron2;      // Servo Motor
float rollSensor, pitchSensor, yawSensor;                       // Data of Axis from MPU6050
float rollChannel, pitchChannel, yawChannel;                    // Data of Axis from Receiver
double rollPidFiltered, pitchPidFiltered, yawPidFiltered;       // Data of Axis from PID Function
float knobChannel;                                              // Flightmodes

// MS5611 ms5611;
// long realPressure;                                            // Pressure value
// double referencePressure;                                     // Reference P
// float absoluteAltitude, relativeAltitude;                     // Altitude

float gyroZ, gyroFiltered;                                      // Gyro Z
VectorInt16 gy;                                                 // Raw data of Gyro Z

typedef struct Pid                                              // For PID Controll
{
  float total, output, lastInput, setpoint;
  unsigned long lastTime, sampleTime = 100;
}Pid;

Pid rollPID, pitchPID, yawPID;                              


void initMPU6050();
void verifyConnection();
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
void printYPRToSerial();
void readGyroData();
void adjustServos();

void setup()
{
  Serial.begin(115200); // Initialize serial communication
  while (!Serial)
    delay(30);
  ; // Wait for Leonardo enumeration, others continue immediately

  CPPM.begin(); // setup CPPM - will be called in loop

  initMPU6050();                                  // Initialize MPU6050 for I2C

//  ms5611.begin();
//  referencePressure = ms5611.readPressure();
//  Serial.println(ms5611.getOversampling());
  
  verifyConnection();                             // Veritfy connection and wait for start
  Serial.println(F("Initializing DMP..."));
  // devStatus = mpu.dmpInitialize();                // Load and configure the DMP
  mpu.load_DMP_Image();                 // Load and configure the DMP
  devStatus = 0;
  setGyroAccOffset();                             // Supply your own gyro offsets here, scaled for min sensitivity
  //calibrateDMP();                                 // Calibrate DMP

  initServo();                              
  initPID();

  pTime = millis();                               // Initialize dt
}

void loop()
{
  /*-------------Using DMP Preset------------*/
  getChannelInput(); // get channel input from CPPM (se jm_CPPM.h "CPPM_ICP1" for PIN)

  // getAltitude();

  mpu.dmp_read_fifo(false); // can be replaced by interrupt on pin "interruptPin" from MPU (but maybe overflowing CPU)

  switch (getSystemSignal()) // use flight mode
  {
  case 0:   //Manual Mode
    manualFlightControl();
    break;
  
  case 2:   //PID Control Mode
    yawPID.setpoint = 0;
    pitchPID.setpoint = 0;
    rollPID.setpoint = 0;
    setAutoPID();
    pidLeveling();
    break;

  default: // case 1:   //Relative Control Mode
    setAutoYPR();
    relativeLeveling();
    break;
  }
  adjustServos();
  printYPRToSerial();
}

/*==================================================================
                                Setup
===================================================================*/

void initMPU6050()
{
  Serial.println(F("Initializing I2C devices..."));
  mpu.begin();
  mpu.CalibrateMPU(); // Calibrates the MPU.
  mpu.load_DMP_Image();
  mpu.on_FIFO(getDmpYPR);
}

void verifyConnection()               
{
  //Verify connection
  Serial.println(F("Testing device connections...\nMPU6050 connection "));
  Serial.println(mpu.TestConnection() ? F("successful") : F("failed"));
} 

void setGyroAccOffset()             
{
  // Supply your own gyro offsets here, scaled for min sensitivity
  mpu.setOffset(ACC_OFFSET_X, ACC_OFFSET_Y, ACC_OFFSET_Z, GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z);
}

void calibrateDMP()
{
  // The code from "MPU6050_DMP6_using_DMP_V6.12.ino" by Jeff Rowberg <jeff@rowberg.net>
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate MPU6050
    mpu.CalibrateAccel(CALLIBRATE_DPM_OFFSET);
    mpu.CalibrateGyro(CALLIBRATE_DPM_OFFSET);
    mpu.PrintActiveOffsets();

    // Enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    
    // Set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // Get expected DMP packet size for later comparison
    packetSize = mpu.packet_length;
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

// Attach pin to servo, And set to middle
void initServo()
{
  serial_printlnF("setting DIGITAL PIN 4, 5, 6, 7 as OUTPUTS");

  servoAileron.attach(4); // using FakeServo sets pinMode internall
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
  rollPID.lastTime = millis()-rollPID.sampleTime;
  pitchPID.lastTime = millis()-pitchPID.sampleTime;
  yawPID.lastTime = millis()-yawPID.sampleTime;
}

/*==================================================================
                                Loop
===================================================================*/

// Calculate PID of target
void computePID(Pid *target,float input, int limitMin, int limitMax)
{
  unsigned long now = millis();
  unsigned long timeChange = now - target->lastTime;

  if(timeChange >= target->sampleTime)
  {
    float error = target->setpoint - input;
    float dInput = input - target-> lastInput;
    
    target->total += KI * error;

    if(target->total > limitMax)
      target->total = limitMax;
    else if(target->total < limitMin)
      target->total = limitMin;

    float output = KP * error;
    output += target->total - KD * dInput;

    if(output > limitMax)
      output = limitMax;
    else if(output < limitMin)
      output = limitMin;

    target->output = output;
  
    target->lastInput = input;
    target->lastTime = now;
  }
}

// Setting Flight Control mode
// simplified version from ChatGPT using abs()
int getSystemSignal()
{
  if (abs(knobChannel - FLIGHT_MODE_0) <= DEADZONE) // prüfe, ob knobChannel in der Nähe von FLIGHT_MODE_0 liegt
    return  0;
  else if (abs(knobChannel - FLIGHT_MODE_1) <= DEADZONE) // prüfe, ob knobChannel in der Nähe von FLIGHT_MODE_1 liegt
    return 1;
  else if (abs(knobChannel - FLIGHT_MODE_2) <= DEADZONE) // prüfe, ob knobChannel in der Nähe von FLIGHT_MODE_2 liegt
    return 2;

  return 0;
}

// Get Channel PWM value
void getChannelInput()
{
  CPPM.cycle(); // update some variables and check for timeouts...

  if (CPPM.synchronized()) // only in sync at specific timespans (TODO: more buffer?)
  {
    pitchChannel = CPPM.read_us(PITCH); //Receiver Pitch
    rollChannel = CPPM.read_us(ROLL); // Receiver Roll
    yawChannel = CPPM.read_us(YAW); // Receiver Yaw
    knobChannel = CPPM.read_us(KNOB); // Intensity Knob
  }
}

void getDmpYPR(int16_t *gyro, int16_t *accel, int32_t *quat)
{
  float dt, cTime;
  static float pTime; // Timer for Gyro Z

  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);

  // Calculate dt
  cTime = millis();
  dt = (cTime - pTime) / MILLI_SEC;
  pTime = cTime;

  gyroZ = (gyro[2] / FS_SEL_0_GYRO) * dt; //TODO: check is this really Z?!
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
  yawSensor = gyroFiltered * 1000;
  pitchSensor = ypr[1] * DEGREE_PER_PI;
  rollSensor = ypr[2] * DEGREE_PER_PI;

  // Conversion angle overflowed
  if((rollSensor > 90 || rollSensor < -90) && (pitchSensor >= 0)) 
    pitchSensor = 180 - pitchSensor;
  else if((rollSensor > 90 || rollSensor < -90) && (pitchSensor < 0))
    pitchSensor = -(180 + pitchSensor);
  else if((pitchSensor > 90 || pitchSensor < -90) && (rollSensor >= 0)) 
    rollSensor = 180 - rollSensor;
  else if((pitchSensor > 90 || pitchSensor < -90) && (rollSensor < 0))
    rollSensor = -(180 + rollSensor);

  // Conversion sensor angle to servo angle
  yawSensor = 90 + yawSensor;
  pitchSensor = 90 + pitchSensor;
  rollSensor = 90 - rollSensor;

  // Limits the angle
  if(yawSensor > 180)
    yawSensor = 180;
  else if(yawSensor < 0)
    yawSensor = 0;

  if(pitchSensor > 180)
    pitchSensor = 180;
  else if(pitchSensor < 0)
    pitchSensor = 0;

  if(rollSensor > 180)
    rollSensor = 180;
  else if(rollSensor < 0)
    rollSensor = 0;
}

// Set value for PID Control mode
void setAutoPID()
{
  // Get sensor value
  yawSensor = gyroFiltered * 1000;
  pitchSensor = ypr[1] * DEGREE_PER_PI;
  rollSensor = ypr[2] * DEGREE_PER_PI;

  // PID Control
  computePID(&rollPID, rollSensor, -90, 90);
  computePID(&pitchPID, pitchSensor, -90, 90);
  computePID(&yawPID, yawSensor, -90, 90);

  // Change to Servo Value
  yawPidFiltered = 90 + yawPID.output;
  rollPidFiltered = 90 - rollPID.output;
  pitchPidFiltered = 90 + pitchPID.output;

  // Limits the angle
  if(yawPidFiltered > 180)
    yawPidFiltered = 180;
  else if(yawPidFiltered < 0)
    yawPidFiltered = 0;

  if(rollPidFiltered > 180)
    rollPidFiltered = 180;
  else if(rollPidFiltered < 0)
    rollPidFiltered = 0;

  if(pitchPidFiltered > 180)
    pitchPidFiltered = 180;
  else if(pitchPidFiltered < 0)
    pitchPidFiltered = 0;
}

// Relatvie Level Control
void relativeLeveling()
{
  if((rollChannel > AILERON_CHANNEL_OFFSET - DEADZONE) && (rollChannel < AILERON_CHANNEL_OFFSET + DEADZONE))
  {
    //servoAileron.write(rollSensor); // value from 0 to 180
    ServoWrite(servoAileron, rollSensor);
    ServoWrite(servoAileron2, 180 - rollSensor); // inverted value for opposite one
  }
  else
  {
    //servoAileron.writeMicroseconds(rollChannel); // value from 1000 to 2000
    ServoWriteMicroseconds(servoAileron, rollChannel);
    ServoWriteMicroseconds(servoAileron2, 3000 - rollChannel);
  }

  if((pitchChannel > ELEVATOR_CHANNEL_OFFSET - DEADZONE) && (pitchChannel < ELEVATOR_CHANNEL_OFFSET + DEADZONE))
  {
    //servoElevator.write(pitchSensor);
    ServoWrite(servoElevator, pitchSensor);
  }
  else
  {
    //servoElevator.writeMicroseconds(pitchChannel);
    ServoWriteMicroseconds(servoElevator, pitchChannel);
  }
  
  if((yawChannel > YAW_CHANNEL_OFFSET - DEADZONE) && (yawChannel < YAW_CHANNEL_OFFSET + DEADZONE))
  {
    //servoRudder.write(yawSensor);
    ServoWrite(servoRudder, yawSensor);
    //ServoWrite(servoAileron2, 180 - yawSensor); // interted value
  }
  else
  {
    //servoRudder.writeMicroseconds(yawChannel);
    //servoAileron2.writeMicroseconds(3000 - yawChannel);
    ServoWriteMicroseconds(servoRudder, yawChannel);
    //ServoWriteMicroseconds(servoAileron2, 3000 - yawChannel);

  }
}

// PID Level Control
void pidLeveling()
{
  if((rollChannel > AILERON_CHANNEL_OFFSET - DEADZONE) && (rollChannel < AILERON_CHANNEL_OFFSET + DEADZONE))
  {
    //servoAileron.write(rollPidFiltered);
    ServoWrite(servoAileron, rollPidFiltered);
    ServoWrite(servoAileron2, 180 - rollPidFiltered);
  }
  else
  {
    //servoAileron.writeMicroseconds(rollChannel);
    ServoWriteMicroseconds(servoAileron, rollChannel);
    ServoWriteMicroseconds(servoAileron2, 3000 - rollChannel);
  }
  
  if((pitchChannel > ELEVATOR_CHANNEL_OFFSET - DEADZONE) && (pitchChannel < ELEVATOR_CHANNEL_OFFSET + DEADZONE))
  {
    //servoElevator.write(pitchPidFiltered);
    ServoWrite(servoElevator, pitchPidFiltered);
  }
  else
  {
    //servoElevator.writeMicroseconds(pitchChannel);
    ServoWriteMicroseconds(servoElevator, pitchChannel);
  }
  
  if((yawChannel > YAW_CHANNEL_OFFSET - DEADZONE) && (yawChannel < YAW_CHANNEL_OFFSET + DEADZONE))
  {
    //servoRudder.write(yawPidFiltered);
    ServoWrite(servoRudder, yawPidFiltered);
  }
  else
  {
    //servoRudder.writeMicroseconds(yawChannel);
    //servoAileron2.writeMicroseconds(3000 - yawChannel);
    ServoWriteMicroseconds(servoRudder, yawChannel);
    //ServoWriteMicroseconds(servoAileron2, 3000-yawChannel);
  }
}

// Manual Control
void manualFlightControl()
{
  //servoAileron.writeMicroseconds(rollChannel);
  //servoElevator.writeMicroseconds(pitchChannel);
  //servoRudder.writeMicroseconds(yawChannel);
  //servoAileron2.writeMicroseconds(3000 - yawChannel);

ServoWriteMicroseconds(servoAileron, rollChannel);
ServoWriteMicroseconds(servoAileron2, 3000 - rollChannel);
ServoWriteMicroseconds(servoElevator, pitchChannel);
ServoWriteMicroseconds(servoRudder, yawChannel);

}

void printYPRToSerial()
{
static unsigned int lastTime = millis();

if (millis() - lastTime < 1000)
    return;

lastTime = millis();
// Print DMP Data
Serial.print("status: ");
Serial.print(getSystemSignal());
Serial.print(" Yaw:");
Serial.print(yawSensor);
Serial.print(" Roll: ");
Serial.print(rollSensor);
Serial.print(" Pitch: ");
Serial.print(pitchSensor);
Serial.print(" Knob: ");
Serial.print(knobChannel);
Serial.print(" Ail: ");
Serial.print(rollChannel);
Serial.print(" Eler: ");
Serial.print(pitchChannel);
Serial.print(" Rud: ");
Serial.print(yawChannel);
Serial.print(" PIDYaw: ");
Serial.print(yawPidFiltered);
Serial.print(" PIDRoll: ");
Serial.print(rollPidFiltered);
Serial.print(" PIDPitch: ");
Serial.println(pitchPidFiltered);
// Serial.print("\tabAlt: ");
// Serial.print(absoluteAltitude);
// Serial.print("\trelAlt: ");
// Serial.println(relativeAltitude);
}

// This function is currently not use
// Failsafe function
//void emergencyLevelling()
//{
//  rollPID.setpoint = 10;
//  pitchPID.setpoint = 10;
//  // TODO: need to apply throttle
//}

void adjustServos()
{
static int loopCounter = 0;
static unsigned long loop_start_time = micros();

// wait until at least 0,4 miliseconds gone by (1000 micros are 1 milis, 1 second has 1.000.000 micros!) since last time
// while (micros() - loop_start_time < 4000)
//  delayMicroseconds(4000 - (micros() - loop_start_time)); // originally this just looped, but delay does not consume CPU power

while (micros() - loop_start_time < 4000)
    ;

  loop_start_time = micros(); // set loop_start_time to current value for next call

  loopCounter++;
  if (loopCounter >= 0)
  {

    loopCounter = 0;
    PORTD |= B11110000;
    unsigned long timer_channel_1 = servoAileron.readMicroseconds() + loop_start_time;
    unsigned long timer_channel_2 = servoElevator.readMicroseconds() + loop_start_time;
    unsigned long timer_channel_3 = servoAileron2.readMicroseconds() + loop_start_time;
    unsigned long timer_channel_4 = servoRudder.readMicroseconds() + loop_start_time;

    // PWM out in a loop - initally set high for all 4 channels and look until all 4 channels gone by
    byte cnt = 0;
    while (cnt < 4) // leading to slowdown of the loop
    {
      cnt = 0;
      unsigned long esc_loop_start_time = micros();
      if (timer_channel_1 <= esc_loop_start_time)
      {
        PORTD &= B11101111;
        cnt++;
      }
      if (timer_channel_2 <= esc_loop_start_time)
      {
        PORTD &= B11011111;
        cnt++;
      }
      if (timer_channel_3 <= esc_loop_start_time)
      {
        PORTD &= B10111111;
        cnt++;
      }
      if (timer_channel_4 <= esc_loop_start_time)
      {
        PORTD &= B01111111;
        cnt++;
      }
    }
  }
}

