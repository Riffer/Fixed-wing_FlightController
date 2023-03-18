#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>

#define PWM_MAX 2000
#define PWM_MIN 1000
#define PWM_MID 1500


template <typename T>
T mapT(T val, T in_min, T in_max, T out_min, T out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

#define ServoWrite(servo, degree) servo.write(mapT((double)degree, (double)0, (double)CENTER_OF_SERVO * 2, (double)PWM_MIN, (double)PWM_MAX))
#define ServoWriteMicroseconds(servo, ms) servo.write(ms)

void initMPU6050();
void initServo();
void initPID();
void copyChannelInput();
void getDmpYPR(int16_t *gyro, int16_t *accel, int32_t *quat);
int getSystemSignal();
void manualFlightControl();
void setAutoYPR();
void relativeLeveling();
void setAutoPID();
void pidLeveling();
void debugPrint();
void readGyroData();

const float DEGREE_PER_PI = 180 / M_PI;

enum CHANNEL
{
  ROLL = 0,
  PITCH = 1,
  YAW = 2,
  AUX = 3,
  CHANNEL_MAX
};
enum GYRO
{
  VAL = 0,
  CAL = 1,
  ACC = 2,
  GYRO_MAX
};

typedef struct RPY
{
  float roll, pitch, yaw, aux;
} RPY;
typedef struct dRPY
{
  double roll, pitch, yaw;
} dRPY;
typedef struct Pid // For PID Controll
{
  float total, output, lastInput, setpoint;
  unsigned long lastTime, sampleTime = 100;
} Pid;


ServoTimer2 servoRudder, servoAileron, servoElevator, servoAileron2;

Simple_MPU6050 mpu;

RPY Sensor;       // Data of Axis from MPU6050
RPY Channel;      // Data of Axis from Receiver
dRPY PIDFiltered; // Data of Axis from PID Function
Pid rollPID, pitchPID, yawPID;

#define DEBUG 1 //0 for turn off, 1 for turn on - this works function wise and the compiler optimizes if(0){} out


//for debugging output
#define serial_printF(x, ...)                     \
    do                                           \
    {                                            \
        if (DEBUG) \
            Serial.print(F(x), ##__VA_ARGS__);      \
    } while (0)

#define serial_print(x, ...)                     \
    do                                           \
    {                                            \
        if (DEBUG) \
            Serial.print(x, ##__VA_ARGS__);      \
    } while (0)

#define serial_printlnF(x, ...)                   \
    do                                           \
    {                                            \
        if (DEBUG) \
            Serial.println(F(x), ##__VA_ARGS__);    \
    } while (0)

#define serial_println(x, ...)                   \
    do                                           \
    {                                            \
        if (DEBUG) \
            Serial.println(x, ##__VA_ARGS__);    \
    } while (0)



#endif // MAIN_H