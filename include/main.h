
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>

#define PWM_MAX 2000
#define PWM_MIN 1000
#define PWM_MID 1500

#define MPU_ADDRESS 0x68

// simple utility routine:
inline double mapf(double val, double in_min, double in_max, double out_min, double out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

enum CHANNEL {ROLL = 0, PITCH = 1, YAW = 2, KNOB = 3, CHANNEL_MAX} ;
enum GYRO {VAL = 0, CAL = 1, ACC = 2, GYRO_MAX} ;


struct pidgainStruct
{
  float p = 0;
  float i = 0;
  float d = 0;
  const int max = 400;
  const int max_i = 100;
};

struct channelValStruct 
{              // INPUT:              // OUTPUT:
  int ch1 = 0; // Receiver Roll       // ROLL
  int ch2 = 0; // Receiver Pitch      // PITCH
  int ch3 = 0; // Receiver Yaw        // INVERTED ROLL
  int ch4 = 0; // Intensity Knob      // YAW
};

struct gyroStruct
{
  long x = 0;
  long y = 0;
  long z = 0;
  long totalVector = 0;
};

struct angleValStruct
{
  float chan;
  float acc;
  float out;
  float adjust;
};

struct PIDStruct
{
  float i_mem;
  float input;
  float output;
  float setpoint;
  float d_error;
  float gyro;
};

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