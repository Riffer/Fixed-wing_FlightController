#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>

#define PWM_MAX 2000
#define PWM_MIN 1000
#define PWM_MID 1500

#define MPU_ADDRESS 0x68

/*
template <typename T>
T mapT(T val, T in_min, T in_max, T out_min, T out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/


// simple utility routine:
inline double mapf(double val, double in_min, double in_max, double out_min, double out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

enum CHANNEL {ROLL = 0, PITCH = 1, YAW = 2, AUX = 3, CHANNEL_MAX} ;
enum GYRO {VAL = 0, CAL = 1, ACC = 2, GYRO_MAX} ;

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