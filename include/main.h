
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>

#define PWM_MAX 2000
#define PWM_MIN 1000
#define PWM_MID 1500

#define MPU_ADDRESS 0x68

#define ServoWrite(servo, degree) servo.write(degree)
#define ServoWriteMicroseconds(servo, ms) servo.writeMicroseconds(ms)

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

// a thin mapping class to avoid using interrupts for servo at all (see adjustServos())
class FakeServo
{

private:
 int _pin, _min, _max, _microSeconds;

public:
  FakeServo(){};
  uint8_t attach(int pin){_pin = pin;pinMode(_pin, OUTPUT); return _pin;};                                          // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(int pin, int min, int max) {_pin = pin; _min=min; _max = max; return _pin;}; // as above but also sets min and max values for writes. 
  void detach();

  void write(int value)
  {
    value = constrain(value, 0, 180);
    value = map(value, 0, 180, PWM_MIN, PWM_MAX);
    writeMicroseconds(value);
  }; 

  void writeMicroseconds(int value)
  {
    value = constrain(value, PWM_MIN, PWM_MAX);                             // ensure pulse width is valid
    _microSeconds = value;                                                  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009
  };                                                                        // Write pulse width in microseconds
  int read() { return map(_microSeconds + 1, PWM_MIN, PWM_MAX, 0, 180); };  // returns current pulse width as an angle between 0 and 180 degrees
  int readMicroseconds() { return _microSeconds; };                         // returns current pulse width in microseconds for this servo (was read_us() in first release)
  bool attached();                                                          // return true if this servo is attached, otherwise false 
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