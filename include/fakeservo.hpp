#ifndef FAKESERVO_H
#define FAKESERVO_H

#include <Arduino.h>

#ifndef PWM_MAX
#define PWM_MAX 2000
#endif

#ifndef PWM_MIN
#define PWM_MIN 1000
#endif

#ifndef PWM_MID
#define PWM_MID 1500
#endif

#ifndef ANGLE_MIN
#define ANGLE_MIN 0
#endif

#ifndef ANGLE_MAX
#define ANGLE_MAX 180
#endif

// a thin mapping class to avoid using interrupts for servo at all (see adjustServos())
class FakeServo
{

private:
  byte _pin, _min, _max;
  uint16_t _microSeconds;
  bool _inverted = false;

public:
  FakeServo(){};

  byte attach(byte pin, byte min = ANGLE_MIN, byte max = ANGLE_MAX, bool inverted = false)
  {
    _min = min;
    _max = max;
    _inverted = inverted;
    _pin = pin;
    pinMode(_pin, OUTPUT);
    return _pin;
  }; 
  void write(float value)
  {
    value = mapf(constrain(value, _min, _max), _min, _max, PWM_MIN, PWM_MAX);
    writeMicroseconds(value);
  };

  void writeMicroseconds(uint16_t value)
  {
    value = constrain(value, PWM_MIN, PWM_MAX); // ensure pulse width is valid
    if (_inverted)
      value = PWM_MIN + PWM_MAX - value;                                   // invert if requested
    _microSeconds = value;                                                 // convert to ticks after compensating for interrupt overhead - 12 Aug 2009
  };                                                                       // Write pulse width in microseconds
  byte read() { return map(_microSeconds + 1, PWM_MIN, PWM_MAX, _min, _max); }; // returns current pulse width as an angle between 0 and 180 degrees
  uint16_t readMicroseconds() { return _microSeconds; };                    // returns current pulse width in microseconds for this servo (was read_us() in first release)

  // only dummies
  void detach();
  bool attached(); // return true if this servo is attached, otherwise false
};

#endif
