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

#endif 
