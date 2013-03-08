
#ifndef ARDUINO_EZMOTOR_H
#define ARDUINO_EZMOTOR_H

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class Motor
{
public:
    Motor();
    void attach(int dir_pin0, int pwm_pin0);
	void attach(int dir_pin0, int dir_pin1, int pwm_pin0);
    void write(int speed);
    void flipDirection();
	void brake();
	void coast();
    boolean forward;

private:
    int dir_pin;
    int dir_pin2;
    int pwm_pin;
    boolean dir_HIGH;
};

#endif