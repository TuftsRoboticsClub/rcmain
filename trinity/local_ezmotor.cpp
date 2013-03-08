
#include "local_ezmotor.h"
#include <stdarg.h>

Motor::Motor()
{
    dir_pin   = -1;
    dir_pin2  = -1;
    pwm_pin   = -1;
    dir_HIGH = true;
    forward = -1;
}

void Motor::attach(int dir_pin0, int pwm_pin0)
{
    pinMode(dir_pin0, OUTPUT);
    pinMode(pwm_pin0, OUTPUT);
    dir_pin   = dir_pin0;
    pwm_pin = pwm_pin0;
}

void Motor::attach(int dir_pin0, int dir_pin1, int pwm_pin0)
{
    pinMode(dir_pin0, OUTPUT);
    pinMode(dir_pin1, OUTPUT);
    pinMode(pwm_pin0, OUTPUT);
    dir_pin   = dir_pin0;
    dir_pin2   = dir_pin1;
    pwm_pin = pwm_pin0;
}

void Motor::write(int speed)
{ 
	speed = constrain(speed,-255,255);
    if (speed < 0) {
        forward = true;
    	if (dir_HIGH) {
    		digitalWrite(dir_pin, HIGH);
    		if (dir_pin2 != -1) digitalWrite(dir_pin2, LOW);
    	} else {
    		digitalWrite(dir_pin, LOW);
    		if (dir_pin2 != -1) digitalWrite(dir_pin2, HIGH);
    	}
    	analogWrite(pwm_pin, -speed);
    } else {
      forward = false;
    	if (dir_HIGH) {
    		digitalWrite(dir_pin, LOW);
    		if (dir_pin2 != -1) digitalWrite(dir_pin2, HIGH);
    	} else {
    		digitalWrite(dir_pin, HIGH);
    		if (dir_pin2 != -1) digitalWrite(dir_pin2, LOW);
    	}
    	analogWrite(pwm_pin, speed);
	}
}

void Motor::flipDirection()
{
	dir_HIGH = !(dir_HIGH);
}

void Motor::brake()
{
	if (dir_pin2 != -1) {
		digitalWrite(dir_pin, HIGH);
		digitalWrite(dir_pin2, HIGH);
	} else {
		return; //"Error: Brake functionality not available for single direction input";
	}
}

void Motor::coast()
{
	analogWrite(pwm_pin, 0);
}
