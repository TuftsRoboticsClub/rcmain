
#ifndef DifferentialDrive_H
#define DifferentialDrive_H

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "local_ezmotor.h"

class DifferentialDrive : public Motor
{
public:
    DifferentialDrive();
    void attachMotor(int dir_pin0, int pwm_pin0, boolean flip_dir);
    void attachMotor(int dir_pin0, int pwm_pin0);
    void attachMotor(int dir1, int dir2, int pwm);
    void set_desired_heading(float h);
    void set_desired_velocity(float v);
    void update_motors();
    void set_current_heading(float h);
    float get_current_heading();
    void set_current_velocity(float v);
    float k_heading;
    float k_velocity;
    int left_pwm;
    int right_pwm;
    Motor left_motor, right_motor;

private:
	
	int num_motors_attached;
	float current_heading;
	float desired_heading;
	float current_velocity;
	float desired_velocity;
	int pwm_limit;
	int base_pwm;
	
	
};

#endif