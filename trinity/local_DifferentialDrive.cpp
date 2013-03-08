
#include "local_DifferentialDrive.h"
#include <stdarg.h>

DifferentialDrive::DifferentialDrive() : Motor()
{
    num_motors_attached = 0;
    current_heading = -1;
    desired_heading = -1;
    current_velocity = -1;
    desired_velocity = -1;
    pwm_limit = 255;
    base_pwm = 200;
    left_pwm = 0;
    right_pwm = 0;
    k_heading = 0.0;
    k_velocity = 0.0;
}

void DifferentialDrive::attachMotor(int dir_pin0, int pwm_pin0, boolean flip_dir)
{
    if (num_motors_attached == 0) {
    	// attach motor 1 (left motor)
    	left_motor.attach(dir_pin0, pwm_pin0);
        if (flip_dir) {
          left_motor.flipDirection();
        }
        num_motors_attached++;
    } else if (num_motors_attached == 1) {
    	// attach motor 2 (right motor)
    	right_motor.attach(dir_pin0, pwm_pin0);
        if (flip_dir) {
          right_motor.flipDirection();
        }
        num_motors_attached++;
    } else {
    	//ERROR: You can only attach two motors to a differential drive robot.
        Serial.println("ERROR: You can only attach two motors to a differential drive robot.");
    }
}

void DifferentialDrive::attachMotor(int dir_pin0, int pwm_pin0)
{
    if (num_motors_attached == 0) {
    	// attach motor 1 (left motor)
    	left_motor.attach(dir_pin0, pwm_pin0);
        num_motors_attached++;
    } else if (num_motors_attached == 1) {
    	// attach motor 2 (right motor)
    	right_motor.attach(dir_pin0, pwm_pin0);
        num_motors_attached++;
    } else {
    	//ERROR: You can only attach two motors to a differential drive robot.
        Serial.println("ERROR: You can only attach two motors to a differential drive robot.");
    }
}

void DifferentialDrive::attachMotor(int dir_pin0, int dir_pin1, int pwm_pin0)
{
    if (num_motors_attached == 0) {
    	// attach motor 1 (left motor)
    	left_motor.attach(dir_pin0, dir_pin1, pwm_pin0);
        num_motors_attached++;
    } else if (num_motors_attached == 1) {
    	// attach motor 2 (right motor)
    	right_motor.attach(dir_pin0, dir_pin1, pwm_pin0);
        num_motors_attached++;
    } else {
    	//ERROR: You can only attach two motors to a differential drive robot.
        Serial.println("ERROR: You can only attach two motors to a differential drive robot.");
    }
}

void DifferentialDrive::set_desired_heading(float h)
{
	desired_heading = h; // (in degrees with 90 = straight ahead)
}

void DifferentialDrive::set_desired_velocity(float v)
{
	desired_velocity = v;
}

void DifferentialDrive::set_current_heading(float h)
{
	current_heading = h; // (in degrees with 90 = straight ahead)
}

void DifferentialDrive::set_current_velocity(float v)
{
	current_velocity = v;
}

float DifferentialDrive::get_current_heading()
{
      return current_heading;
}

void DifferentialDrive::update_motors()
{
	// CONTROL LAWS GO HERE
	// e.g. ...
        float dTheta;
        float diff1 = current_heading - desired_heading;
        float diff2 = current_heading - (desired_heading - 360);
        if (abs(diff1) <= abs(diff2)) {
          dTheta = -diff1;
        } else {
          dTheta = -diff2;
        }
        
//	int dTheta = desired_heading - current_heading; // (in degrees)
	float dV = desired_velocity - current_velocity; // (in cm/s)
	
	//base_pwm += k_velocity*dV;
        base_pwm = 100;

        right_pwm = base_pwm - int(k_heading*dTheta);
        left_pwm = base_pwm + int(k_heading*dTheta);
         
        base_pwm = constrain(base_pwm, -255, 255);
        left_pwm = constrain(left_pwm, -255, 255);
        right_pwm = constrain(right_pwm, -255, 255);
        
        
        Serial.print(" CV: ");
        Serial.print(current_velocity);
        Serial.print(" dV: ");
        Serial.print(dV);
        Serial.print(" CH: ");
        Serial.print(current_heading);
        Serial.print(" dTheta: ");
        Serial.print(dTheta);
        Serial.print(" LPWM: ");
        Serial.print(left_pwm);
        Serial.print(" RPWM: ");
        Serial.println(right_pwm);
	
	left_motor.write(left_pwm);
	right_motor.write(right_pwm);
}
	
	
