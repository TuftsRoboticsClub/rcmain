
#include "local_trinbot.h"
#include "local_DifferentialDrive.h"


IRSensor::IRSensor()
{
    pin = -1;
}

void IRSensor::attach(byte pin0)
{
    pinMode(pin0, INPUT);
    pin = pin0;
}

float IRSensor::distance()
{
    // We are using the Sharp 2D120X sensor, which is different from the
    // Sharp GP2D12 sensor. The code for the latter at 
    // http://www.arduino.cc/playground/Main/ReadGp2d12Range cannot be used
    // for this reason. Instead, see
    // www.cs.uml.edu/teams-academy/uploads/Robots/Nonlinear.pdf
    // for a way to linearize the 2D120X sensor reading.
    int raw = analogRead(pin);
    return (9500.0 - 5.0 * (raw + 20.0)) / (3.0 * (raw + 20.0));
}

FlameSensor::FlameSensor()
{
    pin = -1;
}

void FlameSensor::attach(byte pin0)
{
    pinMode(pin0, INPUT);
    pin = pin0;
    lpfilter.init(2.5, analogRead(pin));
}

float FlameSensor::read_filtered()
{
    return lpfilter.step(analogRead(pin));
}

Trinbot::Trinbot() : DifferentialDrive()
{
  WHEEL_SEPARATION = 12.8;
  WHEEL_RADIUS = 5; //[cm] ****************** <--------- measure this!
  TICKS_PER_CM = 5.8399;
  TICKS_PER_DEG = 1.3083;
  enc_right_A = 3; // right encoder
  enc_right_B = 13; // right encoder
  enc_left_A = 2;  // left encoder
  enc_left_B = 8; // left encoder
  
  //Start Location
  locations[0].x_coord = 100;
  locations[0].y_coord = 23;
  locations[0].visited = 0;
  locations[0].name = "start";
  //Location 1 Top Left
  locations[1].x_coord = 36;
  locations[1].y_coord = 45; //.5
  locations[1].visited = 0;
  locations[1].name = "top left";
  //Location 2 Bottom Left
  locations[2].x_coord = 196; //.5
  locations[2].y_coord = 36;
  locations[2].visited = 0;
  locations[2].name = "bottom left";
  //Location 3 Bottom Right
  locations[3].x_coord = 188;
  locations[3].y_coord = 202;//.5
  locations[3].visited = 0;
  locations[3].name = "bottom right";
  //Location 4 Top Right Island
  locations[4].x_coord = 179;
  locations[4].y_coord = 51;
  locations[4].visited = 0;
  locations[4].name = "top right island";
    
  left_odom = 0; right_odom = 0, last_left_odom = 0, last_right_odom = 0, last_time = 0;
  
  xPos = locations[0].x_coord;
  yPos = locations[0].y_coord;
  
  wall_follow_dist = 10.0;
  front_obs_dist = 18.0;
}

void Trinbot::init()
{
  k_heading = 2.0;
  k_velocity = 1.0;
  line_thresh = 200;
  flameThresh = 40.0;
  leftLineThresh = 800;
  rightLineThresh = 800;
  co2Servo_up = 100;
  co2Servo_down = 0;
  
  attachMotor(4, 5);  // left motor
  attachMotor(7, 6);  // right motor
  left_motor.flipDirection();
  frontServo.attach(9);  
  co2Servo.attach(10);
  flDistSens.attach(A1);
  rlDistSens.attach(A9);
  frDistSens.attach(A11);
  rrDistSens.attach(A12);
  frontDistSens.attach(A10);
  flameSens.attach(A5);
  
  co2Servo.write(co2Servo_up);
  
  pinMode(11,INPUT);
  pinMode(12,INPUT);
  lineSensLeft.init((unsigned char[]) {11}, 1);
  lineSensRight.init((unsigned char[]) {12}, 1);
}

void Trinbot::update()
{
   int d_left_odom = left_odom - last_left_odom;
   int d_right_odom = right_odom - last_right_odom;
   last_left_odom = left_odom;
   last_right_odom = right_odom;
   int dt = millis() - last_time;
   last_time = millis();
   
   double wheel_difference = (d_left_odom-d_right_odom)/(2.0*TICKS_PER_CM);
   double dP = (d_left_odom + d_right_odom)/(2.0*TICKS_PER_CM); //[cm]
   //float dtheta = 180.0*((left_odom-right_odom)/WHEEL_SEPARATION)/PI; //[degrees] 
   
//   double dtheta = (180.0*atan2(wheel_difference,WHEEL_SEPARATION)/PI); //[degrees] 
   double dtheta = wheel_difference/TICKS_PER_DEG; //[degrees] 
   
   float velocity = dP*1000.0/dt; // [cm/s] average of the velocities of the wheels
   float heading = get_current_heading() + dtheta; // [cm/s] small angle assumption
   heading += 720;
   int numfits = heading/360; 
   heading -= numfits*360.0;
   
   xPos += dP*cos(heading*PI/180.0);
   yPos += dP*sin(heading*PI/180.0);
   
   Serial.print("xPos: ");
   Serial.print(xPos,DEC);
   Serial.print(" yPos: ");
   Serial.print(yPos,DEC);
   Serial.print(" theta: ");
   Serial.print(heading,DEC);
   
   int closestloc = getNearestLocation();
   
   Serial.print(" closestLoc: ");
   Serial.print(locations[closestloc].name);
   Serial.println(" ");
   
   set_current_velocity(velocity);
   set_current_heading(heading);
}

int Trinbot::getNearestLocation(){
  int closest_loc = -1;
  int leastDistance = 10000;
  double tempDistance = 0;
  int distx = 0, disty = 0; 
  
  for(int i = 0; i < 5; i ++){
    distx = (locations[i].x_coord - xPos);
    disty = (locations[i].y_coord - yPos);
    tempDistance =  sqrt((distx*distx)+(disty*disty));
    if(tempDistance < leastDistance){
      leastDistance = tempDistance;
      closest_loc = i;
    }
  }  
  return closest_loc;
}

void Trinbot::sweepServo(int start_angle, int end_angle)
{
  //start_millis = millis();
  frontServo.write(start_angle);
  delay(500);
  if (start_angle < end_angle) {
    for (int i = start_angle; i < end_angle; i+=20) {
      int angle = start_angle + i;
      frontServo.write(angle);
      delay(80);
      Serial.println(frontDistSens.distance());
    }
  } else {
    for (int i = end_angle; i <= start_angle; i+=20) {
      int angle = start_angle - i;
      frontServo.write(angle);
      delay(80);
      Serial.println(frontDistSens.distance());
    }
  }
}

int Trinbot::sweepForFlame()
{
  frontServo.write(90);
  delay(1000);
  float maxFlameVal = 0;
  int maxFlameAngle;
  float maxFlameDir;
  float flameVals[60];
  
  for (int i = 0; i < 60; i++) {
    flameVals[i] = 0;
  }
  
  for (int i = 87; i >= 0; i-=3) {
    frontServo.write(i);
    delay(40);
    float val = flameSens.read_filtered();
    
//    Serial.print(int(i/3),DEC);
    
    if (flameVals[int(i/3)] == 0) {
      flameVals[int(i/3)] = val;
//      Serial.println();
    } else {
      flameVals[int(i/3)] = (val + flameVals[int(i/3)])/2;
//      Serial.println(" avg");
    }
    
    if (val > maxFlameVal) {
      maxFlameVal = val;
      maxFlameAngle = i;
      maxFlameDir = 0;
    }
  }
  for (int i = 0; i < 180; i+=3) {
    frontServo.write(i);
    delay(40);
    float val = flameSens.read_filtered();
    
//    Serial.print(int(i/3),DEC);
    
    if (flameVals[int(i/3)] == 0) {
      flameVals[int(i/3)] = val;
//      Serial.println();
    } else {
      flameVals[int(i/3)] = (val + flameVals[int(i/3)])/2;
//      Serial.println(" avg");
    }
    
    if (val > maxFlameVal) {
      maxFlameVal = val;
      maxFlameAngle = i;
      maxFlameDir = 1;
    }
  }
  for (int i = 177; i >= 90; i-=3) {
    frontServo.write(i);
    delay(40);
    float val = flameSens.read_filtered();
    
//    Serial.print(int(i/3),DEC);
    
    if (flameVals[int(i/3)] == 0) {
      flameVals[int(i/3)] = val;
//      Serial.println();
    } else {
      flameVals[int(i/3)] = (val + flameVals[int(i/3)])/2;
//      Serial.println(" avg");
    }
    
    if (val > maxFlameVal) {
      maxFlameVal = val;
      maxFlameAngle = i;
      maxFlameDir = 0;
    }
  }
  
  
  float maxFlameVal2 = 0;
  int maxFlameAngle2 = 0;
  
  for (int i = 0; i < 60; i++) {
    if (maxFlameVal2 < flameVals[i]) {
      maxFlameVal2 = flameVals[i];
      maxFlameAngle2 = i*3;
    }
  }
  
  maxFlameAngle2 += 20;
  frontServo.write(maxFlameAngle2);
  
  Serial.print(maxFlameVal2);
  Serial.print(" ");
  Serial.print(maxFlameAngle2);
  Serial.print("  |  ");
  
  if (maxFlameDir == 1) {
    maxFlameAngle -= 20;
  } else {
    maxFlameAngle += 20;
  }
  
  Serial.print(maxFlameVal);
  Serial.print(" ");
  Serial.println(maxFlameAngle);
  delay(1000);
  frontServo.write(90);
  maxFlameReading = maxFlameVal2;
  maxFlameDir = maxFlameAngle2;
  return maxFlameAngle2;
}

void Trinbot::checkLineSensors()
{
  unsigned int leftLine, rightLine;
  lineSensLeft.read(&leftLine);
  lineSensRight.read(&rightLine);
  line_left = leftLine < line_thresh;
  line_right = rightLine < line_thresh;
}

void Trinbot::detectWalls()
{
   fr_wall = frDistSens.distance() < 30;
   rr_wall = rrDistSens.distance() < 30;
   fl_wall = flDistSens.distance() < 30;
   rl_wall = rlDistSens.distance() < 30;
   front_wall = frontDistSens.distance() < 10;
}

void Trinbot::followWall(WallSide side_to_follow)
{
  if(side_to_follow == Right){
    float f = frDistSens.distance();
    float r = rrDistSens.distance();
    float front = frontDistSens.distance();
    f = constrain(f, 3.0, 30.0);// - constrain(front, 3.0, 30.0)/2;
    r = constrain(r, 3.0, 30.0);
    front = constrain(front, 3.0, 30.0);
    float dD = (r - f);
    dD = - dD;

    
    if (front < front_obs_dist) {
      left_pwm = -125;
      right_pwm = 125;
    } else {
      /*
      if (abs(dD) <= 3.0) {
        float delta = f - wall_follow_dist;
        left_pwm = 125 + delta*3.0;
        right_pwm = 125 - delta*3.0;
      } else {
        left_pwm = 150 - dD*5.0;
        right_pwm = 150 + dD*5.0;
      }
      //*/
       if (abs(dD) <= 3.0) {
       // float delta = f - wall_follow_dist;
        left_pwm = 150 - dD*10.0;
        right_pwm = 150 + dD*10.0;
      } else {
        left_pwm = 150 - dD*20.0;
        right_pwm = 150 + dD*20.0;
      }
       
      
    }
         
  } else {
    float f = flDistSens.distance();
    float r = rlDistSens.distance();
    float front = frontDistSens.distance();
    f = constrain(f, 3.0, 30.0);
    r = constrain(r, 3.0, 30.0);
    front = constrain(front, 3.0, 30.0);
    float dD = (r - f);
    if (front < front_obs_dist) {
      left_pwm = 255;
      right_pwm = -255;
    } else {
      if (abs(dD) <= 3.0) {
        float delta = f - 15.0;
        left_pwm = 200 - delta*5.0;
        right_pwm = 200 + delta*5.0;
      } else {
        left_pwm = 150 + dD*10.0;
        right_pwm = 150 - dD*10.0;
      }
    }
  }
  left_pwm = constrain(left_pwm,-175,175);
  right_pwm = constrain(right_pwm,-175,175);
}

void Trinbot::followWallSkipGaps(WallSide side_to_follow)
{
  if(side_to_follow == Right){
    float f = frDistSens.distance();
    float r = rrDistSens.distance();
    float front = frontDistSens.distance();
    f = constrain(f, 3.0, 30.0);
    r = constrain(r, 3.0, 30.0);
    front = constrain(front, 3.0, 30.0);
    if (r == 30.0 && f == 30.0) {
      // no wall to follow --> go straight
      left_pwm = 255;
      right_pwm = 255;
      frontServo.write(135);
    } else if (r == 30.0) {
      // wall at front sensor
      float delta = f - wall_follow_dist;
      left_pwm = 200 + delta*5.0;
      right_pwm = 200 - delta*5.0;
      frontServo.write(135);
    } else if (f == 30.0) {
      // wall at rear sensor
      float delta = r - wall_follow_dist;
      left_pwm = 200 + delta*5.0;
      right_pwm = 200 - delta*5.0;
      frontServo.write(135);
    } else {
      // there's a wall at both sensors
      frontServo.write(90);
      float dD = (r - f);
      if (abs(dD) <= 3.0) {
        float delta = f - wall_follow_dist;
        left_pwm = 200 + delta*5.0;
        right_pwm = 200 - delta*5.0;
      } else {
        left_pwm = 150 - dD*10.0;
        right_pwm = 150 + dD*10.0;
      }
    }
    if (front < front_obs_dist) {
      left_pwm = -255;
      right_pwm = 255;
    }
  } else {
    float f = flDistSens.distance();
    float r = rlDistSens.distance();
    float front = frontDistSens.distance();
    f = constrain(f, 3.0, 30.0);
    r = constrain(r, 3.0, 30.0);
    front = constrain(front, 3.0, 30.0);
    if (r == 30.0 && f == 30.0) {
      // no wall to follow --> go straight
      left_pwm = 255;
      right_pwm = 255;
      frontServo.write(45);
    } else if (r == 30.0) {
      // wall at front sensor
      float delta = f - wall_follow_dist;
      left_pwm = 200 - delta*5.0;
      right_pwm = 200 + delta*5.0;
      frontServo.write(45);
    } else if (f == 30.0) {
      // wall at rear sensor
      float delta = r - wall_follow_dist;
      left_pwm = 200 - delta*5.0;
      right_pwm = 200 + delta*5.0;
      frontServo.write(45);
    } else {
      // there's a wall at both sensors
      frontServo.write(90);
      float dD = (r - f);
      if (abs(dD) <= 3.0) {
        float delta = f - wall_follow_dist;
        left_pwm = 200 - delta*5.0;
        right_pwm = 200 + delta*5.0;
      } else {
        left_pwm = 150 + dD*10.0;
        right_pwm = 150 - dD*10.0;
      }
    }
    if (front < front_obs_dist) {
      left_pwm = 255;
      right_pwm = -255;
    }
  }
  
}

void Trinbot::extinguish()
{
  co2Servo.write(co2Servo_down);
  delay(1000);
  co2Servo.write(co2Servo_up);
//  delay(1000);
}

void debug(char *fmt, ...)
{
    char tmp[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, 128, fmt, args);
    va_end(args);
    Serial.print(tmp);
}
