/*
USE_GITHUB_USERNAME=langfordw

Gist/GIT Repo: https://gist.github.com/2096323

To Do List: (last updated 03/20/12)
- Verify that we stop in the home circle in "ReturnHome"
- Improve or replace FollowWallSkipGaps. It's currently not very consistent.
- Implement a smarter ReturnHome such that every single wall doesn't need to be retraced.
- Add constant distance sensor checks + interrupt capabilities to avoid danger while maintaning the current state.
 */
 /* Jonah Kadoko
 
 Notes:
 writing the code for Jumbotron
 To Do List:
 * Test front Servo
 * Test extinguish servo
 * Test IR Sensors
 * 
 */
 
#include "local_trinbot.h"

int flame=0;
const int FLAME_PLATFORM = 500;
Trinbot trinbot;
struct waypoint { 
  float x; 
  float y; 
};

WallSide last_wall, current_wall, wall_to_follow_in_room;

int counter;
int short_delay = 250, long_delay = 500;   // this is for 10V motor power, double both for 7V
long start_time, start_dOdom;
int flameAngle;

int state;
const int HallwayNavigation = 10,
          AlignToLine = 11,
          CheckRoom = 12,
          RoomNavigation = 13,
          AlignToCircle = 14,
          FinalFlameSweep = 15,
          Extinguish = 16,
          ReturnHome = 17,
          HallwayNavigation2 = 18;

waypoint trin_waypoints[20];
int num_waypoints;

void setup() {
  Serial.begin(57600);

  trinbot.init();

  // attach encoders
  attachInterrupt(0, read_quad_left, CHANGE); // left encoder
  attachInterrupt(1, read_quad_right, CHANGE); // right encoder

  initialize_waypoints();

  trinbot.set_current_velocity(0);
  trinbot.set_current_heading(90);
  trinbot.set_desired_velocity(15);
  
  state = HallwayNavigation;
  state = 0;
  trinbot.left_motor.write(255);
  trinbot.right_motor.write(255);
  delay(1000);
}

void loop() {
  //update_waypoint();
//  update_current_state();
  //trinbot.update_motors();
  
  //trinbot.update();
  // Testing the servos
 // trinbot.frontServo.write(90);
 //trinbot.sweepForFlame(); // helps avoid head on collisions with the small wall
  // trinbot.co2Servo.write(140);
   // flame sensor
  // Serial.print("Flame Sensor");
  //Serial.println(trinbot.flameSens.read_filtered());
  //Serial.print(" ");
  //trinbot.left_motor.write(100);
  //trinbot.right_motor.write(100);
  //trinbot.followWall(Right);
 
 trinbot.followWall(Right); 
  unsigned int leftLine, rightLine;
  trinbot.lineSensLeft.read(&leftLine);
  trinbot.lineSensRight.read(&rightLine);
  
  Serial.print(constrain(trinbot.frDistSens.distance(),3.0,30.0));
  Serial.print("    ");    
  Serial.println(constrain(trinbot.rrDistSens.distance(),3.0,30.0));
 /*
 Serial.print(leftLine);
Serial.print(" ");
 Serial.print(rightLine);
 Serial.println(" ");
 
 Serial.print(150 - (trinbot.frDistSens.distance()-trinbot.rrDistSens.distance())*2.0);
 Serial.print("  ");
 Serial.println(150 + (trinbot.frDistSens.distance()-trinbot.rrDistSens.distance())*2.0);
       // right_pwm = 150 + dD*2.0;
        
   /*  
   
 Serial.print("Front Left IR =  ");
Serial.print(trinbot.flDistSens.distance());
 Serial.print("Front right IR = ");
Serial.print(trinbot.frDistSens.distance());
  Serial.print("Rear Left IR = ");
Serial.print(trinbot.rlDistSens.distance());
 Serial.print("Front Left right = ");
Serial.print(trinbot.rrDistSens.distance());
 Serial.print("Front dist IR = ");
Serial.println(trinbot.frontDistSens.distance());
*/
//  Serial.print(trinbot.left_odom);
//  Serial.print(" ");
//  Serial.print(trinbot.right_odom);
//  Serial.println(" ");
  
  switch (state) {
    // *******************************
    case HallwayNavigation:
      //By Default, follow right wall
      trinbot.followWall(Right);

      //If a door indicator is detected, iterate counter
      if (leftLine < trinbot.leftLineThresh || rightLine < trinbot.rightLineThresh) { counter++; } 
      else { counter = 0; }
      
      //If a door indicator is detected for five intervals, back up and switch to AlignToLine state
      if (counter > 5) { 
        trinbot.left_motor.write(-150);
        trinbot.right_motor.write(-150);
        delay(long_delay);
        start_time = millis();
        start_dOdom = trinbot.left_odom-trinbot.right_odom;
        state = AlignToLine;
        counter = 0; 
      }
      break;
   // *******************************
   
   // *******************************
    case HallwayNavigation2:
//      trinbot.followWallSkipGaps(Left);
      trinbot.followWall(Right);
      
      if (leftLine < trinbot.leftLineThresh || rightLine < trinbot.rightLineThresh) { counter++; } 
      else { counter = 0; }
      
      if (counter > 5) { 
        trinbot.left_motor.write(-150);
        trinbot.right_motor.write(-150);
        delay(long_delay);
        start_time = millis();
        start_dOdom = trinbot.left_odom-trinbot.right_odom;
        state = AlignToLine;
        counter = 0; 
      }
      break;
   // *******************************
   
   // *******************************
   case AlignToLine:
      /* Attempt to allign with line towards room:
         - Adjust to line, if line is passed or aligned with, switch to checkroom state. */
      if (abs(trinbot.left_pwm) > 40 || abs(trinbot.right_pwm) > 40) { counter = 0; } 
      else { counter++; }
//      Serial.println(abs(start_dOdom - (trinbot.left_odom-trinbot.right_odom)));
      if (counter < 100 && (millis()-start_time) < 1750 && abs(start_dOdom - (trinbot.left_odom-trinbot.right_odom)) < 150) {
//      if (counter < 100 && (millis()-start_time) < 1750) {
//      if (counter < 100) {
        trinbot.left_pwm = (int(leftLine)-trinbot.leftLineThresh)/7;
        trinbot.right_pwm = (int(rightLine)-trinbot.rightLineThresh)/7;
//        trinbot.left_pwm = constrain(trinbot.left_pwm, -255,150);
//        trinbot.right_pwm = constrain(trinbot.right_pwm, -255,150);
      } else {
        state = CheckRoom;
        counter = 0;
      }
      break;
   // *******************************
   
   // *******************************
   case AlignToCircle:
      // Attmpt to allign with flame platform circle.
      if (trinbot.left_pwm > 40 || trinbot.right_pwm > 40) { counter = 0; } 
      else { counter++; }
      
      //If failed to align with circle, adjust back, otherwise switch state to FinalFlameSweep 
      if (counter < 100 && (millis()-start_time) < 2000 && abs(start_dOdom - (trinbot.left_odom-trinbot.right_odom)) < 150) {
//      if (counter < 100 && (millis()-start_time) < 1750) {
//      if (counter < 100) {
        trinbot.left_pwm = (int(leftLine)-trinbot.leftLineThresh)/7;
        trinbot.right_pwm = (int(rightLine)-trinbot.rightLineThresh)/7;
//        trinbot.left_pwm = constrain(trinbot.left_pwm, -255,150);
//        trinbot.right_pwm = constrain(trinbot.right_pwm, -255,150);
      } else {
        state = FinalFlameSweep;
        counter = 0;
      }
      break;
   // *******************************
    
    // *******************************
    case FinalFlameSweep:
      //Stop the bot
      trinbot.left_motor.write(0);
      trinbot.right_motor.write(0);

      //Sweep for flame again
      trinbot.sweepForFlame();

      //If flame found, switch to extinguish state, otherwise, backup and switch back to RoomNavigation state.
      if (trinbot.maxFlameReading > trinbot.flameThresh) {
        //if (abs(90-a) < 30) {
          state = Extinguish;
        //}
      } else {
        trinbot.left_motor.write(-255);
        trinbot.right_motor.write(-255);
        delay(500);
        state = RoomNavigation;
      }
      break;
      // *******************************
      
   // *******************************
    case Extinguish:
      //Stop the bot
      trinbot.left_motor.write(0);
      trinbot.right_motor.write(0);

      //Attempt to Extinguish, then search again for flames
      trinbot.extinguish();
      trinbot.sweepForFlame();

      //If flame successfully extinguished, back up, switch state to ReturnHome
      if (trinbot.maxFlameReading < trinbot.flameThresh) {
        trinbot.left_motor.write(-255);
        trinbot.right_motor.write(-255);
        delay(short_delay);
        trinbot.left_motor.write(255);
        trinbot.right_motor.write(-255);
        delay(long_delay);
        state = ReturnHome; // return home <----- this needs to be implemented
      } else {
        state = Extinguish; //** THIS MIGHT BE REDUNDANT AND POTENTIALLY CAN BE REMOVED (we are already in state Extinguish)
      }
      break;
      // *******************************
    
    // *******************************
    case CheckRoom:
      //Stop the bot
      trinbot.left_motor.write(0);
      trinbot.right_motor.write(0);
      Serial.println("Checking Room...");

      //Discover the angle of the flame (if its in the room) otherwise default to _____
      flameAngle = trinbot.sweepForFlame();
      Serial.println(trinbot.maxFlameReading);
      Serial.println(trinbot.maxFlameDir);

      //Setup wall to follow based off of the what diretion the flame is in 
      if (flameAngle > 90) {
        wall_to_follow_in_room = Right;
      } else {
        wall_to_follow_in_room = Left;
      }

      /*If flame detected, begin moving, inform user, and set state to RoomNavigation
        Otherwise back out of room, wait, back turn to realign with wall, inform user, 
      and switch to HallwayNavigation (1 or 2??) */
      if (trinbot.maxFlameReading > trinbot.flameThresh) {
        trinbot.left_motor.write(255);
        trinbot.right_motor.write(255);
        delay(long_delay);
        Serial.println("There's a flame!");
        state = RoomNavigation;
        counter = 0;
      } else {
        trinbot.left_motor.write(-255);
        trinbot.right_motor.write(-255);
        delay(long_delay);
        trinbot.left_motor.write(-255);
        trinbot.right_motor.write(255);
        delay(long_delay);
        Serial.println("No flame.");
        state = HallwayNavigation2; // TEMPORARY <-------- CHANGE THIS FOR THE REAL DEAL
      }
      break;
      // *******************************
      
      // *******************************
      case RoomNavigation: //If fire is found
        trinbot.followWall(wall_to_follow_in_room);
        
        //If flame platform detected, iterate counter
        if (leftLine > FLAME_PLATFORM || rightLine > FLAME_PLATFORM) { counter++; } 
        else { counter = 0; }
        
        /* If flame platform detected for five intervals, back up, setupfor alignment with platform,
           and switch state to AlignToCircle */
        if (counter > 5) {
          trinbot.left_motor.write(-255);
          trinbot.right_motor.write(-255);
          delay(short_delay);
          start_time = millis();
          start_dOdom = trinbot.left_odom-trinbot.right_odom;
          state = AlignToCircle;
          counter = 0;
        }
        
        break;
      // *******************************
      
      // *******************************
      case ReturnHome: //THIS STILL NEEDS TO BE IMPLEMENTED 
        trinbot.frontServo.write(45); // helps avoid head on collisions with the small wall
        trinbot.followWall(Left);
        
        if (leftLine > 500 || rightLine > 500) { counter++; } 
        else { counter = 0; }
        
        if (counter > 250) { // line sensors have detected the white circle (250 times)
          trinbot.left_motor.write(0);
          trinbot.right_motor.write(0);
          while(1) { } // We win
        }
        
        break;
      // *******************************
  }
  
//  Serial.print(trinbot.left_pwm);
//  Serial.print(" ");
//  Serial.println(trinbot.right_pwm);
  trinbot.left_motor.write(trinbot.left_pwm);
  trinbot.right_motor.write(trinbot.right_pwm);
}

// UPDATE CURRENT STATE
// This function updates the current velocity and heading as well as the desired heading and velocity
void update_current_state() {
  trinbot.update();
}

void update_waypoint() {
  if (trin_waypoints[0].x == 0 && trin_waypoints[0].y == 0) {
    // we've reached the end of the waypoint queue... STOP
    //desired_velocity = 0;
    //trinbot.k_velocity = 0;
    //trinbot.k_heading = 0;
    // stop the program ?
    // add new waypoints ?
    trinbot.set_desired_velocity(0);
  } 
  else {
    //desired_velocity = 60; //[cm/s]
    float dist = pow(pow(trin_waypoints[0].x-trinbot.xPos,2)+pow(trin_waypoints[0].y-trinbot.yPos,2),0.5);
    if (dist < 5) {
      remove_waypoint();
    } 
    else {
      trinbot.set_desired_velocity(30);
      float h = 180.0*atan2(trin_waypoints[0].y-trinbot.yPos,trin_waypoints[0].x-trinbot.xPos)/PI;
      trinbot.set_desired_heading(h);
    }
  }
}

void initialize_waypoints() {
  for (int i = 0; i < (sizeof(trin_waypoints)/sizeof(waypoint)); i++) {
    trin_waypoints[i] = (waypoint){0, 0};
  }
}

void add_waypoint(float x, float y) {
  trin_waypoints[num_waypoints] = (waypoint){ x, y  };
  num_waypoints++;
}

void remove_waypoint() {
  for (int i = 0; i < (sizeof(trin_waypoints)/sizeof(waypoint))-1; i++) {
    trin_waypoints[i].x = trin_waypoints[i+1].x;
    trin_waypoints[i].y = trin_waypoints[i+1].y;
  }
  num_waypoints--;
}

void read_quad_right() { // right encoder
  if (digitalRead(trinbot.enc_right_A) == digitalRead(trinbot.enc_right_B)) {
    trinbot.right_odom++;
  } 
  else {
    trinbot.right_odom--;
  }
}

void read_quad_left() { // left encoder
  if (digitalRead(trinbot.enc_left_A) == digitalRead(trinbot.enc_left_B)) {
    trinbot.left_odom++;
  } 
  else {
    trinbot.left_odom--;
  }
}
