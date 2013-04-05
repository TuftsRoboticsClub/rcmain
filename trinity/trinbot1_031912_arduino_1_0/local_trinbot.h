
#ifndef Trinbot_H
#define Trinbot_H

#include "local_DifferentialDrive.h"
#include "Servo.h"
#include "low_pass_filter.h"
#include "QTRSensors.h"

struct Box {
  int x_coord;
  int y_coord;
  int visited;
  String name;
};

//For wall following
class IRSensor
{
public:
    IRSensor();
    void attach(byte pin0);
    float distance();

private:
    byte pin;
};

//For candle detection
//(Different type of IR)
class FlameSensor
{
public:
    FlameSensor();
    void attach(byte pin0);
    float read_filtered();

private:
    byte pin;
    LPFilter lpfilter;
};

enum WallSide { Left = 0, Right = 1 };

class Trinbot : public DifferentialDrive
{
public:
    Trinbot();
    void init();
    void update();
    void sweepServo(int start_angle, int end_angle);
    void checkLineSensors();
    int sweepForFlame();
    void detectWalls();
    void followWall(WallSide side_to_follow = Right);
    void followWallSkipGaps(WallSide side_to_follow = Right);
    int getNearestLocation();
    void extinguish();
    Box locations[5];
    
    byte enc_right_A, enc_left_A, enc_right_B, enc_left_B;
    long left_odom, right_odom, last_left_odom, last_right_odom, last_time;
    FlameSensor flameSens;
    Servo frontServo, co2Servo;
    IRSensor flDistSens, rlDistSens, frDistSens, rrDistSens, frontDistSens;
    float xPos, yPos; //[cm]
    QTRSensorsRC lineSensLeft; 
    QTRSensorsRC lineSensRight; 
    boolean line_left, line_right;
    boolean fr_wall, fl_wall, rr_wall, rl_wall, front_wall;
    float maxFlameReading;
    int maxFlameDir;
    float flameThresh;
    int leftLineThresh, rightLineThresh;
    
private:   
    // Hardware Pinout Connections:
    float WHEEL_SEPARATION, WHEEL_RADIUS, TICKS_PER_CM, TICKS_PER_DEG;
    int line_thresh;
    float wall_follow_dist, front_obs_dist;
    int co2Servo_up, co2Servo_down;

};

// Debug routine, emulates printf, limited to 127 chars.
void debug(char *fmt, ...);

#endif

