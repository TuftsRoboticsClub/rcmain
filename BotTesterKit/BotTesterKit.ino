
#include <Servo.h> 

Servo contServo;  // create servo object to control a servo 
Servo flipperServo;  // create servo object to control a servo 

//int potpin = 0;  // analog pin used to connect the potentiometer
int contservopin = 11;
int flipperservopin = 3;
int flipperAngle1 = 0;
int flipperAngle2 = 100;
//int left1 = 5, left2 = 4 , right1 = 7 , right2 = 6;
int left1 = 4, left2 = 5 , right1 = 0 , right2 = 0;
char val = ' ';
boolean keys[255];
int FORE  = 'w',
    BACK  = 's',
    LEFT  = 'a',
    RIGHT = 'd',
    HALT  = 'x',
    SERVO_FORE   = 'j',
    SERVO_BACK  = 'k',
    FLIPPER = 'l';
boolean fore = false, back = false, left = false, right = false;
boolean servoF = false, servoB = false;
int SPEED = 255;
int SERVOMID = 90;
int servoVal = SERVOMID;
int lspeed = 0;
int rspeed = 0;

void setup() 
{ 
  for(int i = 0; i < 255; i ++){
    keys[i] = false; 
  }

  contServo.attach(contservopin);  // attaches the servo on pin 9 to the servo object
  flipperServo.attach(flipperservopin);
  
  Serial.begin(9600); 

  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);

} 

void getDirections(); //Apply bools via read
void applyDirectionSpeed();
void applyDirectionToMotors();
void applyDirectionToServo();

void loop() 
{ 
  getDirections();
  applyDirectionSpeed();
  applyDirectionToMotors();
  //applyDirectionToServo();
 /* analogWrite(left1, 250);
  analogWrite(right1, 250);
  digitalWrite(left2, LOW);
  digitalWrite(right2, LOW);
  */
  contServo.write(servoVal);
  /*if(keys[FLIPPER]){
    flipperServo.write(flipperAngle1);
  }else{
    flipperServo.write(flipperAngle2); 
  }
  */
 /*flipperServo.write(0); 
 delay(200);
  flipperServo.write(100); 
  delay(200);
  */

}

void getDirections(){
  if(Serial.available()){
    val = Serial.read();
    if(true){
      Serial.println(val); 
    }
    
    if(val != FLIPPER){
      //Universal keys bool array
      //Set pressed keys to true
      keys[val] = true;
      //Set released keys (represented by capticals thus +32) to false
      keys[val+32] = false;
    
      if(keys[HALT]){
        keys[FORE]  = false;
        keys[BACK]  = false; 
        keys[LEFT]  = false;
        keys[RIGHT] = false;
      }
    }else{
      keys[FLIPPER] = ! keys[FLIPPER];
    }
  }  
}

void applyDirectionSpeed(){
  lspeed = 0;
  rspeed = 0;
  //Motors
  if(keys[FORE]){
    lspeed += SPEED;
    rspeed += SPEED;
    //if(left){
    // rspeed
    //}
    if(keys[LEFT]){
      //lspeed += SPEED /2; 
      lspeed -= SPEED/2;
    }
    if(keys[RIGHT]){
     // rspeed += SPEED /2; 
      rspeed -= SPEED/2;
    }
  }
  if(keys[BACK]){
    lspeed -= SPEED;
    rspeed -= SPEED;
    if(keys[LEFT]){
      rspeed += SPEED/2; 
      //rspeed = SPEED;
    }
    if(keys[RIGHT]){
      lspeed += SPEED /2; 
      //lspeed -= SPEED;
    }
  }
  if(!(keys[FORE]) && !(keys[BACK])){
    if(keys[LEFT]){
      rspeed += SPEED/2; 
      lspeed -= SPEED/2;
    }
    if(keys[RIGHT]){
      lspeed += SPEED/2; 
      rspeed -= SPEED/2;
    }
  }
  
  //Servo
  servoVal = SERVOMID;
  if(keys[SERVO_FORE]){
    servoVal -= 90;
  }
  if(keys[SERVO_BACK]){
    servoVal += 90;
  } 
 
  //Flipper 
  
}


void applyDirectionToMotors(){

  if(lspeed >= 0){
    analogWrite(left1, lspeed);
    digitalWrite(left2, LOW);
  } 
  else{
    analogWrite(left1, 255 - abs(lspeed));
    digitalWrite(left2, HIGH);
  }

  if(rspeed >= 0){
    analogWrite(right1, rspeed);
    digitalWrite(right2, LOW);
  } 
  else{
    analogWrite(right1, 255 - abs(rspeed));
    digitalWrite(right2, HIGH);
  }

}



