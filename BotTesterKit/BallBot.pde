import processing.serial.*;
Serial myPort;

void setup(){
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 9600);
  
}

void draw(){
}

void keyPressed(){
 myPort.write(key); 
 println(key);
}

void keyReleased(){
 myPort.write(char(key -32));
 println(char(key - 32)); 
}
