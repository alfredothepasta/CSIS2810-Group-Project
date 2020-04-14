import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;
float yaw = 0;
float pitch = 0;
float roll = 0;

Serial input;

String data;

void setup() {
  size(1280, 1024, P3D);
  input = new Serial(this, "COM3", 115200);
  
  //throw out the first reading
  
  // split our input on newline
  input.bufferUntil('\n');
}

void draw() {
  translate(width/2, height/2, 0);
  background(150);
  textSize(22);
  textSize(40);
  text(int(yaw) + " " + int(pitch) + " " + int(roll) + "\n", -100, 265);
  
  
  // Object Rotation
  rotateX(radians(pitch));
  rotateY(radians(-yaw));
  rotateZ(radians(-roll));
  
  // Make the box
  stroke(255);
  fill(0);
  box(400, 50, 200);
  
  
}

void serialEvent(Serial input){
  data = input.readStringUntil('\n');
  
 
  
  if(data != null){
    data = trim(data);
    String ypr[] = split(data, '/');
    
    if(ypr.length > 1){
      yaw = float(ypr[0]);
      pitch = float(ypr[1]);
      roll = float(ypr[2]);
    }
  }
}
