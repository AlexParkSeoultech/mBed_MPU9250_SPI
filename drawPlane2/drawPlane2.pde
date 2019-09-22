import processing.serial.*;
Serial myPort;
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;
color canopy=color(0,255,255);
color body=color(255,255,255);
color front=color(125,255,125);
color back=color(255,125,125);
color spinner =color(180,0,0);
PShape plane;

void setup() {
  size(800, 500, P3D);
  print("Starting///");
  String[] coms=Serial.list();
  print(coms.length);
  myPort = new Serial(this, Serial.list()[coms.length-1], 115200); // if you have only ONE serial port active
  // if you know the serial port name
  //myPort = new Serial(this, "COM5:", 115200);                    // Windows
  //myPort = new Serial(this, "/dev/ttyACM0", 115200);             // Linux
  //myPort = new Serial(this, "/dev/cu.usbmodem1217321", 115200);  // Mac
  PFont f = createFont("consolas",20,true);
  textFont(f); // set text size
  textMode(SHAPE); // set text mode to shape
  smooth();
  plane = loadShape("Plane2.obj");
}

void draw(){
  serialEvent();  // read and parse incoming serial message
  background(0,0,60); // set background to dark blue
  fill(body);
  String s = "rpy=( " +nfp(roll,3,2)+", "+ nfp(pitch,3,2)+", "+ nfp(yaw,3,2)+" )";
  text(s, 10, 22); 
  lights();

  translate(width/2, height/2); // set position to centre
  pushMatrix(); // begin object

  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(pitch));
  float s2 = sin(radians(pitch));
  float c3 = cos(radians(yaw));
  float s3 = sin(radians(yaw));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);
  scale(8.0);  //650
  rotateX(PI/2);
  rotateZ(PI/2);
  shape(plane);
  popMatrix(); // end of object
}

void serialEvent()
{
  int newLine = 10; // new line character in ASCII
  String message;
  do {
    message = myPort.readStringUntil(newLine); // read from port until new line
    print(message);
    if (message != null) {
      String[] list = split(trim(message), ",");
      if (list.length >= 4 && list[0].equals("YPR")) {
        yaw =float(list[1]); // convert to float yaw
        pitch = float(list[2]); // convert to float pitch
        roll = float(list[3]); // convert to float roll
      }
    }
  } while (message != null);
}
