#include <Servo.h> 

#include <ros.h>
#include <geometry_msgs/Point.h>

ros::NodeHandle nh;
geometry_msgs::Point p;

void moveEyes(const geometry_msgs::Point& p);
ros::Subscriber<geometry_msgs::Point> eyesSub("eyesDirection", &moveEyes);


Servo leftEyeHor, leftEyeVert, rightEyeHor, rightEyeVert;


void setup() 
{ 
  leftEyeHor.attach(9);
  leftEyeVert.attach(8);
  rightEyeHor.attach(7);
  rightEyeVert.attach(4);
  faceServosZeroPos();
  
  nh.initNode();
  nh.subscribe(eyesSub);
  //nh.advertise(headRespPub);
  
  Serial.begin(57600);

} 


float curr_theta = 0.0, curr_phi = 0.0;
void saveCurrPos(float &theta, float &phi) {
  curr_theta = theta;
  curr_phi = phi;
};

void eyeRangeCheck(float &theta, float &phi) {
  if(theta>20) theta=20;
  if(theta<-20) theta=-20;
  if(phi>20) phi=20;
  if(phi<-20) phi=-20;
};

float speed_coeff = 0.3, max_speed = 0.5;
void adjustSpeed(float &theta, float &phi) {
  phi = curr_phi + speed_coeff*(phi-curr_phi);
  theta = curr_theta + speed_coeff*(theta-curr_theta);
};

float vertEyeCoeff = 1.544, horEyeCoeff = 1.852;
void leftEyeLookAtPoint(float theta, float phi) {
  leftEyeVert.write(90.0+vertEyeCoeff*theta);
  leftEyeHor.write(90.0+horEyeCoeff*phi);
};

void rightEyeLookAtPoint(float theta, float phi) {
  eyeRangeCheck(theta, phi);
  rightEyeVert.write(90.0-vertEyeCoeff*theta);
  rightEyeHor.write(90.0+horEyeCoeff*phi);
};

void eyesLookAtPoint(float &theta, float &phi) {
  eyeRangeCheck(theta, phi);
  adjustSpeed(theta, phi);
  saveCurrPos(theta, phi);
  rightEyeLookAtPoint(theta, phi);
  leftEyeLookAtPoint(theta, phi);
};

void moveEyes(const geometry_msgs::Point& p) {
  float phi = p.x; // polar angle phi
  float theta = 90.0-p.y; // polar angle theta
  eyesLookAtPoint(theta, phi);
};

void servoTest(Servo myservo) {
  myservo.write(90);  // set servo to mid-point
  delay(300);
  myservo.write(130);  // set servo to up
  delay(300);
  myservo.write(90);  // set servo to mid-point
  delay(300);
  myservo.write(50);  // set servo to down
  delay(300);
  myservo.write(90);  // set servo to mid-point
  delay(300);
};

void faceServosTest(void) {
  servoTest(leftEyeHor);
  servoTest(leftEyeVert);
  servoTest(rightEyeHor);
  servoTest(rightEyeVert);
};


void faceServosTest2(void) {
  leftEyeLookAtPoint(0,0);
  rightEyeLookAtPoint(0,0);
  delay(1000);
  leftEyeLookAtPoint(20,20);
  rightEyeLookAtPoint(20,20);
  delay(1000);
  leftEyeLookAtPoint(-20,-20);
  rightEyeLookAtPoint(-20,-20);
  delay(1000);
  leftEyeLookAtPoint(20,0);
  rightEyeLookAtPoint(20,0);
  delay(1000);
};

void leftEyeHorRange(void) {
  leftEyeHor.write(90);
  delay(2000);
  leftEyeHor.write(100);
  delay(2000);
  leftEyeHor.write(110);
  delay(2000);
  leftEyeHor.write(120);
  delay(2000);
  leftEyeHor.write(110);
  delay(2000);
  leftEyeHor.write(100);
  delay(2000);
  leftEyeHor.write(90);
  delay(2000);
};


void leftEyeVertRange(void) {
  leftEyeVert.write(90);
  delay(2000);
  leftEyeVert.write(100);
  delay(2000);
  leftEyeVert.write(110);
  delay(2000);
  leftEyeVert.write(120);
  delay(2000);
  leftEyeVert.write(110);
  delay(2000);
  leftEyeVert.write(100);
  delay(2000);
  leftEyeVert.write(90);
  delay(2000);
  leftEyeVert.write(80);
  delay(2000);
  leftEyeVert.write(70);
  delay(2000);
  leftEyeVert.write(60);
  delay(2000);
  leftEyeVert.write(70);
  delay(2000);
  leftEyeVert.write(80);
  delay(2000);
  leftEyeVert.write(90);
  delay(2000);
};



void faceServosZeroPos(void) {
  leftEyeHor.write(90);
  rightEyeHor.write(90);
  leftEyeVert.write(90);
  rightEyeVert.write(90);  
  delay(100);
};

void eyesLookAtTest(void) {
  float theta = 0.0, phi = 0.0;
  eyesLookAtPoint(theta, phi);
  delay(500);

  theta = 10; phi = 10;
  eyesLookAtPoint(theta, phi);
  delay(500);

  theta = 0; phi = 20;
  eyesLookAtPoint(theta, phi);
  delay(500);
  
  theta = -10; phi = -10;
  eyesLookAtPoint(theta, phi);
  delay(500);
};  
  
  


void loop() {
  //faceServosTest2();
  //leftEyeVertRange();
  nh.spinOnce();
  delay(10);
    
  //eyesLookAtTest();
} 
