#include <limits.h>

#include <RedBot.h>
#include <RedBotSoftwareSerial.h>
#include <PID_v1.h>
#include <NewPing.h>

#define MAX_SPEED 200
#define FOLLOW_SPEED 80

#define PIN_BACK_TRIG  A2
#define PIN_BACK_ECHO  A3
#define PIN_RIGHT_TRIG  A4
#define PIN_RIGHT_ECHO  A5
#define PIN_FRONT_TRIG  A0
#define PIN_FRONT_ECHO  A1

// Distance from the wall
#define SETPOINT 5
#define MIN_DISTANCE 3
#define MAX_DISTANCE 200

// Sonar objects
NewPing backSonar(PIN_BACK_TRIG, PIN_BACK_ECHO, MAX_DISTANCE);
NewPing rightSonar(PIN_RIGHT_TRIG, PIN_RIGHT_ECHO, MAX_DISTANCE);
NewPing frontSonar(PIN_FRONT_TRIG,PIN_FRONT_ECHO,MAX_DISTANCE);
// Redbot motor(s) object
RedBotMotors motors;

// PID object and parameters
double Setpoint = SETPOINT, pidOut, pidIn;
double Kp=0.012, Ki=0, Kd=0.01;
PID pid(&pidIn, &pidOut, &Setpoint, Kp, Ki, Kd, DIRECT);

// Takes doubles for left and right as percentage 0.0 - 1.0
void setMotors(double left, double right, int motorSpeed) {
  if (left * motorSpeed > 255) {
    left = 255.0/motorSpeed;
  }
  if (right * motorSpeed > 255) {
    right = 255.0/motorSpeed;
  }
  motors.rightMotor(0 - int(motorSpeed * right));
  motors.leftMotor(0 - int(motorSpeed * left));
}

bool compute = false;
int rightDist = 0;

void followBox(int rightDist){
  
  compute = false;
  while (compute == false) {
    //rightDist = rightSonar.ping_cm();
    if (rightDist == 0){
      rightDist = MAX_DISTANCE;
    }
    if (rightDist > SETPOINT*2) {;
      pid.SetOutputLimits(-0.65,0.65);
    }
    else {
      pid.SetOutputLimits(-0.55,0.55);
    }
    pidIn = rightDist;
    compute = pid.Compute();
  }

  setMotors(1-pidOut, 1+pidOut,FOLLOW_SPEED); 
}


#define BACK_DISTANCE 7
#define PULL_LEFT 0.51
#define PULL_RIGHT 0.22
/*
int pullBack(){
  setMotors(-PULL_LEFT,-PULL_RIGHT, MAX_SPEED);
  int current = backRightSonar.ping_cm();
  int count = 0;
  while(current > BACK_DISTANCE || current == 0){
    delay(75);
    current = backRightSonar.ping_cm();
    count++;
  }
  delay(400);
  setMotors(0,0,0); 
  return count;
}*/

#define FRONT_DISTANCE 7
#define RIGHT_DISTANCE 9
#define ROTATE_SPEED_L .55
#define ROTATE_SPEED_R .5
void rotateRight(){
  setMotors(ROTATE_SPEED_L,-ROTATE_SPEED_R,MAX_SPEED);
  int currentFront = frontSonar.ping_cm();
  int currentRight = frontSonar.ping_cm();
  while(currentFront > FRONT_DISTANCE || currentFront == 0 || currentRight > RIGHT_DISTANCE){
    delay(75);
    currentFront = frontSonar.ping_cm();
    currentRight = frontSonar.ping_cm();
  }
  //delay(200);
  setMotors(0,0,0);
}

#define ROTATE_SPEED .3
void rotateLeft() {
  setMotors(-ROTATE_SPEED_L,ROTATE_SPEED_R,MAX_SPEED);
  delay(500);
  int rightCur = INT_MAX-1, rightLast = INT_MAX;
  while(rightCur < rightLast) {
    rightLast = rightCur;
    rightCur = rightSonar.ping_cm();
    if (rightCur == 0) { rightCur = MAX_DISTANCE; }
    delay(50);
  }
  setMotors(0,0,0);
}

const int NO_BOX_DIST = 20;
void parallelPark(){
  setMotors(.4,.4,MAX_SPEED);
  
  while (rightSonar.ping_cm() > NO_BOX_DIST)
    delay(50);
  int right = rightSonar.ping_cm();
  unsigned long time = millis(); 
  while (right <= NO_BOX_DIST || millis() - time < 1500){
    Serial.print("Follow Box\n");
    followBox(right);
    delay(50);
    right = rightSonar.ping_cm();
  }
  setMotors(.4,.4,MAX_SPEED);
  while (rightSonar.ping_cm() > NO_BOX_DIST)
    delay(50);
  // We are at the second box now
  
  delay(150);
  setMotors(0,0,0);
  
  backIn();
  rotateRight();
  delay(2000);
  rotateLeft();
  
  
  right = rightSonar.ping_cm();
  time = millis(); 
  while (right <= NO_BOX_DIST || millis() - time < 2500){
    Serial.print("Follow Box\n");
    followBox(right);
    delay(50);
    right = rightSonar.ping_cm();
  }
  
  
  setMotors(.4,.42,MAX_SPEED);
}


void backIn() {
  
  // Turn 90 deg. on a slight radius into the parking space
  // The right wheel should be centered on the edge of the box
  setMotors(-0.73,-0.15, 200);
  delay(930);
  int rightCur = INT_MAX-1, rightLast=INT_MAX;
  while (rightCur < rightLast) {
    rightLast = rightCur;
    rightCur = rightSonar.ping_cm();
    delay(30);
  }
  
  // Back up until we're within an inch of the wall
  setMotors(-.4, -.4, MAX_SPEED);
  int back = INT_MAX;
  while (back > 5) {
    back = backSonar.ping_cm();
    Serial.println(back);
    if (back == 0) { back = 200; }
    delay(60);
  }
  
  setMotors(0, 0, 0);
}


void setup(){
  Serial.begin(9600);
  Serial.print("Setup Started.\n");
  //turn the PID on  
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-0.55,0.55);
  pid.SetSampleTime(50);
  Serial.print("Setup Complete.\n");

  parallelPark(); 
}


void loop() {/*
  delay(2000);
  backIn();
  int ct = rotateRight();
  delay(1000);
  rotateLeft(ct);
  /*
  delay(30);
  int back = backSonar.ping_cm();
  delay(30);
  Serial.print(back);
  Serial.print("   ");
  Serial.print(frontSonar.ping_cm());
  delay(30);
  Serial.print("   ");
  Serial.println(rightSonar.ping_cm());*/

}

