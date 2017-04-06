#include <RedBot.h>
#include <RedBotSoftwareSerial.h>
#include <PID_v1.h>
#include <NewPing.h>

#define MAX_SPEED 200
#define FOLLOW_SPEED 80

#define PIN_BACK_TRIG  A4
#define PIN_BACK_ECHO  A5
#define PIN_RIGHT_TRIG  A2
#define PIN_RIGHT_ECHO  A3
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
double Kp=0.01, Ki=0, Kd=0.01;
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
      pid.SetOutputLimits(-0.35,0.35);
    }
    else {
      pid.SetOutputLimits(-0.55,0.55);
    }
    pidIn = rightDist;
    compute = pid.Compute();
  }

  setMotors(1-pidOut, 1+pidOut,FOLLOW_SPEED); 
}
/*void turnLeft() {
  // Turn until the front is 'clear'
  int frontDist = 10;
  while ((frontDist < 20) && (frontDist != 0)) {
    setMotors(-0.8, 0.8);
    delay(50);
    frontDist = backSonar.ping_cm();
  }

  delay(100);

  
  // Turn until the right side stops decreasing
  int current = 199;
  int previous = MAX_DISTANCE;
  while (current <= previous) {
    setMotors(-1, 1);
    previous = current;
    current = rightSonar.ping_cm();
    delay(50);
  }
}*/
#define BACK_DISTANCE 7
#define PULL_LEFT 0.51
#define PULL_RIGHT 0.22
int pullBack(){
  setMotors(-PULL_LEFT,-PULL_RIGHT, MAX_SPEED);
  int current = backSonar.ping_cm();
  int count = 0;
  while(current > BACK_DISTANCE || current == 0){
    delay(75);
    current = backSonar.ping_cm();
    count++;
  }
  delay(400);
  setMotors(0,0,0); 
  return count;
}

#define FRONT_DISTANCE 9
#define ROTATE_SPEED_L .35
#define ROTATE_SPEED_R .3
int rotateRight(){
  setMotors(ROTATE_SPEED_L,-ROTATE_SPEED_R,MAX_SPEED);
  int current = frontSonar.ping_cm();
  int count = 0;
  while(current > FRONT_DISTANCE || current == 0){
    delay(75);
    current = frontSonar.ping_cm();
    count++;
  }
  delay(350);
  setMotors(0,0,0);
  return count;
}

#define ROTATE_SPEED .3
void rotateLeft(int count){
  setMotors(-ROTATE_SPEED_L,ROTATE_SPEED_R,MAX_SPEED);
  while(count > 0){
    delay(75);
    count--;
  }
  delay(300);
  setMotors(0,0,0);
}

void pullOut(int count){
  setMotors(PULL_LEFT,PULL_RIGHT,MAX_SPEED);
  while(count > 0){
    delay(75);
    count--;
  }
  delay(400);
  setMotors(0,0,0);
}


const int NO_BOX_DIST = 20;
void parallelPark(){
  setMotors(.41,.4,MAX_SPEED);
  
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
  delay(500);
  setMotors(0,0,0);
  
  int pullCt = pullBack();
  int rotCt = rotateRight();
  delay(2000);
  rotateLeft(rotCt);
  pullOut(pullCt);
  setMotors(.4,.42,MAX_SPEED);
}

void setup(){
  Serial.begin(9600);
  Serial.print("Setup Started.\n");
  
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-0.55,0.55);
  pid.SetSampleTime(50);
  Serial.print("Setup Complete.\n");

  parallelPark(); 
  
  //turn the PID on


}


//, frontDist = 0;

void loop() {
  //Serial.print(rightSonar.ping_cm());
  //Serial.print(frontSonar.ping_cm());
  //Serial.print(" cm. \n");
  //delay(100);
  
  /*delay(5000);
  int pullCt = pullBack();
  int rotCt = rotateRight();
  delay(2000);
  rotateLeft(rotCt);
  pullOut(pullCt);*/
//}

  //frontDist = backSonar.ping_cm();
  //Serial.print(frontDist);
  //Serial.print('\n');
  //if ((frontDist < 20) && (frontDist != 0)) {
  //  turnLeft();
  //}
  //delay(20);


}

