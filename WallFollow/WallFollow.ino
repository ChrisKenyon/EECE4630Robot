#include <RedBot.h>
#include <RedBotSoftwareSerial.h>
#include <PID_v1.h>
#include <NewPing.h>

#define MAX_SPEED 150

#define FRONT_TRIGGER_PIN  A4
#define FRONT_ECHO_PIN     A5
#define RIGHT_TRIGGER_PIN  A2
#define RIGHT_ECHO_PIN     A3
#define MAX_DISTANCE 200
#define TOO_CLOSE_DIST 13
#define SET_DISTANCE 15

NewPing frontSonar(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing rightSonar(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
int frontDist, rightDist;
RedBotMotors motors; // Instantiate the motor control object.
/*
//Define Variables we'll be connecting to
double Setpoint, Output, pidIn;
//Specify the links and initial tuning parameters
double Kp=0.001, Ki=0, Kd=0.0001;
PID pid(&pidIn, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
*/
void setup(){
  Serial.begin(9600);
  Serial.print("Setup Started.\n");
  /*
  Setpoint = 10;
  //turn the PID on
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-1,1);
  pid.SetSampleTime(50);*/
  //pidIn = rightSonar.ping_cm();
  Serial.print("Setup Complete.\n");
}
/*
void setMotors(double pidOut){
  if (pidOut > 0) {
    motors.rightMotor(MAX_SPEED);
    motors.leftMotor(MAX_SPEED * (1 - pidOut));
  }
  else {
    motors.rightMotor(MAX_SPEED * (1 + pidOut));
    motors.leftMotor(MAX_SPEED);
  }
}
*/

// Takes doubles for left and right as percentage 0.0 - 1.0
void setMotors(double left, double right){
  motors.rightMotor(0 - int(MAX_SPEED * right));
  motors.leftMotor(0 - int(MAX_SPEED * left));
}

bool wRight, wFront, tooClose, lostRightWall, lostFrontWall;
int currentError, previousError, changeInError; 

void loop(){
  //delay(100);
  frontDist = frontSonar.ping_cm();
  rightDist = rightSonar.ping_cm();
  lostRightWall = rightDist == 0;
  lostFrontWall = frontDist == 0;
  wRight = rightDist <= SET_DISTANCE && !lostRightWall;
  wFront = frontDist <= SET_DISTANCE && !lostFrontWall;
  tooClose = (rightDist < TOO_CLOSE_DIST && !lostRightWall) ||
             (frontDist < SET_DISTANCE && !lostFrontWall);
  
  Serial.print("Right: ");
  Serial.print(rightDist);
  Serial.print("\nFront: ");
  Serial.print(frontDist);
  Serial.print("\n");
  

  /*Serial.print("Previous Error");  
  Serial.print(previousError);
  Serial.print("\n");
  */
  if(wRight && !wFront)
  {
    if (tooClose){    
      Serial.print("Turning left\n");
      setMotors(0.5,0.9);
      currentError = SET_DISTANCE - rightDist;
      changeInError =  currentError - previousError;
    }
    else {       
      Serial.print("Moving forward\n");
      setMotors(1.0,1.0);
      currentError = SET_DISTANCE - rightDist;
      changeInError =  currentError - previousError;
    }
  }
  else if(wRight && wFront)
  {
    Serial.print("Hard turning left\n");
    setMotors(0.0,1.0);
    currentError = SET_DISTANCE - rightDist;
    changeInError =  currentError - previousError;
  }
  else if(!wRight && !wFront)
  {
    if (lostRightWall){
      Serial.print("Hard turning right\n");
      setMotors(0.9,0.3);  
      currentError = SET_DISTANCE - rightDist;
      changeInError =  currentError - previousError;
    }
    else {
      Serial.print("Turning right\n");
      setMotors(0.9,0.65);  
      currentError = SET_DISTANCE - rightDist;
      changeInError =  currentError - previousError;
      }
  }
  else if(!wRight && wFront)
  {
    Serial.print("Hard turning left\n");
    setMotors(0.,1.0);
    currentError = SET_DISTANCE - rightDist;
    changeInError =  currentError - previousError;   
  }  
  /*
  pidIn = rightDist;
  /*
  Serial.print("PID Input: ");
  Serial.print(pidIn);
  Serial.println("");
  /
  pid.Compute();
  /*
  Serial.print("PID response: ");
  Serial.print(Output);
  Serial.println("");
  
  setMotors(Output);
  Serial.print("Current Error");
  Serial.print(currentError);
  Serial.print("\n");

  Serial.print("Change In Error");
  Serial.print(changeInError);
  Serial.print("\n");
  */
  previousError = currentError;
}

