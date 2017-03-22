#include <RedBot.h>
#include <RedBotSoftwareSerial.h>
#include <PID_v1.h>

#define MAX_SPEED 110
// Experimentally determined, most line readings are in range [850, 950] 
// most wood readings are in range [50, 550]
#define THRESHOLD 800

// States
#define S_STRAIGHT        0
#define S_STRAIGHT_LEFT   1
#define S_STRAIGHT_RIGHT  2
#define S_LEFT            3
#define S_RIGHT           4

#define PIN_LINE_LEFT  A0
#define PIN_LINE_MID   A1
#define PIN_LINE_RIGHT A6

// Want the left and right sensors to read the same value
#define SETPOINT 0

// Redbot motor(s) object
RedBotMotors motors;

// PID object and parameters
double Setpoint = SETPOINT, pidOut, pidIn;
double Kp=0.01, Ki=0.04, Kd=0.0001;
PID pid(&pidIn, &pidOut, &Setpoint, Kp, Ki, Kd, DIRECT);

// Takes doubles for left and right as percentage 0.0 - 1.0
void setMotors(double left, double right) {
  if (left * MAX_SPEED > 255) {
    left = 255.0/MAX_SPEED;
  }
  if (right * MAX_SPEED > 255) {
    right = 255.0/MAX_SPEED;
  }
  motors.leftMotor(0 - int(MAX_SPEED * left));
  motors.rightMotor(0 - int(MAX_SPEED * right));
}

double leftreading, rightreading;
double getError() {
  leftreading = (double)analogRead(PIN_LINE_LEFT);
  rightreading = (double)analogRead(PIN_LINE_RIGHT);
  return leftreading - rightreading;
}

void setup(){
  Serial.begin(9600);
  Serial.print("Setup Started.\n");

  //turn the PID on
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-1,1);
  pid.SetSampleTime(2);

  Serial.print("Setup Complete.\n");
}

bool compute = false;
double error = 0;
int state = 0;

void loop() {

  compute = false;
  while (compute == false) {
    error = getError();
    pidIn = error;
    compute = pid.Compute();
  }
  
  switch (state) {
    case S_STRAIGHT:
      setMotors(1+pidOut, 1-pidOut);
      if (leftreading > THRESHOLD && rightreading > THRESHOLD) {
        // random turn
      }
      else if (leftreading > THRESHOLD) {
        state = S_STRAIGHT_RIGHT;
      }
      else if (rightreading > THRESHOLD) {
        state = S_STRAIGHT_LEFT;
      }
      break;
      
    case S_STRAIGHT_LEFT: // straightaway, left adjustment
      setMotors(1+pidOut, 1-pidOut);
      if (leftreading > THRESHOLD && rightreading > THRESHOLD) {
        state = S_LEFT;
      }
      else if (leftreading < THRESHOLD) {
        state = S_STRAIGHT;
      }
      break;
    
    case S_STRAIGHT_RIGHT: // straightaway, right adjustment
      setMotors(1+pidOut, 1-pidOut);
      if (leftreading > THRESHOLD && rightreading > THRESHOLD) {
        state = S_RIGHT;
      }
      else if (rightreading < THRESHOLD) {
        state = S_STRAIGHT;
      }
      break;
     
    case S_LEFT: // hard left turn
      setMotors(-0.3, 1);
      if (leftreading < THRESHOLD) {
        state = S_STRAIGHT;
      }
      break;
   
    case S_RIGHT: // hard right turn
      setMotors(1, -0.3);
      if (rightreading < THRESHOLD) {
        state = S_STRAIGHT;
      }
      break;  
  } 
}
