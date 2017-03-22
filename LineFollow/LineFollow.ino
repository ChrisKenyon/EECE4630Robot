#include <RedBot.h>
#include <RedBotSoftwareSerial.h>
#include <PID_v1.h>

#define MAX_SPEED 100

#define PIN_LINE_LEFT  A0
#define PIN_LINE_MID   A1
#define PIN_LINE_RIGHT A6

// Want the left and right sensors to read the same value
#define SETPOINT 0

// Redbot motor(s) object
RedBotMotors motors;

// PID object and parameters
double Setpoint = SETPOINT, pidOut, pidIn;
double Kp=0.001, Ki=0, Kd=0.0001;
PID pid(&pidIn, &pidOut, &Setpoint, Kp, Ki, Kd, DIRECT);

// Takes doubles for left and right as percentage 0.0 - 1.0
void setMotors(double left, double right) {
  if (left * MAX_SPEED > 255) {
    left = 255.0/MAX_SPEED;
  }
  if (right * MAX_SPEED > 255) {
    right = 255.0/MAX_SPEED;
  }
  motors.rightMotor(0 - int(MAX_SPEED * right));
  motors.leftMotor(0 - int(MAX_SPEED * left));
}

double getError() {
  return (double)(analogRead(PIN_LINE_LEFT)-analogRead(PIN_LINE_RIGHT));
}

void setup(){
  Serial.begin(9600);
  Serial.print("Setup Started.\n");

  //turn the PID on
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-0.5,0.5);
  pid.SetSampleTime(20);

  Serial.print("Setup Complete.\n");
}

bool compute = false;
double error = 0;

void loop() {

  compute = false;
  while (compute == false) {
    error = getError();
    pidIn = error;
    compute = pid.Compute();
  }

  setMotors(1-pidOut, 1+pidOut);
}
