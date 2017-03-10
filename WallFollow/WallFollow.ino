#include <RedBot.h>
#include <RedBotSoftwareSerial.h>
#include <PID_v1.h>
#include <NewPing.h>

#define LINE_IN1 A0
#define LINE_IN2 A1
#define LINE_IN3 A6

#define MAX_SPEED 100

RedBotMotors motors; // Instantiate the motor control object.

//Define Variables we'll be connecting to
double Setpoint, in1, in2, in3, Output, pidIn;
//Specify the links and initial tuning parameters
double Kp=0.001, Ki=0, Kd=0.0001;
PID pid(&pidIn, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup(){
  Serial.begin(9600);
  Serial.print("Setup Started.\n");
  //initialize the variables we're linked to
  in1 = analogRead(LINE_IN1);
  //in2 = analogRead(LINE_IN2);
  in3 = analogRead(LINE_IN3);
  pidIn = in3 - in1;
  Setpoint = 0;

  //turn the PID on
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-1,1);
  pid.SetSampleTime(50);
  Serial.print("Setup Complete.\n");
}

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

void loop(){
  //delay(50);
  in1 = analogRead(LINE_IN1);
  //in2 = analogRead(LINE_IN2);
  in3 = analogRead(LINE_IN3);
  /*
  Serial.print("Input 1: ");
  Serial.print(in1);
  Serial.println("");
  Serial.print("Input 3: ");
  Serial.print(in3);
  Serial.println("");
  */
  pidIn = in3 - in1;
  /*
  Serial.print("PID Input: ");
  Serial.print(pidIn);
  Serial.println("");
  */
  pid.Compute();
  /*
  Serial.print("PID response: ");
  Serial.print(Output);
  Serial.println("");
  */
  setMotors(Output);
}

