#include <RedBot.h>
#include <RedBotSoftwareSerial.h>
#include <PID_v1.h>
#include <NewPing.h>

#define MAX_SPEED 180

#define PIN_FRONT_TRIG  A4
#define PIN_FRONT_ECHO  A5
#define PIN_RIGHT_TRIG  A2
#define PIN_RIGHT_ECHO  A3

// Distance from the wall
#define SETPOINT 18
#define MIN_DISTANCE 3
#define MAX_DISTANCE 200

// Sonar objects
NewPing frontSonar(PIN_FRONT_TRIG, PIN_FRONT_ECHO, MAX_DISTANCE);
NewPing rightSonar(PIN_RIGHT_TRIG, PIN_RIGHT_ECHO, MAX_DISTANCE);

// Redbot motor(s) object
RedBotMotors motors;

// PID object and parameters
double Setpoint = SETPOINT, pidOut, pidIn;
double Kp=0.01, Ki=0, Kd=0.015;
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

void turnLeft() {
  // Turn until the front is 'clear'
  int frontDist = 10;
  while ((frontDist < 20) && (frontDist != 0)) {
    setMotors(-1, 1);
    delay(50);
    frontDist = frontSonar.ping_cm();
  }

  delay(200);

  /*// Turn until the right side stops decreasing
  int current = 199;
  int previous = MAX_DISTANCE;
  while (current <= previous) {
    setMotors(-1, 1);
    previous = current;
    current = rightSonar.ping_cm();
    delay(50);
  }*/
}

void setup(){
  Serial.begin(9600);
  Serial.print("Setup Started.\n");

  //turn the PID on
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-0.55,0.55);
  pid.SetSampleTime(50);

  Serial.print("Setup Complete.\n");
}

bool compute = false;
int rightDist = 0, frontDist = 0;

void loop() {

  frontDist = frontSonar.ping_cm();
  Serial.print(frontDist);
  Serial.print('\n');
  if ((frontDist < 20) && (frontDist != 0)) {
    turnLeft();
  }
  delay(20);

  compute = false;
  while (compute == false) {
    rightDist = rightSonar.ping_cm();
    pidIn = rightDist;
    compute = pid.Compute();
  }

  setMotors(1-pidOut, 1+pidOut);
}
