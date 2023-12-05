#include "AS5600.h"
#include "Wire.h"
#include <PID_v1.h>

#define IN1 2
#define IN2 4
#define PWM 15

AS5600 as5600;   //  use default Wire

double offset = 0;
double adjustedAngle = 0;
double desiredAngle = 0;
double motorOutput = 0;

// PID coefficients
double kp = 5;  // Proportional coefficient
double ki = 0.1;  // Integral coefficient
double kd = 0.05;  // Derivative coefficient
PID myPID(&adjustedAngle, &output, &desiredAngle, kp, ki, kd, DIRECT);

void setup() {

  // Serial begin
  Serial.begin(9600);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  Serial.println(as5600.getAddress());

  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  delay(1000);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);

  calibrate_encoder();

  myPID.SetMode(AUTOMATIC);
}

void loop() {
  getAngle();
  Serial.println("Enter an angle between 0 and 360");
  if (Serial.available() > 0) {
    desiredAngle = Serial.parseFloat();
  }
  desiredAngle = Serial.parseFloat();
  myPID.Compute();
  move_to_angle();
  delay(100);
  Serial.print("Desired angle: ");
  Serial.println(desiredAngle);
  Serial.print("Adjusted angle: ");
  Serial.println(adjustedAngle);
  Serial.print("Output: ");
  Serial.println(output);

}

void calibrate_encoder() {
  Serial.println("Move the motor to the zero position");
  delay(2000);
  offset = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
  Serial.println("Calibration complete");
  Serial.print("Offset: ");
  Serial.println(offset);
}

void getAngle() {
  adjustedAngle = as5600.rawAngle() * AS5600_RAW_TO_DEGREES - offset;
  if (adjustedAngle < 0) {
    adjustedAngle += 360;
  } else if (adjustedAngle >= 360) {
    adjustedAngle -= 360;
  }
}

void move_to_angle() {
  float leeway = 5;
  // find delta considering mod 360
  float delta = int(desiredAngle - adjustedAngle) % 360;
  if (abs(delta) < leeway) {
    move_motor(0);
  } else {
    // constrain motor output to -70/70
    output = constrain(output, -60, 60);
    move_motor(output);
  }
}


void move_motor(int speed) {
  if (speed > 0) {
    Motor_Forward(speed);
  } else if (speed < 0) {
    Motor_Backward(-speed);
  } else {
    Motor_Brake();
  }
}

void Motor_Forward(int Speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, Speed);
}

void Motor_Backward(int Speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(PWM, Speed);
}

void Motor_Brake() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}