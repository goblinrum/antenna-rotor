#include "AS5600.h"
#include "Wire.h"

#define IN1 2
#define IN2 4
#define PWM 15

AS5600 as5600;   //  use default Wire

float offset = 0;
float adjustedAngle = 0;
float desiredAngle = 0;
float integral = 0;
float previous_error = 0;

// PID coefficients
float kp = 0.5;  // Proportional coefficient
float ki = 0.5;  // Integral coefficient
float kd = 0.01;  // Derivative coefficient

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
}

void loop() {
  getAngle();
  Serial.println("Enter an angle between 0 and 360");
  if (Serial.available() > 0) {
    desiredAngle = Serial.parseFloat();
  }
  desiredAngle = Serial.parseFloat();
  move_to_angle();
  delay(100);
  Serial.println(adjustedAngle);

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
    move_motor(40);
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