#include "AS5600.h"
#include "Wire.h"

#define IN1 22
#define IN2 23
#define PWM 21

AS5600 as5600;   //  use default Wire

float offset = 0;
float adjustedAngle = 0;

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
  Serial.println(adjustedAngle);
  delay(100);

  // Motor_Brake();
  // delay(100);
  // Motor_Forward(70); // Forward, PWM setting 0-255
  // delay(3000);
  // Motor_Brake();
  // delay(100);
  // Motor_Backward(70); // Reverse, PWM setting 0-255  
  // delay(3000);
}

void calibrate_encoder() {
  Serial.println("Move the motor to the zero position");
  delay(1000);
  offset = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
}

void getAngle() {
  adjustedAngle = as5600.rawAngle() * AS5600_RAW_TO_DEGREES - offset;
  if (adjustedAngle < 0) {
    adjustedAngle += 360;
  } else if (adjustedAngle >= 360) {
    adjustedAngle -= 360;
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