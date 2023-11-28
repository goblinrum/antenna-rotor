const int IN1 = 22;
const int IN2 = 23;
const int PWM = 21;
void setup()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
}

void loop()
{
  Motor_Brake();
  delay(100);
  Motor_Forward(70); // Forward, PWM setting 0-255
  delay(3000);
  Motor_Brake();
  delay(100);
  Motor_Backward(70); // Reverse, PWM setting 0-255  
  delay(3000);
}

void Motor_Forward(int Speed)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, Speed);
}
void Motor_Backward(int Speed)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(PWM, Speed);
}
void Motor_Brake()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}