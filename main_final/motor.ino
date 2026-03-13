void motor(int LPWM, int RPWM) 
{
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  if (LPWM >= 0) 
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } 
  else 
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }

  if (RPWM >= 0) 
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } 
  else 
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  analogWrite(PWMA, abs(LPWM));
  analogWrite(PWMB, abs(RPWM));
}