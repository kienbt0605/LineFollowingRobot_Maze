void stop()
{
  motor(0,0);
  digitalWrite(LED,HIGH);
}

void move_forward(int moveTime)
{
  int startTime = millis();  // Lưu thời gian bắt đầu
  while (millis() - startTime < moveTime) 
  {
    motor(speed, speed);  // Tiếp tục di chuyển xe
  }
}

void PID_control(float Kp, float Ki, float Kd) 
{
  //Thuật toán PID
  if(sumSensor > 0)
  {
    currPos = sumWeight / sumSensor;
    eCurr = cenPos - currPos;
    dE = eCurr - ePrev;
    eSum += eCurr;

    PID = Kp * eCurr + Kd * dE + Ki * eSum;

    int leftSpeed  = speed - PID;
    int rightSpeed = speed + PID;

    ePrev = eCurr;

    motor(leftSpeed, rightSpeed);
  }
//Xử lý rẽ phải trái  
int bitSensor = 0;
  for(int i = 0; i < 8; i++) 
  {
    bitSensor = (bitSensor << 1) | sensorD[i];  // Chuyển mảng sensorD[] thành bit
  }
  //Serial.println(bitSensor);

  switch(bitSensor) 
  {
  // left side detection
    case 0b11110000: direction = "left"; break;
    case 0b11111000: direction = "left"; break;
    case 0b11111100: direction = "left"; break;
    case 0b11111110: direction = "left"; break;

  // right side detection
    case 0b00001111: direction = "right"; break;
    case 0b00011111: direction = "right"; break;
    case 0b00111111: direction = "right"; break;
    case 0b01111111: direction = "right"; break;
  }

//Rẽ khúc cua gấp
  if(bitSensor == 0) 
  {
    if(direction != "straight") 
    {
      if(direction == "right") 
      {
        turnRight();
      } 
      else 
      {
        turnLeft();
      }
    }
  }
//Chữ T
  else if(bitSensor == 255) 
  {
    move_forward(move_forward_time);  
    read_black_line();
    
    if (sumSensor == 0) 
    {
      turnLeft();
    }

//Dấu cộng 
    else
    {
      if(bitSensor > 0 && bitSensor < 255) 
      {
        direction == "straight"; 
      }
    }
//Đích
    while(sumSensor == 8)
    {
      stop();
    }
  }
}