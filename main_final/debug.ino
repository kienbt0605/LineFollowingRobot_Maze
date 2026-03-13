void debug() 
{
  sumSensor = 0;
  sumWeight = 0;

  for (int i = 0; i < sensorNumber; i++) 
  {
    sensorA[i] = analogRead(sensorPins[i]);
    Serial.print(String(sensorA[i]) + " ");

    if(sensorA[i] < 150)
    {
      sensorA[i] = 0;
    }
    else if(sensorA[i] > 300)
    {
      sensorA[i] = 1023;
    }
    sensorD[i] = (sensorA[i] < threshold) ? 0 : 1;  // Đổi từ Analog sang Digital
    sumSensor += sensorD[i];
    sumWeight += sensorD[i] * weightValue[i];  // Cộng dồn trọng số

    // In ra giá trị của sensorA (raw) sau khi đã scale
    Serial.print(String(sensorD[i]) + " ");
    Bluetooth.print(String(sensorD[i]) + " ");
    if ((i + 1) % 8 == 0) 
    {
      Serial.println();  // Xuống dòng sau khi in 8 giá trị
      Bluetooth.println();
      //Serial.print(" SensorD: ");
      //Serial.print(String(sumWeight) + "  ");
    }
  }
}

void debug1(float Kp, float Ki, float Kd) 
{
  read_black_line();
  //Thuật toán PID
    //Serial.print(String(eCurr) + "  ");
    //Serial.print(direction + "  ");
    Bluetooth.print(direction + "  ");
    currPos = sumWeight / sumSensor;
    eCurr = cenPos - currPos;
    dE = eCurr - ePrev;
    eSum += eCurr;

    PID = Kp * eCurr + Kd * dE + Ki * eSum;

    int leftSpeed  = speed - PID;
    int rightSpeed = speed + PID;
    //Serial.print("currPos: " + String(currPos) + "  " + "leftSpeed: " + String(leftSpeed) + "  " + "leftSpeed: " + String(rightSpeed) + "  ");

    ePrev = eCurr;

    motor(leftSpeed, rightSpeed);

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
    if (direction != "straight") 
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
    
    if (bitSensor == 255) 
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