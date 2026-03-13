void read_black_line()
{
  sumSensor = 0;  
  sumWeight = 0;  

  // Đọc giá trị cảm biến
  for (int i = 0; i < sensorNumber; i++) 
  {
    sensorA[i] = analogRead(sensorPins[i]);

    // Scale giá trị về [0, 1023]
    if(sensorA[i] < 200)
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

    // Lưu các giá trị trong mảng sensorD[i] để dùng cho tính bitSensor bên PID_control
    if (i == 0) senD1 = sensorD[i];
    if (i == 1) senD2 = sensorD[i];
    if (i == 2) senD3 = sensorD[i];
    if (i == 3) senD4 = sensorD[i];
    if (i == 4) senD5 = sensorD[i];
    if (i == 5) senD6 = sensorD[i];
    if (i == 6) senD7 = sensorD[i];
    if (i == 7) senD8 = sensorD[i];
  }
}
