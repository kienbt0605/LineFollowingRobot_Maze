void turnRight() 
{
  while(sensorD[4] != 1) // Phá vỡ vòng lặp khi sensor 5 được phát hiện
  { 
    motor(turnSpeed, -turnSpeed); // tiến trái, lùi phải
    digitalWrite(LED,HIGH);
    read_black_line(); // tiếp tục quan sát thay đổi của cảm biến
    direction = "straight"; // set direction thành default
  }
  digitalWrite(LED,LOW);
}

void turnLeft() 
{
  while(sensorD[4] != 1) 
  {
    motor(-turnSpeed, turnSpeed); 
    digitalWrite(LED,HIGH);
    read_black_line(); 
    direction = "straight";
  }
  digitalWrite(LED,LOW);
}