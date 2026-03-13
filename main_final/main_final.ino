#include<SoftwareSerial.h>
SoftwareSerial Bluetooth(10, 11); // RX, TX

#define S1 A7
#define S2 A6
#define S3 A5
#define S4 A4
#define S5 A3
#define S6 A2
#define S7 A1
#define S8 A0

#define BUTTON1 2
#define BUTTON2 12
#define LED 13

#define PWMA 3
#define AIN1 4
#define AIN2 5
#define STBY 6
#define BIN1 7
#define BIN2 8
#define PWMB 9

const int sensorNumber = 8;
int sensorPins[sensorNumber] = {S1, S2, S3, S4, S5, S6, S7, S8};
int threshold = 200;
int sumSensor;
int sumWeight;
int senD1, senD2, senD3, senD4, senD5, senD6, senD7, senD8;
int senA1, senA2, senA3, senA4, senA5, senA6, senA7, senA8;
int sensorD[sensorNumber] = {senD1, senD2, senD3, senD4, senD5, senD6, senD7, senD8};
int sensorA[sensorNumber] = {senA1, senA2, senA3, senA4, senA5, senA6, senA7, senA8};
int weightValue[sensorNumber] = {10, 20, 30, 40, 50, 60, 70, 80};
String direction = "straight";

int currPos;
float cenPos = 45;   // trung tâm
float ePrev = 0;
float eCurr;
float eSum;
float PID;
float dE;

int speed = 255;
int turnSpeed = 255;
int move_forward_time = 60;//ms
float P = 17;
float D = 3;
float I = 0;

void setup() 
{
  Serial.begin(9600);
  Bluetooth.begin(9600);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(S6, INPUT);
  pinMode(S7, INPUT);
  pinMode(S8, INPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(LED, OUTPUT);

  digitalWrite(STBY, LOW);
  digitalWrite(BUTTON1, HIGH);
  digitalWrite(BUTTON2, HIGH);
}

void loop() 
{
  if(digitalRead(BUTTON1)==0)
  {
    delay(1000);
    digitalWrite(STBY,HIGH);
    while(1)
    {
      read_black_line();
      PID_control(P, I, D);
      delay(5);
    }
  }

  if(digitalRead(BUTTON2)==0)
  {
    while(1)
    {
      debug();
    }
  }
}