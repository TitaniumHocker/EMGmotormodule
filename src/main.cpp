#include <Arduino.h>
#include <SoftwareSerial.h>

//Bluetooth HC-05 pins:
const int BT_RxD = 13;
const int BT_TxD = 12;
//First L293D PWM pins:
const int EN1 = 3;
const int EN2 = 5;
//Second L293D PWM pins;
const int EN3 = 6;
const int EN4 = 9;
//Shift 74HC595 pins:
const int clockPin = 7;
const int latchPin = 4;
const int dataPin = 8;
//Byte Commands:
const byte standBy = B00000000;
const byte toFist = B01010101;
const byte toFlat = B10101010;

int sensorValue1;
int sensorValue2;
int filteredSensorValue;
int threshold1;
int threshold2;

SoftwareSerial btSerial(BT_RxD, BT_TxD);

int i = 0;
char buff[50];

void motorCommand(byte byteCmd){
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, byteCmd);
  digitalWrite(latchPin, HIGH);
}

void setMotorSpeed(int speed){
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  analogWrite(EN3, speed);
  analogWrite(EN4, speed);
}

void setup() {
  //SetUp Serials:
  Serial.begin(9600);
  Serial.print("Getting started...");
  btSerial.begin(38400);
  //SetUP motors and speed to zero:
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);
  pinMode(EN4, OUTPUT);
  setMotorSpeed(0);
  //SetUp 74HC595:
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  //Set motors to standBy by 74HC595:
  motorCommand(standBy);
  //StartUp delay:
  delay(2000);
  Serial.println("Done.");
}

void loop() {
  if (btSerial.available() > 0){
    char ch = btSerial.read();
    if (ch == '\n'){
      i = 0;
      buff[i++] = ch;
      Serial.print(buff);
      sscanf(buff, "%i %i %i %i %i", 
            &sensorValue1,
            &sensorValue2,
            &filteredSensorValue,
            &threshold1,
            &threshold2);
      //Serial.println(filteredSensorValue);
      memset(buff, 0, sizeof(buff));
    }else{
      buff[i++] = ch;
    }
  }
  if(filteredSensorValue > threshold1 * 1.5){
    motorCommand(toFist);
    setMotorSpeed(100);
  } else if(filteredSensorValue < threshold2 * 2){
    motorCommand(toFlat);
    setMotorSpeed(100);
  } else{
    motorCommand(standBy);
    setMotorSpeed(0);
  }
}