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
//Third L293D PWM pins;
const int EN5 = 10;
//Shift 74HC595 pins:
const int clockPin = 7;
const int latchPin = 4;
const int dataPin = 8;
//Finger potents pins:
const int fingerPotent1 = A0;
const int fingerPotent2 = A1;
const int fingerPotent3 = A2;
const int fingerPotent4 = A4;
const int fingerPotent5 = A5;
//Finger potents thresholds:
int fingerPotent1thresholds[2];
int fingerPotent2thresholds[2];
int fingerPotent3thresholds[2];
int fingerPotent4thresholds[2];
int fingerPotent5thresholds[2];

//EMGValues
int sensorValue1;
int sensorValue2;
int filteredSensorValue;
int threshold1;
int threshold2;

SoftwareSerial btSerial(BT_RxD, BT_TxD);

int i = 0;
char buff[256];
byte lastCommand[2];

void motorCommand(byte byteCmd1, byte byteCmd2){
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, byteCmd1);
  shiftOut(dataPin, clockPin, MSBFIRST, byteCmd2);
  digitalWrite(latchPin, HIGH);
  lastCommand[0] = byteCmd1;
  lastCommand[1] = byteCmd2;
}

void setMotorSpeed(int speed){
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  analogWrite(EN3, speed);
  analogWrite(EN4, speed);
  analogWrite(EN5, speed);
}

void stopSeparatedMotor(byte byteCmd1, byte byteCmd2){
  motorCommand(lastCommand[0] xor byteCmd1, lastCommand[1] xor byteCmd2);
  setMotorSpeed(0);
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
  //SetUp potents pins:
  pinMode(fingerPotent1, INPUT);
  pinMode(fingerPotent2, INPUT);
  pinMode(fingerPotent3, INPUT);
  pinMode(fingerPotent4, INPUT);
  pinMode(fingerPotent5, INPUT);
  //Set motors to standBy by 74HC595:
  motorCommand(B00000000, B00000000);
  //StartUp delay:
  delay(2000);
  Serial.println("Done.");
}

void loop() {
  if (btSerial.available() > 0){
    char ch = btSerial.read();
    if(ch < 60 && ch > 1){
      if (ch == '\n'){
        i = 0;
        buff[i++] = ch;
        //Serial.print(buff);
        sscanf(buff, "%i %i %i %i %i", 
              &sensorValue1,
              &sensorValue2,
              &filteredSensorValue,
              &threshold1,
              &threshold2);
        Serial.print(filteredSensorValue);
        Serial.print(' ');
        Serial.print(threshold1 * 1.5);
        Serial.print(' ');
        Serial.println(threshold2 * 1.2);
        memset(buff, 0, sizeof(buff));
      }else{
        if(i == 255){
          i = 0;
          memset(buff, 0, sizeof(buff));
        } else {
          buff[i++] = ch;
        }
      }
    }
  }
  if(filteredSensorValue > threshold1 * 1.5){
    motorCommand(B10101010, B10000000);
    setMotorSpeed(100);
  } else if(filteredSensorValue < threshold2 * 2){
    motorCommand(B01010101, B10000000);
    setMotorSpeed(100);
  } else{
    motorCommand(B00000000, B00000000);
    setMotorSpeed(0);
  }
  /*
  if (analogRead(fingerPotent1) < fingerPotent1thresholds[0] 
  || analogRead(fingerPotent1) > fingerPotent1thresholds[1]){
    stopSeparatedMotor(B01000000, B00000000);
  } else if (analogRead(fingerPotent2) < fingerPotent2thresholds[0] 
  || analogRead(fingerPotent2) > fingerPotent2thresholds[1]){
    stopSeparatedMotor(B00010000, B00000000);  
  } else if (analogRead(fingerPotent3) < fingerPotent3thresholds[0]
  || analogRead(fingerPotent3) > fingerPotent3thresholds[1]){
    stopSeparatedMotor(B00000100, B00000000);
  } else if (analogRead(fingerPotent4) < fingerPotent4thresholds[0]
  || analogRead(fingerPotent4) > fingerPotent4thresholds[1]){
    stopSeparatedMotor(B00000001, B00000000);
  } else if (analogRead(fingerPotent5) < fingerPotent5thresholds[0]
  || analogRead(fingerPotent5) > fingerPotent5thresholds[1]){
    stopSeparatedMotor(B00000001, B00000000);
  }
  */
}