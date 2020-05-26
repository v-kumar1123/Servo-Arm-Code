#include <VarSpeedServo.h>

#include <RH_ASK.h>
#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
int joyX = 0;//joystick pin for X direction for servos
int joyY = 1;//joystick pin for Y direction for motors, joystick 2 plugs in here

int joyValY;//y-position
int joyValX;//x-position
int joy2ValY;
int speedX=0;
const byte address[6] = "00001";

String text="";

RF24 radio(9, 10);
int speedY=0;
VarSpeedServo shoulderRot;
VarSpeedServo elbow;
VarSpeedServo gripper;
VarSpeedServo wrist;

void setup() {
  radio.begin();
  //set the address
  radio.openReadingPipe(0, address); 
  //Set module as receiver
  radio.startListening();
  //rf_driver.init();
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, INPUT);
  digitalWrite(13, HIGH);
  shoulderRot.attach(7);
  elbow.attach(10);
  elbow.write(90);
  gripper.attach(5);
  wrist.attach(2);
  //TIMSK0=0;
  
}

void loop() {
  // put your main code here, to run repeatedly:
  char buf[32] = {0};
  radio.read(&buf, sizeof(buf));
  uint8_t buflen = sizeof(buf);
  Serial.println(buf);
  
  speedY=abs(joyValY-500)/100;
  //Serial.println(digitalRead(13));

  Serial.println(joyValY);
  if(radio.available()){
    partCounter=0;
    for (j=0;j<buflen;j++){
      if (buf[j]==':'){
        partCounter++;

        if (partCounter==4) {
          //JOY3x
          joyValX=text.toInt();
          speedX=abs(joyValX-520)/100;
        }
        if (partCounter==5) {
          joyValY=text.toInt();
          speedY=abs(joyValY-500)/100;
        }
        text="";
        continue;
      }
      text+=(char)(buf[j]);
    }
  }

  if(digitalRead(13)==0){
    grip();
  }
  if(digitalRead(13)==1){
    letgo();
  }
  if(joyValX>520){
    left();
  }
  if(joyValX<500){
    right();
  }
  if(joyValY<490){
    forward();
  }
  if(joyValY>507){
    backward();
  }
  delay(20);
  //Serial.println(joyValY);
}
void right(){
  shoulderRot.write(shoulderRot.read()+speedX);
}

void left(){
  shoulderRot.write(shoulderRot.read()-speedX);
}

void forward(){
  elbow.write(elbow.read()+speedY);
  //Serial.println(elbow.read());
}
void backward(){
  elbow.write(elbow.read()-speedY);
}
void grip(){
  gripper.write(gripper.read()-3);
}
void letgo(){
  gripper.write(gripper.read()+100);
}
