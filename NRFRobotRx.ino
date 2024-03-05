#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
int l_motorpin1 = 3;                  //define digital output pin no.
int l_motorpin2 = 4;
int r_motorpin3 = 5;
int r_motorpin4 = 6;  

//int l_enable1 = 10̣; // speed control for wheel motors
int r_enable2 = 9; // speed control for wheel motors
int l_enable1 = 10;
//For claw pins
//Digital pins 14 and 15 are for claw motors
int claw_pin1 = 15;
int claw_pin2 = 16;
int platform_pin1 = 17;
int platform_pin2 = 18;
char claw_state[]="";
char platform_state[]="";
char speed_state[] = "";
int forTime = 0;
int i = 0;
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.enableAckPayload();
  radio.setAutoAck(true);
  //radio.printDetails();
  radio.startListening();

  pinMode(l_motorpin1,OUTPUT);        //set pin 3 as output
  pinMode(l_motorpin2,OUTPUT);        // set pin 4 as output
  pinMode(r_motorpin3,OUTPUT);        //set pin 3 as output
  pinMode(r_motorpin4,OUTPUT);        // set pin 4 as output
  pinMode(l_enable1,OUTPUT);        //set pin 2 left motor enable as output
  pinMode(r_enable2,OUTPUT);        // set pin 7 right motor enable as output

  pinMode(claw_pin1,OUTPUT);  
  pinMode(claw_pin2,OUTPUT);
  pinMode(platform_pin1, OUTPUT);
  pinMode(platform_pin2, OUTPUT);


}
void loop() {
  if (radio.available()) {
    Serial.println("radio avlbl");
    char text[32] = "";
    radio.read(&text, sizeof(text));
    
    Serial.println(text);
    delay(250);
    radio.flush_rx();
    radio.flush_tx();
    // text will be mr2ff00xx - 9 characters
    // m = master
    // r = robot
    // 2 = robot number 2
    // f = speed; f: fast; m = medium; s = slow
    // f = movement; forward, backward, left turn, right turn, stop
    // 2 chars = angle to turn; 00 for forward and backward
    // x = claw operation; close (c) or open (o) claw or stop (x)
    // x = platform operation; up (u) or down (d) platform or stop (x)
    //Serial.println(text);
    //Serial.println("text1");
    // set the speed here by checking 4th byte (byte number = 3)
    // f = fast
    // m = medium
    // s = slow
   setSpeed(text[3]);
   Serial.println(text);
    switch (text[4]){
      case 'f':
         Serial.println("came in switch f");
         forTime = 1;
         moveForward(forTime);
         stopCar();
         break;
      case 'b':
         forTime = 1;
         moveBackward(forTime);
         stopCar();
         break;
      case 'l':
         forTime = 1;
         leftTurn(forTime);
         stopCar();
         break;
      case 'r':
         forTime = 1;
         rightTurn(forTime);
         stopCar();
         break;
      case 'x':
         stopCar();
         break;
         
    }
   // claw operation
    switch (text[7]){
      case 'c':
         forTime = 1;
         closeClaw(forTime);
         stopClaw();
         claw_state[0] = 'x';
         break;
      case 'o':
         forTime = 1;
         openClaw(forTime);
         stopClaw();
         claw_state[0] = 'x';
         break;
      case 'x':
         if (claw_state[0] != 'x'){
         forTime = 1;
         stopClaw();
         claw_state[1] = 'x';
         }
         break;
    }
         // platform operation
       switch (text[8]){
      case 'u':
         forTime = 1;
         upPlatform(forTime);
         stopPlatform();
         platform_state[0] = 'x';
         break;
      case 'd':
         forTime = 1;
         downPlatform(forTime);
         stopPlatform();
         platform_state[0]='x';
         break;
      case 'x':
        if (platform_state[0] != 'x'){
         forTime = 1;
         stopPlatform();
         platform_state[0]='x';
        }
         break;
    }
     radio.writeAckPayload( 1, &i, sizeof(int) );
     Serial.println("sent ack");
      delay(500);

 
  }
}

void setSpeed(char speed){
  // set pulse width
  Serial.println("Speed = f");
 analogWrite(l_enable1,200);// run left motor as PWM 150 speed
  analogWrite(r_enable2,200);// run right motor as PWM 150 speed
  return;
}
void moveForward(int delayTime) {
  Serial.println("came into moveForward");
  digitalWrite(l_motorpin1,LOW);
  digitalWrite(l_motorpin2,HIGH);
  digitalWrite(r_motorpin3,LOW);
  digitalWrite(r_motorpin4,HIGH);
  delay(delayTime*1000);// delay in milliseconds
  //stopCar();
  return;
}

void moveBackward(int delayTime) {
  Serial.println("came into moveBackward");
  digitalWrite(l_motorpin1,HIGH);
  digitalWrite(l_motorpin2,LOW);
  digitalWrite(r_motorpin3,HIGH);
  digitalWrite(r_motorpin4,LOW);
  delay(delayTime*1000);// delay in milliseconds
  //stopCar();
  return;
}

void leftTurn(int delayTime) {
  Serial.println("came into leftTurn");
  digitalWrite(l_motorpin1,HIGH);
  digitalWrite(l_motorpin2,LOW);
  digitalWrite(r_motorpin3,LOW);
  digitalWrite(r_motorpin4,HIGH);
  delay(delayTime*1000);// delay in milliseconds
  //stopCar();
  return;
}

void rightTurn(int delayTime) {
  Serial.println("came into rightTurn");
  // TODO: logic for turning a specific angle; 45 and 90 degrees
  digitalWrite(l_motorpin1,LOW);
  digitalWrite(l_motorpin2,HIGH);
  digitalWrite(r_motorpin3,HIGH);
  digitalWrite(r_motorpin4,LOW);
  delay(delayTime*1000);// delay in milliseconds
  //stopCar();
  return;
}

void stopCar() {
  Serial.println("came into stop");
  digitalWrite(l_motorpin1,LOW);
  digitalWrite(l_motorpin2,LOW);
  digitalWrite(r_motorpin3,LOW);
  digitalWrite(r_motorpin4,LOW);
  //digitalWrite(claw_pin1,LOW);
  //digitalWrite(claw_pin2,LOW);
  //digitalWrite(platform_pin1,LOW);
  //digitalWrite(platform_pin2,LOW);
  //delay(500);// stop for 500 milliseconds
  return;
}

void closeClaw(int delayTime){
  Serial.println("came into Close claw");
  digitalWrite(claw_pin1, LOW);
  digitalWrite(claw_pin2, HIGH);
  delay(delayTime*1000);// delay in milliseconds
  return;
 
}

void upPlatform(int delayTime){
  Serial.println("came into up platform");
  digitalWrite(platform_pin1, HIGH);
  digitalWrite(platform_pin2, LOW);
  delay(delayTime*1000);// delay in milliseconds
  return;
 
}
void openClaw(int delayTime){
  Serial.println("came into Open claw");
  digitalWrite(claw_pin1, HIGH);
  digitalWrite(claw_pin2, LOW);
  delay(delayTime*1000);// delay in milliseconds
  return;
}

void downPlatform(int delayTime){
  Serial.println("came into down platform");
  digitalWrite(platform_pin1, LOW);
  digitalWrite(platform_pin2, HIGH);
  delay(delayTime*1000);// delay in milliseconds
  return;
}

void stopClaw() {
  Serial.println("came into stopClaw");
  digitalWrite(claw_pin1,LOW);
  digitalWrite(claw_pin2,LOW);
  //delay(delayTime*1000);// delay in milliseconds
  return;
}

void stopPlatform() {
  Serial.println("came into stopPlatform");
  digitalWrite(platform_pin1,LOW);
  digitalWrite(platform_pin2,LOW);
  //delay(delayTime*1000);// delay in milliseconds
  return;
}
