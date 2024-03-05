#include <SPI.h>
#include "RF24.h"

RF24 radio(7,8); // CE, CSN      
byte address[6] = "00001";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.
String readString;
char text[32]="";
int i = 0;
 bool rslt;
 
void setup() {
 Serial.begin(9600);
radio.begin();           //Starting the Wireless communication
radio.enableAckPayload();
radio.setAutoAck(true);
radio.setRetries(5,5); // delay, count
                                       // 5 gives a 1500 µsec delay which is needed for a 32 byte ackPayload

radio.openWritingPipe(address); //Setting the address where we will send the data
radio.setPALevel(RF24_PA_MIN);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
//radio.stopListening();          //This sets the module as transmitter
//char text[] = "mr1fs00xx";
}

void loop() {
  
  while(!Serial.available()) {}
  // serial read section
  while (Serial.available())
  {
    delay(30);
    
    if (Serial.available() >0)
    {
      if (i==8){
        i=0;
      }
      char c = Serial.read();  //gets one byte from serial buffer
      readString += c; //makes the string readString
      text[i]=c;
      i++;
      //Serial.println(i);
      //text[i]='\0';
    }
  }
  //readString="mr1fs00xx";
  //char text[] = "mr1fs00xx";

  if (readString.length() >0)
  {
    Serial.print("Arduino received: ");  
    //Serial.println(readString); //see what was received
    Serial.println(text);
    rslt=radio.write(&text,sizeof(text));
    //Stext[32]="";
    //delay(250);   
    if (rslt){
          Serial.println("tx ack");
 
    }
    //radio.flush_tx();
    //radio.flush_rx();
  }
  
  
}

