#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"

const int SPI_CS_PIN = 10;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

const int CAN_INT_PIN = 2;

const byte myID = 0x0;

static volatile int flagRecv = 0;

void MCP2515_ISR() {
   flagRecv = 1;
}

void setup() {
   Serial.begin(115200);
   while(!Serial){};
  
   attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MCP2515_ISR, FALLING);
   // init can bus : baudrate = 500k
   while (CAN_OK != CAN.begin(CAN_1000KBPS,MCP_8MHz)) {
      Serial.println("CAN init fail, retry...");
      delay(100);
   }
   Serial.println("CAN init ok!");
   CAN.setSleepWakeup(true);
   CAN.setMode(MODE_NORMAL);
   //CAN.setMode(MODE_LOOPBACK);
}




void printMessage(byte *msg, byte len)
{  
   for(byte i=0;i<len;i++) {
      Serial.print(msg[i],HEX);
      Serial.print(" ");
   }
   Serial.println();
}

byte sendAndPrint(byte *msg,byte len)
{
   Serial.print(F("Sending Message: "));
   printMessage(msg,len);
   byte res=CAN.sendMsgBuf(myID, 0, len,msg);
   Serial.print(F("SendMessage returns: "));Serial.println(res,HEX);
   return res;
}

void checkReceive() {
   byte recvMsg[16];
   byte len;   

   if (flagRecv) {
      flagRecv = 0;
      while (CAN_MSGAVAIL == CAN.checkReceive()) {
            Serial.println(CAN.getCanId(),HEX);
            memset(recvMsg,0,sizeof(recvMsg));
            CAN.readMsgBuf(&len,recvMsg);
            Serial.print(F("Received Message with length: "));
            Serial.print(len);
            if (len>0) {
               Serial.print(F(" => "));
               printMessage(recvMsg,len);
            } 
       }      
   }
}

byte readpos[] = {0x96,0,0x0c,0,0x0c};
byte readvoltage[] = {0x96,0,0x12,0,0x012};
byte neutral[] = {0x96,0,0x1e,2,0xb8,0x0b,0xe3};
byte left[] = {0x96,0,0x1e,2,0x58,0x02,0x7a};

void loop() {

    if (CAN.getMode()==MODE_LOOPBACK) {
       Serial.println(F("\nLoopback test"));
       sendAndPrint(neutral,sizeof(neutral));
       checkReceive();
       delay(1000);
    }
    else {
      
      byte res=sendAndPrint(readvoltage,sizeof(readvoltage)); 
      if (res==CAN_OK) checkReceive();    
      delay(1000);
      // sendAndPrint(neutral,sizeof(neutral));
      // delay(1000);
      // sendAndPrint(left,sizeof(left));
      // delay(1000);

    }

}