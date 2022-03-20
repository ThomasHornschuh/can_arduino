#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_can.h"

const int SPI_CS_PIN = 10;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

const int CAN_INT_PIN = 2;

const byte myID = 0x0;

const uint32_t responseWaitTimeout = 30; // 30ms 

enum hitec_opcodes  {
   reg_pos = 0x0c,
   reg_torque = 0x10,
   reg_voltage = 0x12,
   reg_temp = 0x14,
   reg_setpos = 0x1e
};

enum servo_pos {
   servo_left = 700,
   servo_neutral = 3000,
   servo_right = 5300
};

static volatile int flagRecv = 0;

void MCP2515_ISR() {
   flagRecv = 1;
}



uint16_t readServoValue(enum hitec_opcodes op) {
byte msgbuf[7];


   msgbuf[0]=0x96; msgbuf[1] = 0;
   msgbuf[2] = op; msgbuf[3] = 0;
   msgbuf[4] = (byte)op; // Checksum

   byte res=CAN.sendMsgBuf(myID,0,5, msgbuf);
   if (res==CAN_OK) {
      uint32_t timeout = millis() + responseWaitTimeout;
      while(!flagRecv) {
         if (millis()>timeout) return 0; // Avoid endless loop when there os no response from servo
      }; 
      flagRecv=0;
      if (CAN_MSGAVAIL == CAN.checkReceive()) {
          byte len;
          CAN.readMsgBuf(&len,msgbuf);
          if (len>0 && msgbuf[2]==op && msgbuf[3]==2) {
             return (uint16_t)(msgbuf[4] | msgbuf[5] << 8);               
          }
      } 
   }
   return 0;
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


byte setServoPos(uint16_t pos)
{
   byte msgbuf[7];


   msgbuf[0]=0x96; msgbuf[1] = 0;
   msgbuf[2] = reg_setpos; msgbuf[3] = 2;
   msgbuf[4] = pos & 0xff; msgbuf[5] = pos >> 8;
   byte chksum = 0;
   for(byte i=1;i<=5;i++) chksum+=msgbuf[i];
   msgbuf[6] = chksum;
   return sendAndPrint(msgbuf,7);
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


uint32_t trackPos(uint32_t time,uint16_t target,uint16_t delta)
{
uint32_t timeout = millis()+time;
uint32_t start = millis();  
int counter = 0;

   Serial.print(F("tracking Target: "));Serial.println(target);
   while (millis() < timeout ) {
       uint16_t v=readServoValue(reg_pos);
       counter++;
       if (v==0) return millis() - start; // Servo not responding...
       Serial.print(" ");Serial.print(v);
       if (abs(target-v)<delta) {
          uint32_t delta = millis() - start;
          Serial.print(F("\nTarget reached after: "));Serial.print(delta);Serial.print(F(" ms loops: "));
          Serial.println(counter);
          return delta; // Target position reached
       } 
   }
   return time;  
}

void printInfo()
{
   Serial.print("Servo Voltage: ");Serial.print((float)readServoValue(reg_voltage)/100);Serial.println("V");
   Serial.print("Servo Temp: ");Serial.print(readServoValue(reg_temp));Serial.println("C");   
}


void setup() {
   Serial.begin(115200);
   while(!Serial){};
   Serial.println(F("CANArduino 0.0"));
  
   attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MCP2515_ISR, FALLING);
   // init can bus : baudrate = 500k
   while (CAN_OK != CAN.begin(CAN_1000KBPS,MCP_8MHz)) {
      Serial.println("CAN init fail, retry...");
      delay(100);
   }
   Serial.println("CAN init ok!");
   CAN.setSleepWakeup(true);
   CAN.setMode(MODE_NORMAL);
   printInfo();
   setServoPos(servo_neutral);
   delay(3000);
   //CAN.setMode(MODE_LOOPBACK);
}


void loop() {
uint32_t t;   

    if (CAN.getMode()==MODE_LOOPBACK) {
       Serial.println(F("\nLoopback test"));
       setServoPos(servo_neutral);
       checkReceive();
       delay(1000);
    }
    else {
      
      // byte res=sendAndPrint(readvoltage,sizeof(readvoltage)); 
      // if (res==CAN_OK) checkReceive();    
     
      printInfo();
      setServoPos(servo_left);
      t=trackPos(400,servo_left,100);
      delay(1000L-t);
      Serial.println();
      setServoPos(servo_right);
      t=trackPos(400,servo_right,100);
      delay(1000L-t);
      Serial.println();

    }

}