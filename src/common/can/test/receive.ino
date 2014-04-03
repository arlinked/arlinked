// demo: CAN-BUS Shield, receive data
#include "mcp2515_can.h"
#include <SPI.h>

long unsigned int rxId;
unsigned char rxBuf[9];

const uint8 pinINTE = 9;
const uint8 pinCS = 10;
MCP_CAN CAN0(pinCS);                               // Set CS to pin 10

unsigned char stmp[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};

extern "C" void setup()
{
	Serial.begin(9600);

	if(CAN0.begin(CAN_125KBPS) == CAN_OK)
		Serial.print("can init ok!!\r\n");
	else
		Serial.print("Can init fail!!\r\n"); 

	pinMode(pinINTE, INPUT);
	CAN0.reportState(0x2b);
	Serial.println("MCP2515 Library Receive Example...");
}

extern "C" void loop()
{
	unsigned char len = 0;
	char tmp[64];
	if(digitalRead(pinINTE) == LOW) {
	if(CAN_NOMSG != CAN0.readMsgBuf(&len, rxBuf))              // Read data: len = data length, buf = data byte(s)
	{
		rxId = CAN0.getCanId();                    // Get message ID
		rxBuf[7] = '\0';
//		Serial.print("rxId:");Serial.print(rxId);Serial.print("  len:");Serial.println(len);
		Serial.println((char*)rxBuf);
	
//		Serial.print(rxId, HEX);
//		Serial.print("  Data: ");
//		for(int i = 0; i<len; i++)                // Print each byte of the data
//		{
//		  if(rxBuf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
//		  {
//		    Serial.print("0");
//		  }
//		  Serial.print(rxBuf[i], HEX);
//		  Serial.print(" ");
//		}
//		Serial.println();
	}
	else
	{
		Serial.println("no message received");
	}
	}
	else
	delay(100);
}

