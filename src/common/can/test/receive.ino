// demo: CAN-BUS Shield, receive data
#include "mcp2515_can.h"
#include <SPI.h>

const uint8 pinCS = 10;
can::CCanMcp2515Impl can10(pinCS);                               // Set CS to pin 10

unsigned char stmp[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};

extern "C" void setup()
{
	Serial.begin(9600);

	if(can10.begin(CAN_125KBPS) == CAN_OK)
		Serial.print("can init ok!!\r\n");
	else
		Serial.print("Can init fail!!\r\n"); 

	Serial.println("MCP2515 Library Receive Example...");
}

extern "C" void loop()
{
  can::CCanMessage msg0, msg1; 
  uint8 nRecvStat = can10.recv(&msg0, &msg1);              // Read data: len = data length, buf = data byte(s)

  if(nRecvStat)
  {
    if(MCP_SPI_DAT_RX_STATUS_BIT_RX0 & nRecvStat)
    {
    	Serial.print("RXB0: ");
    	Serial.println((char*)msg0.getData());
    }
    if(MCP_SPI_DAT_RX_STATUS_BIT_RX1 & nRecvStat)
    {
    	Serial.print("RXB1: ");
    	Serial.println((char*)msg1.getData());
    }
  }
  else
  {
  	Serial.println("no message received");
    delay(1000);
  }
  delay(100);
}

