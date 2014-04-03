// demo: CAN-BUS Shield, send data
#include "mcp2515_can.h"
#include <SPI.h>


MCP_CAN CAN0(10);                                      // Set CS to pin 10

extern "C" void setup()
{
	Serial.begin(9600);

	// init can bus, baudrate: 500k
	if(CAN0.begin(CAN_125KBPS) == CAN_OK)
		Serial.print("can init ok!!\r\n");
	else
		Serial.print("Can init fail!!\r\n");
}

unsigned int counter = 0;

extern "C" void loop()
{
	char stmp[8];
	snprintf(stmp, 8, "%07d", counter);

	// send data:  id = 0x00, standrad flame, data len = 8, stmp: data buf
	CAN0.sendMsgBuf(0x0e, 0, 8, (byte*)stmp);
	Serial.println(counter);
	delay(1000);                       // send data per 100ms
	++counter;
}


