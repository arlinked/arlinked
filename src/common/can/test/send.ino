// demo: CAN-BUS Shield, send data
#include "mcp2515_can.h"
#include <SPI.h>


can::CCanMcp2515Impl can10(10);                                      // Set CS to pin 10

extern "C" void setup()
{
  Serial.begin(9600);
  SPI.begin();

  // init can bus, baudrate: 500k
  if(can10.begin(CAN_125KBPS) == CAN_OK)
    Serial.print("can init ok!!\r\n");
  else
    Serial.print("Can init fail!!\r\n");
}

unsigned int counter = 0;

extern "C" void loop()
{
  char stmp[9];
  snprintf(stmp, 9, "%08d", counter);

  can::CCanMessage msg(8, (uint8*)stmp);

  // send data:  id = 0x00, standrad flame, data len = 8, stmp: data buf
  uint8 stat = can10.send(&msg);
  if(stat != CAN_OK)
  {
    Serial.print(counter);
    Serial.println(" failed to send");
  }
  else
  {
    Serial.print(counter);
    Serial.print("\tmsg: ");
    Serial.println((char*)msg.getData());
  }
  delay(1000);                       // send data per 100ms
  ++counter;
}


