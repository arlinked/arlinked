/**
 * File: client.cpp implementation of client unit
 *
 * Copyright (C) 2014 Arlinked Inc. All right reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 **/

#include "arlinked_client.h"
#include "../../common/can/mcp2515_can.h"

uint8 CClient::init()
{
  if(CAN_OK == m_canModule.begin(CAN_125KBPS))
  {
    Serial.print("Can init ok!!\r\n");
  }
  else
  {
    Serial.print("Can init fail!!\r\n");
    return ARLINKED_CLIENT_FAIL;
  }
  return ARLINKED_CLIENT_OK;
}

uint8 CClient::run()
{
  uint8 idle = 1;
  uint8 nRecvStatus = 0;
  uint8 nInterruptStatus = 0;
  can::CCanMessage msg0, msg1;

  do
  {
    nRecvStatus = m_canModule.recv(&msg0, &msg1);
    if(nRecvStatus)
      idle = 0;

    if(MCP_SPI_DAT_RX_STATUS_BIT_RX0 & nRecvStatus)
    {
      Serial.print("Received message RXB0: "); // classic dummy message
      Serial.println((char*)msg0.getData());
    }

    if(MCP_SPI_DAT_RX_STATUS_BIT_RX1 & nRecvStatus)
    {
      Serial.print("Received message RXB1: "); // classic dummy message
      Serial.println((char*)msg1.getData());
    }
  } while (nRecvStatus);

  handleInterrupts();

  if(idle)
    return ARLINKED_CLIENT_IDLE;
  else
    return ARLINKED_CLIENT_OK;
}

uint8 CClient::handleInterrupts()
{
  uint8 interrupt = m_canModule.checkInterrupt();
  if(interrupt)
  {
    Serial.print("Interrupt: ");                // classic dummy message
    Serial.println(interrupt, HEX);

    if(interrupt & MCP_INT_BIT_ERR)             // Error interrupt
    {
    }

    if(interrupt & MCP_INT_BIT_WAK)             // Wake interrupt
    {
    }

    if(interrupt & MCP_INT_BIT_MERR)            // Message error interrupt
    {
    }

    m_canModule.clearInterrupt(0b11100000);     // Message error interrupt
  }
  return ARLINKED_CLIENT_OK;
}

uint8 CClient::sleep()
{
  return ARLINKED_CLIENT_OK;
}

uint8 CClient::wake()
{
  return ARLINKED_CLIENT_OK;
}

