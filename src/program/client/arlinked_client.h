/**
 * File: client.h implementation of client unit
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

#ifndef __ARLINKED_CLIENT_H_
#define __ARLINKED_CLIENT_H_

#include "Arduino.h"
#include "../../common/can/mcp2515_can.h"

#define ARLINKED_CLIENT_FAIL -1
#define ARLINKED_CLIENT_OK   0
#define ARLINKED_CLIENT_IDLE 1

class CClient
{
public:
  uint8 init();
  uint8 run();
  uint8 sleep();
  uint8 wake();
  uint8 handleInterrupts();
public:
  CClient(uint8 nCanBusWakePin, uint8 nCanBusChipSelectPin)
    : m_nCanBusWakePin(nCanBusWakePin),
      m_nCanBusChipSelectPin(nCanBusChipSelectPin),
      m_canModule(nCanBusChipSelectPin)
  {
    pinMode(nCanBusWakePin, INPUT);
  }

private:
  uint8 m_nCanBusWakePin;
  uint8 m_nCanBusChipSelectPin;
  can::CCanMcp2515Impl m_canModule;
};

#endif

