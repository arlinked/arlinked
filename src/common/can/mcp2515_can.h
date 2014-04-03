/**
 * File: mcp2515_can.h implementation of mcp2515 can-bus
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

#ifndef __MCP2515_CAN_H__
#define __MCP2515_CAN_H__

#include "mcp2515_define.h"

#define MAX_CHAR_IN_MESSAGE 8

namespace can {

class CCan;

namespace inner {
class CCanMcp2515Impl;
}

extern uint8 CAN_DEBUG_LOG;

// can message
class CCanMessage {
friend class inner::CCanMcp2515Impl;
private:

  /* identifier xxxID either extended (the 29 LSB) or standard (the 11 LSB) */
  uint8 m_nExtFlg;

  /* can id */
  uint32 m_nId;

  /* data length */
  uint8 m_nDlc;

  /* data */
  uint8 m_nDta[MAX_CHAR_IN_MESSAGE + 1];

  /* rtr */
  uint8 m_nRtr;

  uint8 m_nfilhit;

public:
  CCanMessage() : m_nExtFlg(0), m_nId(0), m_nDlc(0), m_nRtr(0), m_nfilhit(0)
  {
    for(uint8 i = 0; i < MAX_CHAR_IN_MESSAGE + 1; ++i)
      m_nDta[i] = 0;
  }

  uint8 getExtFlag()
  {
    return m_nExtFlg;
  }

  /* get can id */
  uint32 getId()
  {
    return m_nId;
  }

  uint8 getLen()
  {
    return m_nDlc;
  }

  uint8* getData()
  {
    return m_nDta;
  }

  void setExtFlag(uint8 flag)
  {
    m_nExtFlg = flag;
  }

  void setId(uint32 id)
  {
    m_nId = id;
  }

  void setData(uint8 len, const uint8* data)
  {
    m_nDlc = len;
    for(uint8 i = 0; i < len && i < MAX_CHAR_IN_MESSAGE + 1; ++i)
      m_nDta[i] = data[i];
  }
};

class CCanStatus
{
public:
  uint8 getRecv0Interrupt() { return true == m_nStatus & MCP_STAT_RX0IF; }
  uint8 getRecv1Interrupt() { return true == m_nStatus & MCP_STAT_RX1IF; }
  uint8 getSend0Request()   { return true == m_nStatus & MCP_STAT_TX0REQ; }
  uint8 getSend0Interrupt() { return true == m_nStatus & MCP_STAT_TX0IF; }
  uint8 getSend1Request()   { return true == m_nStatus & MCP_STAT_TX1REQ; }
  uint8 getSend1Interrupt() { return true == m_nStatus & MCP_STAT_TX1IF; }
  uint8 getSend2Request()   { return true == m_nStatus & MCP_STAT_TX2REQ; }
  uint8 getSend2Interrupt() { return true == m_nStatus & MCP_STAT_TX2IF; }

public:
  CCanStatus(uint8 status) : m_nStatus(status) { }

private:
  uint8 m_nStatus;
};

namespace inner {

class CCanMcp2515Impl
{
private:
  uint8 m_nChipSelectPin;
public:

  /*
  * mcp2515 driver function
  */

  /* reset mcp2515 */
  void reset(void);

  /* read mcp2515's register */
  uint8 readRegister(const uint8 address);

  void readRegisterS(const uint8 address, uint8 values[], const uint8 n);

  /* set mcp2515's register */
  void setRegister(const uint8 address, const uint8 value);

  /* set mcp2515's registers */
  void setRegisterS(const uint8 address, const uint8 values[], const uint8 n);

  void initCANBuffers(void);

  /* set bit of one register */
  void modifyRegister(const uint8 address, const uint8 mask, const uint8 data);

  /* read mcp2515's Status */
  uint8 readStatus(void);

  /* set mode */
  uint8 setCanCtrlMode(const uint8 newmode);

  /* set boadrate */
  uint8 configRate(const uint8 canSpeed);

  /* mcp2515init */
  uint8 init(const uint8 canSpeed);

  /* write can id */
  void write_id( const uint8 mcp_addr, const uint8 ext, const uint32 id );

  /* read can id */
  void read_id( const uint8 mcp_addr, uint8* ext, uint32* id );

  /* write can msg */
  void write_canMsg(const uint8 buffer_sidh_addr, const CCanMessage* msg);

  /* read can msg */
  void read_canMsg(const uint8 buffer_sidh_addr, CCanMessage* msg);

  /* start transmit */
  void start_transmit(const uint8 mcp_addr);

  /* get Next free txbuf */
  uint8 getNextFreeTXBuf(uint8 *txbuf_n);

  /*********
  * can operator function
  */

  /* read message */
  uint8 readMsg(CCanMessage* msg);

  /* send message */
  uint8 sendMsg(const CCanMessage* msg);

  void reportState(uint8 addr);

public:
  CCanMcp2515Impl(uint8 _CS);

};

} // end of namespace inner

class CCan {

public:
  /* init can */
  uint8 begin(uint8 speedset);

  /* init Masks */
  uint8 initMask(uint8 num, uint8 ext, uint32 ulData);

  /* init filters */
  uint8 initFilt(uint8 num, uint8 ext, uint32 ulData);

  /* send buf */
  uint8 send(const CCanMessage *msg);

  /* read buf */
  uint8 recv(CCanMessage *msg);

  /* if something received */
  uint8 checkReceive(void);

  /* if something error */
  uint8 checkError(void);

  /* read status */
  uint8 checkStatus();

  uint8 sleep();

  uint8 wake();

public:
  CCan(uint8 chipSelectPin);

private:
  //! the implementation
  inner::CCanMcp2515Impl m_canImpl;
};

} // end of namespace can

#endif

