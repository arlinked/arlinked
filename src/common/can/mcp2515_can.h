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

extern uint8 CAN_DEBUG_LOG;

class CCanMcp2515Impl;

// can message
class CCanMessage {
friend class CCanMcp2515Impl;
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
  CCanMessage() : m_nExtFlg(0), m_nId(0b11111111111), m_nDlc(0), m_nRtr(0), m_nfilhit(0)
  {
    for(uint8 i = 0; i < MAX_CHAR_IN_MESSAGE + 1; ++i)
      m_nDta[i] = 0;
  }

  CCanMessage(uint8 nExtFlg, uint8 nId, uint8 nLen, const uint8* nData, uint8 nRtr = 0, uint8 nfilhit = 0)
    : m_nExtFlg(nExtFlg), m_nId(nId), m_nDlc(nLen), m_nRtr(nRtr), m_nfilhit(nfilhit)
  {
    if(m_nDlc > 8) m_nDlc = 8;
    for(uint8 i = 0; i < m_nDlc && i < MAX_CHAR_IN_MESSAGE + 1; ++i)
      m_nDta[i] = nData[i];
    m_nDta[MAX_CHAR_IN_MESSAGE] = '\0';
  }

  CCanMessage(uint8 nLen, const uint8* nData)
    : m_nExtFlg(0), m_nId(0b11111111111), m_nDlc(nLen), m_nRtr(0), m_nfilhit(0)
  {
    if(m_nDlc > 8) m_nDlc = 8;
    for(uint8 i = 0; i < m_nDlc && i < MAX_CHAR_IN_MESSAGE + 1; ++i)
      m_nDta[i] = nData[i];
    m_nDta[MAX_CHAR_IN_MESSAGE] = '\0';
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
  uint8 recv0Interrupt() const { return true == m_nStatus & MCP_STAT_RX0IF; }
  uint8 recv1Interrupt() const { return true == m_nStatus & MCP_STAT_RX1IF; }
  uint8 send0Request()   const { return true == m_nStatus & MCP_STAT_TX0REQ; }
  uint8 send0Interrupt() const { return true == m_nStatus & MCP_STAT_TX0IF; }
  uint8 send1Request()   const { return true == m_nStatus & MCP_STAT_TX1REQ; }
  uint8 send1Interrupt() const { return true == m_nStatus & MCP_STAT_TX1IF; }
  uint8 send2Request()   const { return true == m_nStatus & MCP_STAT_TX2REQ; }
  uint8 send2Interrupt() const { return true == m_nStatus & MCP_STAT_TX2IF; }
  uint8 value() const { return m_nStatus; }

public:
  CCanStatus(uint8 status) : m_nStatus(status) { }
  CCanStatus& operator=(uint8 val) { m_nStatus = val; }

private:
  uint8 m_nStatus;
};

class CCanRecvStatus
{
public:
/*
  uint8 recv0Interrupt() const { return true == m_nStatus & MCP_STAT_RX0IF; }
  uint8 recv1Interrupt() const { return true == m_nStatus & MCP_STAT_RX1IF; }
  uint8 send0Request()   const { return true == m_nStatus & MCP_STAT_TX0REQ; }
  uint8 send0Interrupt() const { return true == m_nStatus & MCP_STAT_TX0IF; }
  uint8 send1Request()   const { return true == m_nStatus & MCP_STAT_TX1REQ; }
  uint8 send1Interrupt() const { return true == m_nStatus & MCP_STAT_TX1IF; }
  uint8 send2Request()   const { return true == m_nStatus & MCP_STAT_TX2REQ; }
  uint8 send2Interrupt() const { return true == m_nStatus & MCP_STAT_TX2IF; }
  uint8 value() const { return m_nStatus; }
*/

public:
  CCanRecvStatus(uint8 status) : m_nStatus(status) { }
  CCanRecvStatus& operator=(uint8 val) { m_nStatus = val; }

private:
  uint8 m_nStatus;
};

class CCanInterruptStatus
{
public:
  uint8 recv0Interrupt()   const { return true == m_nStatus & MCP_INT_BIT_RX0; }
  uint8 recv1Interrupt()   const { return true == m_nStatus & MCP_INT_BIT_RX1; }
  uint8 send0Interrupt()   const { return true == m_nStatus & MCP_INT_BIT_TX0; }
  uint8 send1Interrupt()   const { return true == m_nStatus & MCP_INT_BIT_TX1; }
  uint8 send2Interrupt()   const { return true == m_nStatus & MCP_INT_BIT_TX2; }
  uint8 errorInterrupt()   const { return true == m_nStatus & MCP_INT_BIT_ERR; }
  uint8 wakeInterrupt()    const { return true == m_nStatus & MCP_INT_BIT_WAK; }
  uint8 msgErrInterrupt()  const { return true == m_nStatus & MCP_INT_BIT_MERR; }
  uint8 value() const { return m_nStatus; }

public:
  CCanInterruptStatus(uint8 status) : m_nStatus(status) { }
  CCanInterruptStatus& operator=(uint8 val) { m_nStatus = val; }

private:
  uint8 m_nStatus;
};

class CCanMcp2515Impl
{
private:

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
  uint8 read_status(void);
  uint8 read_RX_status(void);

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

  void read_from_RXB0_and_clear_RX0IF(CCanMessage* msg);
  void read_from_RXB1_and_clear_RX1IF(CCanMessage* msg);
  void read_from_RXBn_and_clear_RXnIF(const uint8 addr_mask, CCanMessage* msg);

  /* start transmit */
  void start_transmit(const uint8 mcp_addr);

  /* get Next free txbuf */
  uint8 getNextFreeTXBuf(uint8 *txbuf_n);

  /*********
  * can operator function
  */

  /* send message */
  uint8 sendMsg(const CCanMessage* msg);

  void reportState(uint8 addr);

public:
  CCanMcp2515Impl(uint8 _CS);

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
  uint8 recv(CCanMessage *msg0, CCanMessage* msg1);
  uint8 recv0(CCanMessage *msg);
  uint8 recv1(CCanMessage *msg);

  /* if something received */
  uint8 checkReceive(void);

  /* if something error */
  uint8 checkError(void);

  /* check status */
  uint8 checkStatus();

  /* check interrupt status */
  uint8 checkInterrupt();
  void clearInterrupt(uint8 clearMask);

  uint8 sleep();

  uint8 wake();

private:
  uint8 m_nChipSelectPin;

};

} // end of namespace can

#endif

