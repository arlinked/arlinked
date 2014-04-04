/**
 * File: mcp2515_can.cpp implementation of mcp2515 can-bus
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

#include "mcp2515_can.h"
#include <SPI.h>

#define spi_transfer SPI.transfer

#if CAN_DEBUG_MODE
#define CAN_DEBUG_LOG(...) do{ if(CAN_DEBUG_LOG_ENABLE){ Serial.println(__VA_ARGS__); } } while(0)
#else
#define CAN_DEBUG_LOG(...)
#endif

namespace can {

uint8 CAN_DEBUG_LOG_ENABLE = 1;

/*********************
** Function name:           reset
** Descriptions:            reset the device
*********************/
void CCanMcp2515Impl::reset(void)
{
  MCP2515_SELECT(m_nChipSelectPin);
  spi_transfer(MCP_RESET);
  MCP2515_UNSELECT(m_nChipSelectPin);
  delay(10);
}

/*********************
** Function name:           readRegister
** Descriptions:            read register
*********************/
uint8 CCanMcp2515Impl::readRegister(const uint8 address)
{
  uint8 ret;

  MCP2515_SELECT(m_nChipSelectPin);
  spi_transfer(MCP_READ);
  spi_transfer(address);
  ret = spi_transfer(0x0);
  MCP2515_UNSELECT(m_nChipSelectPin);

  return ret;
}

/*********************
** Function name:           readRegisterS
** Descriptions:            read registerS
*********************/
void CCanMcp2515Impl::readRegisterS(const uint8 address, uint8 values[], const uint8 n)
{
  uint8 i;
  MCP2515_SELECT(m_nChipSelectPin);
  spi_transfer(MCP_READ);
  spi_transfer(address);
  // mcp2515 has auto-increment of address-pointer
  for (i=0; i<n; i++)
  {
    values[i] = spi_transfer(0x0);
  }
  MCP2515_UNSELECT(m_nChipSelectPin);
}

/*********************
** Function name:           setRegister
** Descriptions:            set register
*********************/
void CCanMcp2515Impl::setRegister(const uint8 address, const uint8 value)
{
  MCP2515_SELECT(m_nChipSelectPin);
  spi_transfer(MCP_WRITE);
  spi_transfer(address);
  spi_transfer(value);
  MCP2515_UNSELECT(m_nChipSelectPin);
}

/*********************
** Function name:           setRegisterS
** Descriptions:            set registerS
*********************/
void CCanMcp2515Impl::setRegisterS(const uint8 address, const uint8 values[], const uint8 n)
{
  uint8 i;
  MCP2515_SELECT(m_nChipSelectPin);
  spi_transfer(MCP_WRITE);
  spi_transfer(address);

  for (i=0; i<n; i++)
  {
    spi_transfer(values[i]);
  }
  MCP2515_UNSELECT(m_nChipSelectPin);
}

/*********************
** Function name:           modifyRegister
** Descriptions:            set bit of one register
*********************/
void CCanMcp2515Impl::modifyRegister(const uint8 address, const uint8 mask, const uint8 data)
{
  MCP2515_SELECT(m_nChipSelectPin);
  spi_transfer(MCP_BITMOD);
  spi_transfer(address);
  spi_transfer(mask);
  spi_transfer(data);
  MCP2515_UNSELECT(m_nChipSelectPin);
}

/*********************
** Function name:           read_status
** Descriptions:            read mcp2515's Status
*********************/
uint8 CCanMcp2515Impl::read_status(void)
{
  uint8 i;
  MCP2515_SELECT(m_nChipSelectPin);
  spi_transfer(MCP_READ_STATUS);
  i = spi_transfer(0x0);
  MCP2515_UNSELECT(m_nChipSelectPin);

  return i;
}

uint8 CCanMcp2515Impl::read_RX_status(void)
{
  uint8 i;
  MCP2515_SELECT(m_nChipSelectPin);
  spi_transfer(MCP_SPI_CMD_RX_STATUS);
  i = spi_transfer(0x0);
  MCP2515_UNSELECT(m_nChipSelectPin);
  return i;
}

/*********************
** Function name:           setCanCtrlMode
** Descriptions:            set control mode
*********************/
uint8 CCanMcp2515Impl::setCanCtrlMode(const uint8 newmode)
{
  uint8 i;

  modifyRegister(MCP_CANCTRL, MCP_MODE_MASK, newmode);

  i = readRegister(MCP_CANCTRL);
  i &= MCP_MODE_MASK;

  if ( i == newmode )
  {
    return MCP2515_OK;
  }

  return MCP2515_FAIL;
}

/*********************
** Function name:           configRate
** Descriptions:            set boadrate
*********************/
uint8 CCanMcp2515Impl::configRate(const uint8 canSpeed)
{
  uint8 set, cfg1, cfg2, cfg3;
  set = 1;
  switch (canSpeed)
  {
    case (CAN_5KBPS):
    cfg1 = MCP_16MHz_5kBPS_CFG1;
    cfg2 = MCP_16MHz_5kBPS_CFG2;
    cfg3 = MCP_16MHz_5kBPS_CFG3;
    break;

    case (CAN_10KBPS):
    cfg1 = MCP_16MHz_10kBPS_CFG1;
    cfg2 = MCP_16MHz_10kBPS_CFG2;
    cfg3 = MCP_16MHz_10kBPS_CFG3;
    break;

    case (CAN_20KBPS):
    cfg1 = MCP_16MHz_20kBPS_CFG1;
    cfg2 = MCP_16MHz_20kBPS_CFG2;
    cfg3 = MCP_16MHz_20kBPS_CFG3;
    break;

    case (CAN_31K25BPS):
    cfg1 = MCP_16MHz_31k25BPS_CFG1;
    cfg2 = MCP_16MHz_31k25BPS_CFG2;
    cfg3 = MCP_16MHz_31k25BPS_CFG3;
    break;

    case (CAN_40KBPS):
    cfg1 = MCP_16MHz_40kBPS_CFG1;
    cfg2 = MCP_16MHz_40kBPS_CFG2;
    cfg3 = MCP_16MHz_40kBPS_CFG3;
    break;

    case (CAN_50KBPS):
    cfg1 = MCP_16MHz_50kBPS_CFG1;
    cfg2 = MCP_16MHz_50kBPS_CFG2;
    cfg3 = MCP_16MHz_50kBPS_CFG3;
    break;

    case (CAN_80KBPS):
    cfg1 = MCP_16MHz_80kBPS_CFG1;
    cfg2 = MCP_16MHz_80kBPS_CFG2;
    cfg3 = MCP_16MHz_80kBPS_CFG3;
    break;

    /* 100KBPS */
    case (CAN_100KBPS):
    cfg1 = MCP_16MHz_100kBPS_CFG1;
    cfg2 = MCP_16MHz_100kBPS_CFG2;
    cfg3 = MCP_16MHz_100kBPS_CFG3;
    break;

    case (CAN_125KBPS):
    cfg1 = MCP_16MHz_125kBPS_CFG1;
    cfg2 = MCP_16MHz_125kBPS_CFG2;
    cfg3 = MCP_16MHz_125kBPS_CFG3;
    break;

    case (CAN_200KBPS):
    cfg1 = MCP_16MHz_200kBPS_CFG1;
    cfg2 = MCP_16MHz_200kBPS_CFG2;
    cfg3 = MCP_16MHz_200kBPS_CFG3;
    break;

    case (CAN_250KBPS):
    cfg1 = MCP_16MHz_250kBPS_CFG1;
    cfg2 = MCP_16MHz_250kBPS_CFG2;
    cfg3 = MCP_16MHz_250kBPS_CFG3;
    break;

    case (CAN_500KBPS):
    cfg1 = MCP_16MHz_500kBPS_CFG1;
    cfg2 = MCP_16MHz_500kBPS_CFG2;
    cfg3 = MCP_16MHz_500kBPS_CFG3;
    break;

    case (CAN_1000KBPS):
    cfg1 = MCP_16MHz_1000kBPS_CFG1;
    cfg2 = MCP_16MHz_1000kBPS_CFG2;
    cfg3 = MCP_16MHz_1000kBPS_CFG3;
    break;

    default:
    set = 0;
    break;
  }

  if (set) {
    setRegister(MCP_CNF1, cfg1);
    setRegister(MCP_CNF2, cfg2);
    setRegister(MCP_CNF3, cfg3);
    return MCP2515_OK;
  }
  else {
    return MCP2515_FAIL;
  }
}

/*********************
** Function name:           initCANBuffers
** Descriptions:            init canbuffers
*********************/
void CCanMcp2515Impl::initCANBuffers(void)
{
  uint8 i, a1, a2, a3;

  uint8 std = 0;
  uint8 ext = 1;
  uint32 ulMask = 0x00, ulFilt = 0x00;


  /*Set both masks to 0 */
  /*Mask register ignores ext bit */
  write_id(MCP_RXM0SIDH, ext, ulMask);
  write_id(MCP_RXM1SIDH, ext, ulMask);

  /* Set all filters to 0 */
  /* RXB0: extended */
  /* RXB1: standard */
  /* RXB2: extended */
  /* RXB3: standard */
  write_id(MCP_RXF0SIDH, ext, ulFilt);
  write_id(MCP_RXF1SIDH, std, ulFilt);
  write_id(MCP_RXF2SIDH, ext, ulFilt);
  write_id(MCP_RXF3SIDH, std, ulFilt);
  write_id(MCP_RXF4SIDH, ext, ulFilt);
  write_id(MCP_RXF5SIDH, std, ulFilt);

  /* Clear, deactivate the three */
  /* transmit buffers */
  /* TXBnCTRL -> TXBnD7 */
  a1 = MCP_TXB0CTRL;
  a2 = MCP_TXB1CTRL;
  a3 = MCP_TXB2CTRL;

  /* in-buffer loop */
  for (i = 0; i < 14; i++) {
    setRegister(a1, 0);
    setRegister(a2, 0);
    setRegister(a3, 0);
    a1++;
    a2++;
    a3++;
  }
  setRegister(MCP_RXB0CTRL, 0x60);
  setRegister(MCP_RXB1CTRL, 0x60);
}

/*********************
** Function name:           init
** Descriptions:            init the device
*********************/
uint8 CCanMcp2515Impl::init(const uint8 canSpeed)
{

  uint8 res;

  reset();

  res = setCanCtrlMode(MCP_MODE_CONFIG);
  if(res > 0)
  {
    CAN_DEBUG_LOG("Enter setting mode fall");
    return res;
  }
  CAN_DEBUG_LOG("Enter setting mode success");

  /* set boadrate */
  if(configRate(canSpeed))
  {
    CAN_DEBUG_LOG("set rate fall!!");
    return res;
  }
  CAN_DEBUG_LOG("set rate success!!");

  if ( res == MCP2515_OK )
  {

  /* init canbuffers */
  initCANBuffers();

  /* interrupt mode */
  setRegister(MCP_CANINTE, MCP_INT_BIT_RX0 | MCP_INT_BIT_RX0 | MCP_INT_BIT_ERR | MCP_INT_BIT_WAK | MCP_INT_BIT_MERR);

#if (DEBUG_RXANY==1)
  modifyRegister(MCP_RXB0CTRL,
      MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
      MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
  modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
      MCP_RXB_RX_ANY);
#else
  modifyRegister(MCP_RXB0CTRL,
      MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
      MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
  modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
    MCP_RXB_RX_STDEXT);
#endif

  /* enter normal mode */
  res = setCanCtrlMode(MCP_MODE_NORMAL);
  if(res)
  {
    CAN_DEBUG_LOG("Enter Normal Mode Fall!!");
    return res;
  }

  CAN_DEBUG_LOG("Enter Normal Mode Success!!");
  }
  return res;
}

/*********************
** Function name:           write_id
** Descriptions:            write can id
*********************/
void CCanMcp2515Impl::write_id( const uint8 mcp_addr, const uint8 ext, const uint32 id )
{
  uint16_t canid;
  uint8 tbufdata[4];

  canid = (uint16_t)(id & 0x0FFFF);

  if ( ext == 1)
  {
    tbufdata[MCP_EID0] = (uint8) (canid & 0xFF);
    tbufdata[MCP_EID8] = (uint8) (canid >> 8);
    canid = (uint16_t)(id >> 16);
    tbufdata[MCP_SIDL] = (uint8) (canid & 0x03);
    tbufdata[MCP_SIDL] += (uint8) ((canid & 0x1C) << 3);
    tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
    tbufdata[MCP_SIDH] = (uint8) (canid >> 5 );
  }
  else
  {
    tbufdata[MCP_SIDH] = (uint8) (canid >> 3 );
    tbufdata[MCP_SIDL] = (uint8) ((canid & 0x07 ) << 5);
    tbufdata[MCP_EID0] = 0;
    tbufdata[MCP_EID8] = 0;
  }
  setRegisterS( mcp_addr, tbufdata, 4 );
}

/*********************
** Function name:           read_id
** Descriptions:            read can id
*********************/
void CCanMcp2515Impl::read_id( const uint8 mcp_addr, uint8* ext, uint32* id )
{
  uint8 tbufdata[4];

  *ext = 0;
  *id = 0;

  readRegisterS( mcp_addr, tbufdata, 4 );

  *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

  if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) == MCP_TXB_EXIDE_M )
  {
    /* extended id */
    *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
    *id = (*id<<8) + tbufdata[MCP_EID8];
    *id = (*id<<8) + tbufdata[MCP_EID0];
    *ext = 1;
  }
}

/*********************
** Function name:           write_canMsg
** Descriptions:            write msg
*********************/
void CCanMcp2515Impl::write_canMsg(const uint8 buffer_sidh_addr, const CCanMessage* msg)
{
  uint8 mcp_addr;
  uint8 nDlc;
  mcp_addr = buffer_sidh_addr;
  /* write data bytes */
  setRegisterS(mcp_addr+5, msg->m_nDta, msg->m_nDlc);
  /* if RTR set bit in byte */
  if(msg->m_nRtr == 1)
  {
    nDlc = msg->m_nDlc | MCP_RTR_MASK;
  }
  /* write the RTR and DLC */
  setRegister((mcp_addr+4), nDlc);
}

/*********************
** Function name:           read_canMsg
** Descriptions:            read message
*********************/
void CCanMcp2515Impl::read_canMsg(const uint8 buffer_sidh_addr, CCanMessage* msg)
{
  uint8 mcp_addr, ctrl;

  mcp_addr = buffer_sidh_addr;

  read_id(mcp_addr, &(msg->m_nExtFlg), &(msg->m_nId));

  ctrl = readRegister(mcp_addr-1);
  msg->m_nDlc = readRegister(mcp_addr+4);

  if ((ctrl & 0x08))
  {
    msg->m_nRtr = 1;
  }
  else
  {
    msg->m_nRtr = 0;
  }

  msg->m_nDlc &= MCP_DLC_MASK;
  readRegisterS(mcp_addr+5, &(msg->m_nDta[0]), msg->m_nDlc);
}

void CCanMcp2515Impl::read_from_RXB0_and_clear_RX0IF(CCanMessage* msg)
{
  read_from_RXBn_and_clear_RXnIF(0b00000000, msg);
}
void CCanMcp2515Impl::read_from_RXB1_and_clear_RX1IF(CCanMessage* msg)
{
  read_from_RXBn_and_clear_RXnIF(0b00000100, msg);
}
void CCanMcp2515Impl::read_from_RXBn_and_clear_RXnIF(const uint8 addr_mask, CCanMessage* msg)
{
  uint8 spi_cmd = 0b10010000 | (0b00000100 & addr_mask);
  MCP2515_SELECT(m_nChipSelectPin);

  spi_transfer(spi_cmd);
  uint8 nRXBnSIDH = spi_transfer(0x0);
  uint8 nRXBnSIDL = spi_transfer(0x0);
  uint8 nRXBnEID8 = spi_transfer(0x0);
  uint8 nRXBnEID0 = spi_transfer(0x0);
  uint8 nRXBnDLC  = spi_transfer(0x0);

  uint8 nLen = nRXBnDLC & 0b00001111;

  uint8 data[8];
  for(uint8 i = 0; i < nLen; ++i)
    msg->m_nDta[i]= spi_transfer(0x0);

  // release chip selection, this will clear the RXnIF
  MCP2515_UNSELECT(m_nChipSelectPin);

  uint32 id = (((uint32)nRXBnSIDH)<<3) | (((uint32)nRXBnSIDL)>>5);

  if((nRXBnSIDL & MCP_TXB_EXIDE_M) == MCP_TXB_EXIDE_M)
  {
    /* extended id */
    id <<= 18;
    id |= (nRXBnSIDL & 0x03) << 16;
    id |= nRXBnEID8 << 8;
    id |= nRXBnEID0;
    msg->m_nExtFlg = 1;
  }
  msg->m_nId = id;
  msg->m_nDlc = nLen;
  msg->m_nRtr = nRXBnDLC & 0x40;
}

/*********************
** Function name:           sendMsg
** Descriptions:            send message
*********************/
void CCanMcp2515Impl::start_transmit(const uint8 mcp_addr)
{
  modifyRegister( mcp_addr-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
}

/*********************
** Function name:           sendMsg
** Descriptions:            send message
*********************/
uint8 CCanMcp2515Impl::getNextFreeTXBuf(uint8 *txbuf_n)
{
  uint8 res, i, ctrlval;
  uint8 ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

  res = MCP_ALLTXBUSY;
  *txbuf_n = 0x00;

  /* check all 3 TX-Buffers */
  for (i=0; i<MCP_N_TXBUFFERS; i++)
  {
    ctrlval = readRegister( ctrlregs[i] );
    if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 )
    {
      /* return SIDH-address of Buffer */
      *txbuf_n = ctrlregs[i]+1;
      res = MCP2515_OK;
      /* ! function exit */
      return res;
    }
  }
  return res;
}

/*********************
** Function name:           set CS
** Descriptions:            init CS pin and set UNSELECTED
*********************/
CCanMcp2515Impl::CCanMcp2515Impl(uint8 _CS)
{
  m_nChipSelectPin = _CS;
  pinMode(m_nChipSelectPin, OUTPUT);
  MCP2515_UNSELECT(m_nChipSelectPin);
}

void CCanMcp2515Impl::reportState(uint8 addr)
{
  char tmp[32];
  uint8 ret;
  MCP2515_SELECT(m_nChipSelectPin);
  spi_transfer(MCP_READ);
  spi_transfer(addr);
  ret = spi_transfer(0x0);
  MCP2515_UNSELECT(m_nChipSelectPin);
  sprintf(tmp, "addr 0x%x stat:0x%x", addr, ret);
  CAN_DEBUG_LOG((char*)tmp);
}

/*********************
** Function name:           init_Mask
** Descriptions:            init canid Masks
*********************/
uint8 CCanMcp2515Impl::initMask(uint8 num, uint8 ext, uint32 ulData)
{
  uint8 res = MCP2515_OK;
  CAN_DEBUG_LOG("Begin to set Mask!!\r\n");
  res = setCanCtrlMode(MCP_MODE_CONFIG);
  if(res > 0)
  {
    CAN_DEBUG_LOG("Enter setting mode fall\r\n");
    return res;
  }

  if (num == 0)
  {
    write_id(MCP_RXM0SIDH, ext, ulData);
  }
  else if(num == 1)
  {
    write_id(MCP_RXM1SIDH, ext, ulData);
  }
  else
  {
    res =  MCP2515_FAIL;
  }

  res = setCanCtrlMode(MCP_MODE_NORMAL);
  if(res > 0)
  {
    CAN_DEBUG_LOG("Enter normal mode fall\r\n");
    return res;
  }
  CAN_DEBUG_LOG("set Mask success!!\r\n");
  return res;
}

/*********************
** Function name:           init_Filt
** Descriptions:            init canid filters
*********************/
uint8 CCanMcp2515Impl::initFilt(uint8 num, uint8 ext, uint32 ulData)
{
  uint8 res = MCP2515_OK;
  CAN_DEBUG_LOG("Begin to set Filter!!\r\n");
  res = setCanCtrlMode(MCP_MODE_CONFIG);
  if(res > 0)
  {
    CAN_DEBUG_LOG("Enter setting mode fall\r\n");
    return res;
  }

  switch( num )
  {
    case 0:
    write_id(MCP_RXF0SIDH, ext, ulData);
    break;

    case 1:
    write_id(MCP_RXF1SIDH, ext, ulData);
    break;

    case 2:
    write_id(MCP_RXF2SIDH, ext, ulData);
    break;

    case 3:
    write_id(MCP_RXF3SIDH, ext, ulData);
    break;

    case 4:
    write_id(MCP_RXF4SIDH, ext, ulData);
    break;

    case 5:
    write_id(MCP_RXF5SIDH, ext, ulData);
    break;

    default:
    res = MCP2515_FAIL;
  }

  res = setCanCtrlMode(MCP_MODE_NORMAL);
  if(res > 0)
  {
    CAN_DEBUG_LOG("Enter normal mode fall\r\nSet filter fail!!\r\n");
    return res;
  }
  CAN_DEBUG_LOG("set Filter success!!\r\n");

  return res;
}

/*********************
** Function name:           sendMsg
** Descriptions:            send message
*********************/
uint8 CCanMcp2515Impl::sendMsg(const CCanMessage* msg)
{
  uint8 res, res1, txbuf_n;
  uint16_t uiTimeOut = 0;

  do {
    /* info = addr. */
    res = getNextFreeTXBuf(&txbuf_n);
    uiTimeOut++;
  } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

  if(uiTimeOut == TIMEOUTVALUE)
  {
    /* get tx buff time out */
    return CAN_GETTXBFTIMEOUT;
  }
  uiTimeOut = 0;
  write_canMsg(txbuf_n, msg);
  start_transmit(txbuf_n);
  do
  {
    uiTimeOut++;
    /* read send buff ctrl reg   */
    res1= readRegister(txbuf_n);
    res1 = res1 & 0x08;
  }while(res1 && (uiTimeOut < TIMEOUTVALUE));
  /* send msg timeout */
  if(uiTimeOut == TIMEOUTVALUE)
  {
    return CAN_SENDMSGTIMEOUT;
  }
  return CAN_OK;
}

/*********************
** Function name:           init
** Descriptions:            init can and set speed
*********************/
uint8 CCanMcp2515Impl::begin(uint8 speedset)
{
  uint8 res;
  res = init(speedset);
  if(res == MCP2515_OK)
    return CAN_OK;
  else
    return CAN_FAILINIT;
}


/*********************
** Function name:           sendMsgBuf
** Descriptions:            send buf
*********************/
uint8 CCanMcp2515Impl::send(const CCanMessage* msg)
{
  return sendMsg(msg);
}

uint8 CCanMcp2515Impl::recv(CCanMessage* msg0, CCanMessage* msg1)
{
  uint8 stat;

  stat = read_RX_status();

  /* Msg in Buffer 0 */
  if(stat & MCP_SPI_DAT_RX_STATUS_BIT_RX0)
    read_from_RXB0_and_clear_RX0IF(msg0);
  /* Msg in Buffer 1 */
  if(stat & MCP_SPI_DAT_RX_STATUS_BIT_RX1)
    read_from_RXB1_and_clear_RX1IF(msg1);

  return stat & ( MCP_SPI_DAT_RX_STATUS_BIT_RX0 | MCP_SPI_DAT_RX_STATUS_BIT_RX1);
}

uint8 CCanMcp2515Impl::recv0(CCanMessage* msg)
{
  read_from_RXB0_and_clear_RX0IF(msg);
  return CAN_OK;
}

uint8 CCanMcp2515Impl::recv1(CCanMessage* msg)
{
  read_from_RXB1_and_clear_RX1IF(msg);
  return CAN_OK;
}

/*********************
** Function name:           checkReceive
** Descriptions:            check if got something
*********************/
uint8 CCanMcp2515Impl::checkReceive(void)
{
  uint8 res;
  /* RXnIF in Bit 1 and 0 */
  res = read_status();
  if ( res & MCP_STAT_RXIF_MASK )
  {
    return CAN_MSGAVAIL;
  }
  else
  {
    return CAN_NOMSG;
  }
}

/*********************
** Function name:           checkError
** Descriptions:            if something error
*********************/
uint8 CCanMcp2515Impl::checkError(void)
{
  uint8 eflg = readRegister(MCP_EFLG);

  if ( eflg & MCP_EFLG_ERRORMASK )
  {
    return CAN_CTRLERROR;
  }
  else
  {
    return CAN_OK;
  }
}

uint8 CCanMcp2515Impl::checkStatus()
{
  return read_status();
}

uint8 CCanMcp2515Impl::checkInterrupt()
{
  return readRegister(MCP_CANINTF);
}

void CCanMcp2515Impl::clearInterrupt(uint8 clearMask)
{
  modifyRegister(MCP_CANINTF, clearMask, 0x0);
}

uint8 CCanMcp2515Impl::sleep()
{
  modifyRegister(MCP_CANCTRL, MCP_MODE_MASK, MCP_MODE_SLEEP);

  uint8 mode = 0;
  uint8 count = 0;
  do
  {
    mode = readRegister(MCP_CANCTRL) & MCP_MODE_MASK;
    if(mode == MCP_MODE_SLEEP)
      break;
    delay(10);
  } while(++count < 50);

  if(mode == MCP_MODE_SLEEP)
    return CAN_OK;
  else
    return CAN_CTRLERROR;
}

uint8 CCanMcp2515Impl::wake()
{
  modifyRegister(MCP_CANCTRL, MCP_MODE_MASK, MCP_MODE_NORMAL);

  uint8 mode = 0;
  uint8 count = 0;
  do
  {
    mode = readRegister(MCP_CANCTRL) & MCP_MODE_MASK;
    if(mode == MCP_MODE_NORMAL)
      break;
    delay(10);
  } while(++count < 50);

  if(mode == MCP_MODE_NORMAL)
    return CAN_OK;
  else
    return CAN_CTRLERROR;
}

} // end of namespace can

