/*
Copyright (c) 2011-2015 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _ENC28J60_PHY_H
#define _ENC28J60_PHY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//#define LAN_NODE

#if (ENC_PHY == 1)

#define PHY1_ADDR_t                 uint32_t
//#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)inet_addr(255,255,255,255)
//#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)inet_addr(255,255,255,255)

#define PHY1_Init                   ENC28J60_Init
#define PHY1_Ready                  ENC28J60_System_Ready
#define PHY1_Send                   ENC28J60_Send
#define PHY1_Get                    ENC28J60_Get

#define PHY1_ReadOD                 ENC28J60_ReadOD
#define PHY1_WriteOD                ENC28J60_WriteOD

#define PHY1_SIZEOF_CFG             24
#define ENC_EEP                     eePhy1

#else
#error ENC_PHY unknown inteface
#endif

#define inet_addr(d,c,b,a)          (((uint32_t)a<<24) | ((uint32_t)b << 16) | ((uint32_t)c<<8)  | ((uint32_t)d))
/*
#ifndef OD_DEF_IP_ADDR
#define OD_DEF_IP_ADDR      0xFFFFFFFF      // Default IP - use DHCP
#endif  //  OD_DEF_IP_ADDR
#ifndef OD_DEF_IP_MASK
#define OD_DEF_IP_MASK      0xFFFFFFFF      // Default IP Mask - use DHCP
#endif  //  OD_DEF_IP_MASK
#ifndef OD_DEF_IP_ROUTER
#define OD_DEF_IP_ROUTER    0xFFFFFFFF      // Default IP Gateway - use DHCP
#endif  //  OD_DEF_IP_ROUTER
#ifndef OD_DEF_IP_BROKER
#define OD_DEF_IP_BROKER    0xFFFFFFFF      // Default IP Broker - auto resolve
#endif  //  OD_DEF_IP_BROKER
//#ifndef OD_DEF_DEV_MAC
//#define OD_DEF_DEV_MAC      {0x00,0x04,0xA3,0x01,0x02,0x03}
//#endif  //  OD_DEF_DEV_MAC
*/

// API Section
void ENC28J60_Init(void);
bool ENC28J60_System_Ready(void);
void ENC28J60_Send(void *pBuf);
void * ENC28J60_Get(void);

uint8_t ENC28J60_ReadOD(uint16_t idx, uint8_t *pLen, uint8_t *pBuf);
uint8_t ENC28J60_WriteOD(uint16_t idx, uint8_t Len, uint8_t *pBuf);


#ifdef __cplusplus
}
#endif

#endif  //  _UART_PHY_H
