/*
Copyright (c) 2011-2015 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// ENC28J60 ethernet phy interface

#include "../../config.h"

#ifdef ENC_PHY

#include "enc28j60_hw.h"
#include "enc28j60_net.h"

static MQ_t *   enc_in_msg = NULL;
static uint8_t  enc_exchange_buf[MAX_FRAME_BUF];

// Process MQTT-SN Packet
void phy_mqttsn_filter(uint16_t len, eth_frame_t * pFrame)
{
#ifdef LED_On
    LED_On();
#endif  //  LED_On

    enc_in_msg = mqAlloc(sizeof(MQ_t));
    ip_packet_t *ip = (void*)(pFrame->data);

    enc28j60_GetPacket((void*)enc_in_msg->m.raw, len);
    memcpy(enc_in_msg->a.phy1addr, ip->sender_ip, 4);
    enc_in_msg->Length = len;
}


//////////////////////////////////////////////////////////////////////
// PHY_API
void ENC28J60_Init(void)
{
    enc_in_msg = NULL;
    enc28j60_init_net();
}

void ENC28J60_Send(void *pBuf)
{
    if(!ENC28J60_System_Ready())
    {
        mqFree(pBuf);
        return;
    }

#ifdef LED_On
    LED_On();
#endif  //  LED_On

    eth_frame_t * pFrame = (void *)enc_exchange_buf;

    ip_packet_t *ip = (void*)(pFrame->data);
    udp_packet_t *udp = (void*)(ip->data);

    memcpy(ip->target_ip, (((MQ_t *)pBuf)->a.phy1addr), 4);
    udp->target_port = MQTTSN_UDP_PORT;
    udp->sender_port = MQTTSN_UDP_PORT;
    uint16_t len = ((MQ_t *)pBuf)->Length;
    memcpy((void*)(udp->data), (((MQ_t *)pBuf)->m.raw), len);

    mqFree(pBuf);
    udp_send(len, pFrame);
}

void * ENC28J60_Get(void)
{
#ifdef NET_WITH_DHCP
    dhcp_poll((eth_frame_t *)enc_exchange_buf);
#endif  //  NET_WITH_DHCP

    // Rx Section
    if(en28j60_DataRdy())
    {
        uint16_t len = enc28j60_GetPacketLen();
        if(len > sizeof(eth_frame_t))
        {
            eth_frame_t * pFrame = (void *)enc_exchange_buf;
            enc28j60_GetPacket((void *)pFrame,  sizeof(eth_frame_t));
            eth_filter(len, pFrame);
        }
        enc28j60_ClosePacket();
    }

    if(enc_in_msg != NULL)
    {
        MQ_t * retval = enc_in_msg;
        enc_in_msg = NULL;
        return retval;
    }

    return NULL;
}

/*
#ifdef LAN_NODE
static uint8_t cbWriteLANParm(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf);
static uint8_t cbReadLANParm(subidx_t *pSubidx, uint8_t *pLen, uint8_t *pBuf);
#endif  //  LAN_NODE

#ifdef LAN_NODE
    {{objEEMEM, objArray, eeMACAddr},
        objMACAddr, (cbRead_t)&cbReadLANParm, (cbWrite_t)&cbWriteLANParm, NULL},
    {{objEEMEM, objArray, eeIPAddr},
        objIPAddr, (cbRead_t)&cbReadLANParm, (cbWrite_t)&cbWriteLANParm, NULL},
    {{objEEMEM, objArray, eeIPMask},
        objIPMask, (cbRead_t)&cbReadLANParm, (cbWrite_t)&cbWriteLANParm, NULL},
    {{objEEMEM, objArray, eeIPRouter},
        objIPRouter, (cbRead_t)&cbReadLANParm, (cbWrite_t)&cbWriteLANParm, NULL},
    {{objEEMEM, objArray, eeIPBroker},
        objIPBroker, (cbRead_t)&cbReadLANParm, (cbWrite_t)&cbWriteLANParm, NULL},
#endif  //  LAN_NODE

#ifdef LAN_NODE
// Convert raw data from mqtt-sn packet to LAN variables
static uint8_t cbWriteLANParm(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf)
{
    uint16_t Base = pSubidx->Base;

    if(Base == eeMACAddr)
    {
        if(Len != 6)
        {
            return MQTTSN_RET_REJ_NOT_SUPP;
        }
    }
    else if(Len != 4)
    {
        return MQTTSN_RET_REJ_NOT_SUPP;
    }

    eeprom_write(pBuf, Base, Len);
    return MQTTSN_RET_ACCEPTED;
}

// Convert LAN variables to mqtt-sn packet
static uint8_t cbReadLANParm(subidx_t *pSubidx, uint8_t *pLen, uint8_t *pBuf)
{
    uint16_t Base = pSubidx->Base;

    if(Base == eeMACAddr)
    {
        *pLen = 6;
    }
    else
    {
        *pLen = 4;
    }

    eeprom_read(pBuf, Base, *pLen);
    return MQTTSN_RET_ACCEPTED;
}
#endif  //  LAN_NODE

#ifdef LAN_NODE
        uint32_t ulTmp = OD_DEF_IP_BROKER;
        WriteOD(objIPBroker, MQTTSN_FL_TOPICID_PREDEF, 4, (uint8_t *)&ulTmp);
        ulTmp = OD_DEF_IP_ROUTER;
        WriteOD(objIPRouter, MQTTSN_FL_TOPICID_PREDEF, 4, (uint8_t *)&ulTmp);
        ulTmp = OD_DEF_IP_MASK;
        WriteOD(objIPMask,   MQTTSN_FL_TOPICID_PREDEF, 4, (uint8_t *)&ulTmp);
        ulTmp = OD_DEF_IP_ADDR;
        WriteOD(objIPAddr,   MQTTSN_FL_TOPICID_PREDEF, 4, (uint8_t *)&ulTmp);
#ifndef OD_DEF_DEV_MAC
        uint8_t   defMAC[6];
        ulTmp = hal_GetDeviceID();
        defMAC[0] = 0x00;       // Microchip
        defMAC[1] = 0x04;
        defMAC[2] = 0xA3;
        defMAC[3] = (ulTmp >> 16) & 0xFF;
        defMAC[4] = (ulTmp >> 8) & 0xFF;
        defMAC[5] = ulTmp & 0xFF;
#else   //  OD_DEF_DEV_MAC
        uint8_t   defMAC[] = OD_DEF_DEV_MAC;
#endif  //  OD_DEF_DEV_MAC
        WriteOD(objMACAddr, MQTTSN_FL_TOPICID_PREDEF, 6, (uint8_t *)&defMAC);       // Default MAC
#endif  //  LAN_NODE


*/

/*
void * ENC28G60_GetCB(uint16_t index)
{
    switch(index & 0x0001F)
    {
        default:
            break;
    }

    return NULL;
}
*/
#endif  //  ENC_PHY
