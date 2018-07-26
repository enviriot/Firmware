/*
Copyright (c) 2011-2015 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#include <string.h>

#include "config.h"
#include "hal.h"
#include "mqTypes.h"
#include "mqMEM.h"
#include "eep.h"

#if (defined ENC_PHY)

#include "enc28j60_hw.h"
#include "enc28j60_net.h"


#ifndef ENC28J60_DEFAULT_IP
#define ENC28J60_DEFAULT_IP     0xFFFFFFFF
#endif  // ENC28J60_DEFAULT_IP

#ifndef ENC28J60_DEFAULT_MASK
#define ENC28J60_DEFAULT_MASK   0xFFFFFFFF
#endif  // ENC28J60_DEFAULT_MASK

#ifndef ENC28J60_DEFAULT_GW
#define ENC28J60_DEFAULT_GW     0xFFFFFFFF
#endif  //  ENC28J60_DEFAULT_GW

#define phy_get             enc28j60_GetPacket
#define phy_skip            enc28j60_Skip

#define SUBNET_BROADCAST    (ip_addr | ~ip_mask)
#define NET_BROADCAST       0xFFFFFFFF
#define UNDEF_IP            0xFFFFFFFF

enum
{
    EncEepMac       = 0,        // 0  - 5
    EncEepIP        = 6,        // 6  - 9
    EncEepMask      = 10,       // 10 - 13
    EncEepGW        = 14,       // 14 - 17
    EncEepBroker    = 18,       // 18 - 21
    EncEepCRC       = 22        // 22 - 23
}eEncEep;

typedef enum
{
    DHCP_DISABLED,
    DHCP_ASSIGNED,
    DHCP_INIT,
    DHCP_WAITING_OFFER,
    DHCP_WAITING_ACK
} system_status_t;

// node MAC & IP addresses
static uint8_t          mac_addr[6];
static uint32_t         ip_addr;
static uint32_t         ip_mask;
static uint32_t         ip_gateway;
// ARP record
static uint32_t         arp_ip_addr;
static uint8_t          arp_mac_addr[6];

static system_status_t  system_status;

#ifdef NET_WITH_DHCP
static uint32_t         dhcp_retry_time;
static uint32_t         dhcp_transaction_id;
static uint32_t         dhcp_server;
#endif

static uint16_t ip_cksum(uint32_t sum, uint8_t *buf, size_t len);



// Initialize network variables
void enc28j60_init_net(void)
{
    uint8_t eep[PHY1_SIZEOF_CFG];

    eepReadRaw(ENC_EEP, PHY1_SIZEOF_CFG, eep);

    uint16_t eep_crc = eep[EncEepCRC + 1];
    eep_crc *= 256;
    eep_crc += eep[EncEepCRC];
    
    if(eep_crc != ip_cksum(0, eep, 22))
    {
#ifndef ENC28J60_DEFAULT_MAC
        uint32_t ulTmp = hal_GetDeviceID();
        mac_addr[0] = 0x00;       // Microchip
        mac_addr[1] = 0x04;
        mac_addr[2] = 0xA3;
        mac_addr[3] = (ulTmp >> 16) & 0xFF;
        mac_addr[4] = (ulTmp >> 8) & 0xFF;
        mac_addr[5] = ulTmp & 0xFF;
#else   //  ENC28J60_DEFAULT_MAC
        uint8_t   defMAC[] = ENC28J60_DEFAULT_MAC;
        memcpy(mac_addr, defMAC, 6);
#endif  //  ENC28J60_DEFAULT_MAC

        ip_addr = ENC28J60_DEFAULT_IP;
        ip_mask = ENC28J60_DEFAULT_MASK;
        ip_gateway = ENC28J60_DEFAULT_GW;

        uint32_t broker_ip = UNDEF_IP;

        memcpy(&eep[EncEepMac], mac_addr, sizeof(EncEepMac));
        memcpy(&eep[EncEepIP], &ip_addr, sizeof(ip_addr));
        memcpy(&eep[EncEepMask], &ip_mask, sizeof(ip_mask));
        memcpy(&eep[EncEepGW], &ip_gateway, sizeof(ip_gateway));
        memcpy(&eep[EncEepBroker], &broker_ip, sizeof(broker_ip));

        eep_crc = ip_cksum(0, eep, 22);
        eep[EncEepCRC] = eep_crc & 0xFF;
        eep[EncEepCRC + 1] = eep_crc >> 8;

        eepWriteRaw(ENC_EEP, PHY1_SIZEOF_CFG, eep);
    }
    else
    {
        memcpy(mac_addr, &eep[EncEepMac], sizeof(EncEepMac));
        memcpy(&ip_addr, &eep[EncEepIP], sizeof(ip_addr));
        memcpy(&ip_mask, &eep[EncEepMask], sizeof(ip_mask));
        memcpy(&ip_gateway, &eep[EncEepGW], sizeof(ip_gateway));
    }

    //initialize enc28j60 hardware
    enc28j60Init(mac_addr);

    // Init Variables
    arp_ip_addr = 0;

#ifdef NET_WITH_DHCP
    if(ip_addr != 0xFFFFFFFF)
    {
        system_status = DHCP_DISABLED;
    }
    else
    {
        ip_addr = 0;
        system_status = DHCP_INIT;
        dhcp_retry_time = HAL_get_sec() + 2;
    }
#endif
}

//////////////////////////////////////////////////////////////////////
// System API
bool ENC28J60_System_Ready(void)
{
    return (system_status < DHCP_INIT);
}

uint8_t ENC28J60_ReadOD(uint16_t idx, uint8_t *pLen, uint8_t *pBuf)
{
    switch(idx)
    {
//        case 0x00:      // Control
        case 0x01:      // IP Address
            *pLen = 4;
            eepReadRaw((ENC_EEP + EncEepIP), 4, pBuf);
            return MQTTSN_RET_ACCEPTED;

        case 0x02:      // IP Mask
            *pLen = 4;
            memcpy(pBuf, &ip_mask, 4);
            return MQTTSN_RET_ACCEPTED;

        case 0x03:      // IP Gateway
            *pLen = 4;
            memcpy(pBuf, &ip_gateway, 4);
            return MQTTSN_RET_ACCEPTED;

        case 0x04:      // IP Broker
            *pLen = 4;
            eepReadRaw((ENC_EEP + EncEepBroker), 4, pBuf);
            return MQTTSN_RET_ACCEPTED;

        case 0x0F:      // IP MAC
            *pLen = 6;
            memcpy(pBuf, mac_addr, 6);
            return MQTTSN_RET_ACCEPTED;

        case 0x18:      // Actual IP
            *pLen = 4;
            memcpy(pBuf, &ip_addr, 4);
            return MQTTSN_RET_ACCEPTED;

        case 0x19:      // Undefined ID
        case 0x1A:      // Broadcast ID
            *pLen = 4;
            pBuf[0] = 0xFF;
            pBuf[1] = 0xFF;
            pBuf[2] = 0xFF;
            pBuf[3] = 0xFF;
            return MQTTSN_RET_ACCEPTED;

        default:
            break;
    }
    return MQTTSN_RET_REJ_INV_ID;
}

uint8_t ENC28J60_WriteOD(uint16_t idx, uint8_t Len, uint8_t *pBuf)
{
    uint8_t offset;

    switch(idx)
    {
//        case 0x00:      // Control
        case 0x01:      // IP Address
            if(Len != 4)
            {
                return MQTTSN_RET_REJ_NOT_SUPP;
            }
            offset = EncEepIP;
            break;

        case 0x02:      // IP Mask
            if(Len != 4)
            {
                return MQTTSN_RET_REJ_NOT_SUPP;
            }
            offset = EncEepMask;
            break;

        case 0x03:      // IP Gateway
            if(Len != 4)
            {
                return MQTTSN_RET_REJ_NOT_SUPP;
            }
            offset = EncEepGW;
            break;

        case 0x04:      // IP Broker
            if(Len != 4)
            {
                return MQTTSN_RET_REJ_NOT_SUPP;
            }
            offset = EncEepBroker;
            break;

        case 0x0F:      // IP MAC
            if(Len != 6)
            {
                return MQTTSN_RET_REJ_NOT_SUPP;
            }
            offset = EncEepMac;
            break;

        default:
            return MQTTSN_RET_REJ_INV_ID;
    }

    uint8_t eep[PHY1_SIZEOF_CFG];
    eepReadRaw(ENC_EEP, PHY1_SIZEOF_CFG, eep);

    memcpy(&eep[offset], pBuf, Len);

    uint16_t  eep_crc = ip_cksum(0, eep, 22);
    eep[EncEepCRC] = eep_crc & 0xFF;
    eep[EncEepCRC + 1] = eep_crc >> 8;

    eepWriteRaw(ENC_EEP, PHY1_SIZEOF_CFG, eep);
    return MQTTSN_RET_ACCEPTED;
}

// End System API
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Ethernet Section

// send Ethernet frame
// fields must be set:
//  - frame.target_mac
//  - frame.type
static void eth_send(uint16_t len, eth_frame_t * pFrame)
{
    memcpy(pFrame->sender_mac, mac_addr, 6);
    enc28j60_SendPrep(sizeof(eth_frame_t) + len);
    enc28j60_PutData(sizeof(eth_frame_t) + len, (uint8_t *)pFrame);
    enc28j60_Send();
}

// send Ethernet frame back
static void eth_reply(uint16_t len, eth_frame_t * pFrame)
{
    memcpy(pFrame->target_mac, pFrame->sender_mac, 6);
    eth_send(len, pFrame);
}

static void arp_filter(uint16_t len, eth_frame_t * pFrame);
static void ip_filter(uint16_t len, eth_frame_t * pFrame);

// process Ethernet packet
void eth_filter(uint16_t len, eth_frame_t * pFrame)
{
    if(pFrame->type == ETH_TYPE_ARP)
        arp_filter(len - sizeof(eth_frame_t), pFrame);
    else if(pFrame->type == ETH_TYPE_IP)
        ip_filter(len - sizeof(eth_frame_t), pFrame);
}
// End Ethernet Section
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// ARP Section

// resolve MAC address
static void arp_resolve_req(uint32_t node_ip_addr, eth_frame_t * pFrame)
{
    arp_message_t *arp = (void*)(pFrame->data);
    
    uint8_t ucTmp;

    // send request
    for(ucTmp = 0; ucTmp < 6; ucTmp++) pFrame->target_mac[ucTmp] = 0xFF;
    pFrame->type = ETH_TYPE_ARP;
    arp->hw_type = ARP_HW_TYPE_ETH;
    arp->proto_type = ARP_PROTO_TYPE_IP;
    arp->hw_addr_len = 6;
    arp->proto_addr_len = 4;
    arp->opcode = ARP_TYPE_REQUEST;
    memcpy(arp->sender_mac, mac_addr, 6);
    memcpy(arp->sender_ip, &ip_addr, 4);
    for(ucTmp = 0; ucTmp < 6; ucTmp++) arp->target_mac[ucTmp] = 0;
    memcpy(arp->target_ip, &node_ip_addr, 4);

    eth_send(sizeof(arp_message_t), pFrame);
}

// process arp packet
static void arp_filter(uint16_t len, eth_frame_t * pFrame)
{
    if(len < sizeof(arp_message_t))
        return;

    arp_message_t *arp = (void *)pFrame->data;
  
    phy_get((void *)arp,  sizeof(arp_message_t));

    if((arp->hw_type != ARP_HW_TYPE_ETH) ||
       (arp->proto_type != ARP_PROTO_TYPE_IP) ||
       (memcmp(&arp->target_ip, &ip_addr, 4) != 0))
        return;

    if(arp->opcode == ARP_TYPE_REQUEST)
    {
        arp->opcode = ARP_TYPE_RESPONSE;
        memcpy(arp->target_mac, arp->sender_mac, 6);
        memcpy(arp->sender_mac, mac_addr, 6);
        memcpy(arp->target_ip, arp->sender_ip, 4);
        memcpy(arp->sender_ip, &ip_addr, 4);
        eth_reply(sizeof(arp_message_t), pFrame);
    }
    else if(arp->opcode == ARP_TYPE_RESPONSE)
    {
        memcpy(&arp_ip_addr, arp->sender_ip, 4);
        memcpy(arp_mac_addr, arp->sender_mac, 6);
    }
}
// End ARP Section
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// IP Section

// calculate IP checksum
static uint16_t ip_cksum(uint32_t sum, uint8_t *buf, size_t len)
{
    uint16_t uiTmp;

    while(len > 1)
    {
        uiTmp = ((uint16_t)*buf << 8) | *(buf+1);
        buf += 2;
        len -= 2;

        sum += uiTmp;
    }

    if(len)
        sum += (uint16_t)*buf << 8;

    while(sum >> 16)
        sum = (sum & 0xffff) + (sum >> 16);

    uiTmp = sum & 0xFFFF;
    uiTmp = htons(uiTmp);
    uiTmp = ~uiTmp;

    return uiTmp;
}

// send IP packet
// fields must be set:
//  - ip.dst
//  - ip.proto
// len is IP packet payload length
static void ip_send(uint16_t len, eth_frame_t *pFrame)
{
    ip_packet_t *ip = (void*)(pFrame->data);
  
    uint32_t t_ip;
    memcpy(&t_ip, ip->target_ip, 4);

    // apply route
    if((t_ip == SUBNET_BROADCAST) || (t_ip == NET_BROADCAST))
    {
        uint8_t ucTmp;
        for(ucTmp = 0; ucTmp < 6; ucTmp++) pFrame->target_mac[ucTmp] = 0xFF;
    }
    else
    {
        uint32_t route_ip;
    
        if(((t_ip ^ ip_addr) & ip_mask) == 0)
            route_ip = t_ip;
        else if(((ip_gateway ^ ip_addr) & ip_mask) == 0)
            route_ip = ip_gateway;
        else
            return;

        if(route_ip == arp_ip_addr)
            memcpy(pFrame->target_mac, arp_mac_addr, 6);
        else
        {
            arp_resolve_req(route_ip, pFrame);
            return;
        }
    }

    // send packet
    len += sizeof(ip_packet_t);
  
    pFrame->type = ETH_TYPE_IP;

    ip->ver_head_len = 0x45;
    ip->tos = 0;
    ip->total_len = htons(len);
    ip->fragment_id = 0;
    ip->flags_framgent_offset = 0;
    ip->ttl = IP_PACKET_TTL;
    ip->cksum = 0;
    memcpy(ip->sender_ip, &ip_addr, 4);
    ip->cksum = ip_cksum(0, (void*)ip, sizeof(ip_packet_t));

    eth_send(len, pFrame);
}

// send IP packet back
// len is IP packet payload length
static void ip_reply(uint16_t len, eth_frame_t * pFrame)
{
    ip_packet_t *ip = (void*)(pFrame->data);

    len += sizeof(ip_packet_t);

    ip->total_len = htons(len);
    ip->fragment_id = 0;
    ip->flags_framgent_offset = 0;
    ip->ttl = IP_PACKET_TTL;
    ip->cksum = 0;
    memcpy(ip->target_ip, ip->sender_ip, 4);
    memcpy(ip->sender_ip, &ip_addr, 4);
    ip->cksum = ip_cksum(0, (void*)ip, sizeof(ip_packet_t));

    eth_reply(len, pFrame);
}

static void icmp_filter(uint16_t len, eth_frame_t * pFrame);
static void udp_filter(uint16_t len, eth_frame_t * pFrame);

// process IP packet
static void ip_filter(uint16_t len, eth_frame_t * pFrame)
{
    if(len < sizeof(ip_packet_t))
        return;

    uint16_t hcs;
    ip_packet_t *ip = (void*)(pFrame->data);
  
    phy_get((void *)ip,  sizeof(ip_packet_t));

    hcs = ip->cksum;
    ip->cksum = 0;

    if((ip->ver_head_len != 0x45) ||    // Version 4, Header length 20 bytes
       (ip_cksum(0, (void*)ip, sizeof(ip_packet_t)) != hcs))
        return;

    len = htons(ip->total_len) - sizeof(ip_packet_t);
  
    uint32_t t_ip;
    memcpy(&t_ip, ip->target_ip, 4);

    switch(ip->protocol)
    {
#ifdef NET_WITH_ICMP
        case IP_PROTOCOL_ICMP:
            if(t_ip == ip_addr)
                icmp_filter(len, pFrame);
            break;
#endif  //  NET_WITH_ICMP
        case IP_PROTOCOL_UDP:
            if((t_ip == ip_addr) || (t_ip == SUBNET_BROADCAST) || (t_ip == NET_BROADCAST))
                udp_filter(len, pFrame);
            break;
    }
}
// End IP Section
//////////////////////////////////////////////////////////////////////

#ifdef NET_WITH_ICMP
//////////////////////////////////////////////////////////////////////
// ICMP Section
// process ICMP packet
static void icmp_filter(uint16_t len, eth_frame_t * pFrame)
{
    if((len < sizeof(icmp_echo_packet_t)) || (len > (MAX_FRAME_BUF - sizeof(ip_packet_t) - sizeof(eth_frame_t))))
        return;

    ip_packet_t *ip = (void*)pFrame->data;
    icmp_echo_packet_t *icmp = (void*)ip->data;

    phy_get((void *)icmp, sizeof(icmp_echo_packet_t));
  
    if(icmp->type == ICMP_TYPE_ECHO_RQ)
    {
        phy_get((void *)icmp->data,  len - sizeof(icmp_echo_packet_t));

        icmp->type = ICMP_TYPE_ECHO_RPLY;
        icmp->cksum += 8; // update cksum
        ip_reply(len, pFrame);
    }
}
// End ICMP Section
//////////////////////////////////////////////////////////////////////
#endif  //  NET_WITH_ICMP

//////////////////////////////////////////////////////////////////////
// UDP Section

// send UDP packet
// fields must be set:
//  - ip.target_ip
//  - udp.sender_port
//  - udp.target_port
// uint16_t len is UDP data payload length
void udp_send(uint16_t len, eth_frame_t *pFrame)
{
    ip_packet_t *ip = (void*)(pFrame->data);
    udp_packet_t *udp = (void*)(ip->data);

    len += sizeof(udp_packet_t);

    ip->protocol = IP_PROTOCOL_UDP;
    memcpy(ip->sender_ip, &ip_addr, 4);

    udp->len = htons(len);
    udp->cksum = 0;
    udp->cksum = ip_cksum(len + IP_PROTOCOL_UDP, (uint8_t*)udp - 8, len + 8);

    ip_send(len, pFrame);
}

static void dhcp_filter(uint16_t len, eth_frame_t *pFrame);

// process UDP packet
static void udp_filter(uint16_t len, eth_frame_t * pFrame)
{
    if(len < sizeof(udp_packet_t))
        return;

    ip_packet_t *ip = (void*)(pFrame->data);
    udp_packet_t *udp = (void*)(ip->data);

    phy_get((void *)udp, sizeof(udp_packet_t));

    len = htons(udp->len) - sizeof(udp_packet_t);

    if((udp->target_port == MQTTSN_UDP_PORT) && (len <= (sizeof(MQTTSN_MESSAGE_t) + 1)))
    {
        // Passive ARP Resolve
        if(arp_ip_addr == 0)
        {
            memcpy(&arp_ip_addr, ip->sender_ip, 4);
            memcpy(arp_mac_addr, pFrame->sender_mac, 6);
        }

        phy_mqttsn_filter(len, pFrame);
    }
#ifdef NET_WITH_DHCP
    else if(udp->target_port == DHCP_CLIENT_PORT)
    {
        dhcp_filter(len, pFrame);
    }
#endif  //  NET_WITH_DHCP
}
// End UDP Section
//////////////////////////////////////////////////////////////////////

#ifdef NET_WITH_DHCP
//////////////////////////////////////////////////////////////////////
// DHCP Section

static void dhcp_add_option(uint8_t ** pBuf, uint8_t optcode, uint8_t len, void * pVal)
{
    uint8_t *pOp;
    pOp = *pBuf;

    *(pOp++) = optcode;
    *(pOp++) = len;
    memcpy(pOp, pVal, len);
    pOp += len;
    // Optional padding
//    if(len & 1)
//        *(pOp++) = 0;
        
    *pBuf = pOp;
}

static void dhcp_send(uint16_t length, eth_frame_t *pFrame)
{
    ip_packet_t     * ip   = (void*)pFrame->data;
    udp_packet_t    * udp  = (void*)(ip->data);
    dhcp_message_t  * dhcp = (void*)(udp->data);
    uint8_t         * pOpt = (void*)(dhcp->options);
    
    // Make DHCP header
    dhcp->operation = DHCP_OP_REQUEST;
    dhcp->hw_addr_type = DHCP_HW_ADDR_TYPE_ETH;
    dhcp->hw_addr_len = 6;
    memcpy(dhcp->transaction_id, &dhcp_transaction_id, 4);
    dhcp->flags = DHCP_FLAG_BROADCAST;
    memcpy(dhcp->hw_addr, mac_addr, 6);

    //dhcp->magic_cookie = DHCP_MAGIC_COOKIE;
    *(pOpt++) = ((DHCP_MAGIC_COOKIE>>0) & 0xFF);
    *(pOpt++) = ((DHCP_MAGIC_COOKIE>>8) & 0xFF);
    *(pOpt++) = ((DHCP_MAGIC_COOKIE>>16) & 0xFF);
    *(pOpt++) = ((DHCP_MAGIC_COOKIE>>24) & 0xFF);

    // Make UDP header
    uint16_t len = length + (192 + sizeof(dhcp_message_t) + sizeof(udp_packet_t));

    udp->sender_port = DHCP_CLIENT_PORT;
    udp->target_port = DHCP_SERVER_PORT;
    udp->len = htons(len);
    udp->cksum = 0;

    memcpy(ip->sender_ip, &ip_addr, 4);
    uint8_t ucTmp;
    for(ucTmp = 0; ucTmp < 4; ucTmp++) ip->target_ip[ucTmp] = 0xFF;     //  NET_BROADCAST

    udp->cksum = ip_cksum(len + IP_PROTOCOL_UDP, (uint8_t*)udp - 8, length + (8 + sizeof(dhcp_message_t) + sizeof(udp_packet_t)));

    // Make IP Header
    len += sizeof(ip_packet_t);
    ip->ver_head_len = 0x45;
    ip->tos = 0;
    ip->total_len = htons(len);
    ip->fragment_id = 0;
    ip->flags_framgent_offset = 0;
    ip->ttl = IP_PACKET_TTL;
    ip->protocol = IP_PROTOCOL_UDP;
    ip->cksum = 0;
    ip->cksum = ip_cksum(0, (void*)ip, sizeof(ip_packet_t));

    // Make Ethernet header
    len += sizeof(eth_frame_t);
    for(ucTmp = 0; ucTmp < 6; ucTmp++) pFrame->target_mac[ucTmp] = 0xFF;
    memcpy(pFrame->sender_mac, mac_addr, 6);
    pFrame->type = ETH_TYPE_IP;

    // Send packet to PHY
    enc28j60_SendPrep(len);
    enc28j60_PutData((sizeof(eth_frame_t) +
                      sizeof(ip_packet_t) +
                      sizeof(udp_packet_t) +
                      sizeof(dhcp_message_t)),
                    (uint8_t *)pFrame);
    enc28j60_Fill(192);
    enc28j60_PutData(length, dhcp->options);
    enc28j60_Send();
}

static void dhcp_filter(uint16_t len, eth_frame_t *pFrame)
{
    if(len < SIZEOF_DHCP_MESSAGE)   // sizeof dhcp_message w/o options
        return;

    ip_packet_t     * ip   = (void*)pFrame->data;
    udp_packet_t    * udp  = (void*)(ip->data);
    dhcp_message_t  * dhcp = (void*)(udp->data);
    dhcp_option_t   * opt;

    uint8_t     ucTmp;
    uint32_t    ulTmp;
    
    // DHCP options variables
    uint8_t     type;
    uint32_t    offered_net_mask;
    uint32_t    offered_gateway;
    uint32_t    lease_time = 0;
    uint32_t    renew_time = 0;
    uint32_t    renew_server = 0;

    phy_get((void *)dhcp, sizeof(dhcp_message_t));
    phy_skip(192);
    phy_get((void *)&ulTmp, 4); // Get Magic Cookies

    // Check if DHCP messages directed to us
    if((dhcp->operation != DHCP_OP_REPLY) ||
       (memcmp(dhcp->transaction_id,  &dhcp_transaction_id, 4) != 0) ||
       (ulTmp != DHCP_MAGIC_COOKIE))
        return;

    len -= SIZEOF_DHCP_MESSAGE;

    // parse DHCP options
    while(len >= 2 ) // sizeof(dhcp_option_t)
    {
        opt = (void *)dhcp->options;
        phy_get((void *)opt, 2);

dhcp_filter_lbl1:
        switch(opt->code)
        {
            case DHCP_CODE_PAD:             // 0
                opt->code = opt->len;
                phy_get(&opt->len, 1);
                len--;
                if(len >= 2)
                {
                    goto dhcp_filter_lbl1;
                }
                opt->len = 0;
                len = 0;
                break;
            case DHCP_CODE_END:             // 255
                opt->len = 0;
                len = 0;
                break;
            case DHCP_CODE_MESSAGETYPE:     //  53
                phy_get(&type, 1);
                break;
            case DHCP_CODE_SUBNETMASK:      //  1
                phy_get((uint8_t *)&offered_net_mask, 4);
                break;
            case DHCP_CODE_ROUTER:          //  3
                phy_get((uint8_t *)&offered_gateway, 4);
                break;
            case DHCP_CODE_LEASETIME:       //  51
                phy_get((uint8_t *)&ulTmp, 4);
                ulTmp = ntohl(ulTmp);
                if(ulTmp > 21600)
                    ulTmp = 21600;
                lease_time = ulTmp;
                break;
            case DHCP_CODE_RENEWTIME:       //  58
                phy_get((uint8_t *)&ulTmp, 4);
                ulTmp = ntohl(ulTmp);
                if(ulTmp > 21600)
                    ulTmp = 21600;
                renew_time = ulTmp;
                break;
            case DHCP_CODE_DHCPSERVER:      //  54
                phy_get((uint8_t *)&renew_server, 4);
                break;
            default:                        // unknown
                phy_skip(opt->len);
                break;
        }
        len -= opt->len;
    }
    
    if(renew_server == 0)
        memcpy(&renew_server, ip->sender_ip, 4);

    if(type == DHCP_MESSAGE_OFFER)
    {
        memcpy(&ulTmp, dhcp->offered_addr, 4);
        if((system_status != DHCP_WAITING_OFFER) || (ulTmp == 0))
            return;

        system_status = DHCP_WAITING_ACK;

        uint8_t * pOpt = (void*)(dhcp->options);
        pOpt += 4;
        
        ucTmp = DHCP_MESSAGE_REQUEST;

        dhcp_add_option(&pOpt, DHCP_CODE_MESSAGETYPE, 1, &ucTmp);
        dhcp_add_option(&pOpt, DHCP_CODE_REQUESTEDADDR, 4, dhcp->offered_addr);
        dhcp_add_option(&pOpt, DHCP_CODE_DHCPSERVER, 4, &renew_server);
        // parameter request list
        *(pOpt++) = DHCP_CODE_REQUESTLIST;
        *(pOpt++) = 4;  // length
        *(pOpt++) = DHCP_CODE_SUBNETMASK;
        *(pOpt++) = DHCP_CODE_ROUTER;
        *(pOpt++) = DHCP_CODE_LEASETIME;
        *(pOpt++) = DHCP_CODE_RENEWTIME;
        // Last Parm.
        *(pOpt++) = DHCP_CODE_END;

        for(ucTmp = 0; ucTmp < 4; ucTmp++)
        {
            dhcp->offered_addr[ucTmp] = 0;
            dhcp->server_addr[ucTmp] = 0;
        }
        
        dhcp_send((uint8_t*)pOpt - (uint8_t*)(dhcp->options), pFrame);
    }
    else if(type == DHCP_MESSAGE_ACK)
    {
        if((system_status != DHCP_WAITING_ACK) || (lease_time == 0))
            return;

        if(renew_time == 0)
            renew_time = lease_time / 2;

        system_status = DHCP_ASSIGNED;
        dhcp_server = renew_server;
        dhcp_retry_time = HAL_get_sec() + renew_time;

        // network up
        memcpy(&ip_addr, dhcp->offered_addr, 4);
        //ip_addr = ntohl(ip_addr);
        ip_mask = offered_net_mask;
        ip_gateway = offered_gateway;
        
        enc28j60DisableBroadcast();
    }
}

void dhcp_poll(eth_frame_t * pFrame)
{
    if(system_status == DHCP_DISABLED)
        return;

    uint32_t tick_sec = HAL_get_sec();

#ifdef LED_On
    if((system_status != DHCP_ASSIGNED) && ((tick_sec & 0x00000001) != 0))
    {
        LED_On();
    }
#endif  //  LED_On

    if(dhcp_retry_time > tick_sec)
    {
        return;
    }

    if(enc28j60linkup() == false)
    {
        dhcp_retry_time = tick_sec + 2;
        return;
    }

    ip_packet_t     * ip;
    udp_packet_t    * udp;
    dhcp_message_t  * dhcp;

    uint16_t length;
    uint8_t ucTmp;

    dhcp_retry_time = tick_sec + 15;

    ip = (void*)(pFrame->data);
    udp = (void*)(ip->data);
    dhcp = (void*)(udp->data);

    for(ucTmp = 0; ucTmp < sizeof(dhcp_message_t); ucTmp++)   udp->data[ucTmp] = 0;
        
    dhcp_transaction_id = ((uint32_t)HAL_RNG()<<16) | HAL_RNG();

    uint8_t * pOpt = (void*)(dhcp->options);
    pOpt += 4;

    if(system_status != DHCP_ASSIGNED)
    {
        ucTmp = DHCP_MESSAGE_DISCOVER;
        dhcp_add_option(&pOpt, DHCP_CODE_MESSAGETYPE, 1, &ucTmp);
        system_status = DHCP_WAITING_OFFER;
    }
    else
    {
        ucTmp = DHCP_MESSAGE_REQUEST;
        dhcp_add_option(&pOpt, DHCP_CODE_MESSAGETYPE, 1, &ucTmp);
        dhcp_add_option(&pOpt, DHCP_CODE_REQUESTEDADDR, 4, &ip_addr);
        dhcp_add_option(&pOpt, DHCP_CODE_DHCPSERVER, 4, &dhcp_server);
        system_status = DHCP_WAITING_ACK;
    }

    *(pOpt++) = DHCP_CODE_END;

    length = (uint8_t*)pOpt - (uint8_t*)(dhcp->options);

    // Receive broadcast packets
    enc28j60EnableBroadcast();

    dhcp_send(length, pFrame);
}
// End DHCP Section
//////////////////////////////////////////////////////////////////////
#endif  //  NET_WITH_DHCP
#endif  //  ENC_PHY
