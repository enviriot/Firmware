/*
Copyright (c) 2011-2018 <firmware@enviriot.com>

This file is part of the Enviriot project.
https://enviriot.github.io/
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#include <stdlib.h>

#include "config.h"

#include "hal.h"
#include "mqTypes.h"
#include "ObjDict.h"



/*
// Build Name
static uint8_t mqttsn_build_node_name(uint8_t * pBuf)
{
    uint8_t Length = MQTTSN_SIZEOF_CLIENTID;
    OD_Read(objNodeName, MQTTSN_FL_TOPICID_PREDEF, &Length, pBuf);
    if(Length > 1)
    {
        return Length;
    }

    // Node Name not defined, use default name
    uint8_t pos;
    Length = (MQTTSN_SIZEOF_CLIENTID - sizeof(PHY1_ADDR_t) - 1);
    OD_Read(objDeviceTyp, MQTTSN_FL_TOPICID_PREDEF, &Length, pBuf);
    // Search Delimiter
    for(pos = 0; (pos < Length) && (pBuf[pos] != '.'); pos++);
    Length = pos;
    pBuf += Length;
    *(pBuf++) = '_';
    Length++;

    //uint8_t * pAddr = PHY1_GetAddr();

    uint8_t phy1_addr[sizeof(PHY1_ADDR_t)];
    pos = sizeof(PHY1_ADDR_t);
    OD_Read(objPHY1actualID, MQTTSN_FL_TOPICID_PREDEF, &pos, phy1_addr);

    for(pos = 0; pos < sizeof(PHY1_ADDR_t); pos++)
    {
        uint8_t zif = phy1_addr[pos];   // *(pAddr++);
        uint8_t ch = zif>>4;
        if(ch > 0x09)
        {
            ch += 0x37;
        }
        else
        {
            ch += 0x30;
        }
        *pBuf = ch;
        pBuf++;

        ch = zif & 0x0F;
        if(ch > 0x09)
        {
            ch += 0x37;
        }
        else
        {
            ch += 0x30;
        }
        *pBuf = ch;
        pBuf++;
    }
    Length += (sizeof(PHY1_ADDR_t)*2);

    return Length;
}
*/


static const indextable_t ListPOD[] = {
    {{0, 0, 0}, NULL, NULL, 0},
};
static uint8_t      ListPODflag[sizeof(ListPOD)/sizeof(indextable_t)];


static indextable_t ListOD[OD_MAX_INDEX_LIST];
static uint8_t      ListODflag[OD_MAX_INDEX_LIST];


//////////////////////////
// Callback functions

static uint8_t cbWriteNodeName(subidx_t *pSubidx, uint8_t Len, uint8_t *pBuf)
{
    if(Len > MQTTSN_SIZEOF_CLIENTID)
    {
        return MQTTSN_RET_REJ_NOT_SUPP;
    }
    return eepromWriteOD(pSubidx, Len, pBuf);
}



// Search Object by Index
static indextable_t * scanIndexOD(uint16_t index, uint8_t flags)
{
    uint16_t i;

    flags &= MQTTSN_FL_TOPICID_MASK;
    if(flags == MQTTSN_FL_TOPICID_NORM)
    {
        for(i = 0; i < OD_MAX_INDEX_LIST; i++)
        {
            if(ListOD[i].Index == index)
            {
                return &ListOD[i];
            }
        }
    }
    else if(flags == MQTTSN_FL_TOPICID_PREDEF)
    {
        for(i = 0; i < sizeof(ListPOD)/sizeof(indextable_t); i++)
        {
            if(ListPOD[i].Index == index)
            {
                return (indextable_t *)&ListPOD[i];
            }
        }
    }

    return NULL;
}


void OD_Init(void)
{
    uint8_t pos;
    for(pos = 0; pos < OD_MAX_INDEX_LIST; pos++)
    {
        ListOD[pos].Index = 0xFFFF;
        ListODflag[pos] = 0;
    }

    for(pos = 0; pos < (sizeof(ListPOD)/sizeof(indextable_t)); pos++)
    {
        ListPODflag[pos] = 0;
    }
}

// Make Topic Name from record number
uint8_t OD_MakeTopicName(uint8_t RecNR, uint8_t *pBuf)
{
    *(uint8_t*)(pBuf++) = ListOD[RecNR].sidx.Place;
    *(uint8_t*)(pBuf++) = ListOD[RecNR].sidx.Type;

    uint16_t addr = ListOD[RecNR].sidx.Base;
    // sprintf(pBuf,"%d",addr);
    uint16_t div = 10000;
    uint8_t ch, fl = 0, len = 3;

    while(div >= 10)
    {
        if(addr >= div)
        {
            ch = addr / div;
            addr = addr % div;
        }
        else
        {
            ch = 0;
        }

        div = div/10;

        if((ch != 0) || (fl != 0))
        {
            fl = 1;
            *(pBuf++) = ch + '0';
            len++;
        }
    }
    *pBuf = addr + '0';
    return len;
}

e_MQTTSN_RETURNS_t OD_Register(MQTTSN_MESSAGE_t *pMsg)
{
    return MQTTSN_RET_REJ_NOT_SUPP;
}

void OD_RegAck(uint16_t index)
{
}


e_MQTTSN_RETURNS_t OD_Read(uint16_t Id, uint8_t Flags, uint8_t *pLen, uint8_t *pBuf)
{
    uint8_t tmpLen = 0;
    if(pLen == NULL)
    {
        pLen = &tmpLen;
    }

    indextable_t * pIndex = scanIndexOD(Id, Flags);
    if(pIndex == NULL)
    {
        *pLen = 0;
        return MQTTSN_RET_REJ_INV_ID;
    }

    if(pIndex->cbRead == NULL)
    {
        *pLen = 0;
        return MQTTSN_RET_REJ_NOT_SUPP;
    }

    e_MQTTSN_RETURNS_t retval;
    retval = (pIndex->cbRead)(&pIndex->sidx, pLen, pBuf);
    if(retval != MQTTSN_RET_ACCEPTED)
    {
        *pLen = 0;
    }

    return retval;
}

e_MQTTSN_RETURNS_t OD_ReadPack(uint16_t Id, uint8_t Flags, uint8_t *pLen, uint8_t *pBuf)
{
    *pLen = 0;
    return MQTTSN_RET_REJ_NOT_SUPP;
}


e_MQTTSN_RETURNS_t OD_Write(uint16_t Id, uint8_t Flags, uint8_t Len, uint8_t *pBuf)
{
    return MQTTSN_RET_REJ_NOT_SUPP;
}

e_MQTTSN_RETURNS_t OD_WritePack(uint16_t Id, uint8_t Flags, uint8_t Len, uint8_t *pBuf)
{
    return MQTTSN_RET_REJ_NOT_SUPP;
}
