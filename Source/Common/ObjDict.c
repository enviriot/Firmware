/*
Copyright (c) 2011-2018 <firmware@enviriot.com>

This file is part of the Enviriot project.
https://enviriot.github.io/
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/


// ToDo Clear enent only on read

#include <stdlib.h>

#include "config.h"
#include "hal.h"
#include "mqTypes.h"
#include "eep.h"
#include "ObjDict.h"


static uint8_t cbReadNodeName(subidx_t *pSubidx, uint8_t *pLen, uint8_t *pBuf);
static uint8_t cbWriteNodeName(subidx_t *pSubidx, uint8_t Len, uint8_t *pBuf);

static uint8_t cbReadPHY1(subidx_t *pSubidx, uint8_t *pLen, uint8_t *pBuf);
static uint8_t cbWritePHY1(subidx_t *pSubidx, uint8_t Len, uint8_t *pBuf);

// Predefined Object's
static const uint8_t psDeviceTyp[] = {
                                OD_DEV_UC_TYPE,           // uC Family
                                OD_DEV_UC_SUBTYPE,        // uC SubType
                                OD_DEV_PHY1,              // PHY1 type
#ifdef OD_DEV_PHY2
                                OD_DEV_PHY2,              // PHY2 type
#endif  // OD_DEV_PHY2
#ifdef OD_DEV_PHY3
                                OD_DEV_PHY3,              // PHY3 type
#endif  // OD_DEV_PHY3
#ifdef OD_DEV_PHY4
                                OD_DEV_PHY4,              // PHY4 type
#endif  // OD_DEV_PHY4
                                OD_DEV_HW_TYP_H,          // HW Version High
                                OD_DEV_HW_TYP_L,          // HW Version Low
                                '.',                      // Delimiter
                                OD_DEV_SWVERSH,           // Software Version
                                OD_DEV_SWVERSM,
                                OD_DEV_SWVERSL};


static const indextable_t ListPOD[] = {
    // Global Settings
    {{0, 0, 0}, cbReadNodeName, cbWriteNodeName, objNodeName},

    // PHY1 Settings
    {{0, 0, 0x00}, NULL, cbWritePHY1, objPHY1control},
    {{0, 0, 0x01}, cbReadPHY1, cbWritePHY1, objPHY1address},
    {{0, 0, 0x02}, cbReadPHY1, cbWritePHY1, objPHY1mask},
    {{0, 0, 0x03}, cbReadPHY1, cbWritePHY1, objPHY1gate},
    {{0, 0, 0x04}, cbReadPHY1, cbWritePHY1, objPHY1broker},
    {{0, 0, 0x05}, cbReadPHY1, cbWritePHY1, objPHY1group},
    {{0, 0, 0x06}, cbReadPHY1, cbWritePHY1, objPHY1channel},
    {{0, 0, 0x08}, cbReadPHY1, cbWritePHY1, objPHY1power},
    {{0, 0, 0x09}, cbReadPHY1, cbWritePHY1, objPHY1key},
    {{0, 0, 0x0F}, cbReadPHY1, cbWritePHY1, objPHY1mac},
    {{0, 0, 0x18}, cbReadPHY1, NULL, objPHY1actualID},
    {{0, 0, 0x19}, cbReadPHY1, NULL, objPHY1undefID},
    {{0, 0, 0x1A}, cbReadPHY1, NULL, objPHY1broadID},
    {{0, 0, 0x1F}, cbReadPHY1, NULL, objPHY1rssi}

    /*{{0, 0, 0}, cbReadDeviceType, NULL, objDeviceTyp}*/
};


static uint8_t      ListPODevent[sizeof(ListPOD)/sizeof(indextable_t)];


static indextable_t ListOD[OD_MAX_INDEX_LIST];
static uint8_t      ListODevent[OD_MAX_INDEX_LIST];


//////////////////////////
// Callback functions

static uint8_t cbReadNodeName(__attribute__ ((unused)) subidx_t *pSubidx, uint8_t *pLen, uint8_t *pBuf)
{
    uint8_t Len = eepReadArray(eepNodeName, pBuf);
    if(Len > 0)
    {
        *pLen = Len;
        return MQTTSN_RET_ACCEPTED;
    }

    // Node Name not defined, use default name
    uint8_t pos, Length;
    for(Length = 0; (Length < sizeof(psDeviceTyp)) && (psDeviceTyp[Length] != '.'); Length++)
    {
        *(pBuf++) = psDeviceTyp[Length];
    }

    *(pBuf++) = '_';
    Length++;

    uint8_t phy1_addr[sizeof(PHY1_ADDR_t)];
    OD_Read(objPHY1actualID, MQTTSN_FL_TOPICID_PREDEF, NULL, phy1_addr);

    for(pos = 0; pos < sizeof(PHY1_ADDR_t); pos++)
    {
        uint8_t zif = phy1_addr[pos];
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
    *pLen = Length;

    return MQTTSN_RET_ACCEPTED;
}

static uint8_t cbWriteNodeName(__attribute__ ((unused)) subidx_t *pSubidx, uint8_t Len, uint8_t *pBuf)
{
    if(Len > MQTTSN_SIZEOF_CLIENTID)
    {
        return MQTTSN_RET_REJ_NOT_SUPP;
    }

    eepWriteArray(eepNodeName, Len, pBuf);
    return MQTTSN_RET_ACCEPTED;
}

// PHY Section
static uint8_t cbReadPHY1(subidx_t *pSubidx, uint8_t *pLen, uint8_t *pBuf)
{
    return PHY1_ReadOD(pSubidx->Base, pLen, pBuf);
}

static uint8_t cbWritePHY1(subidx_t *pSubidx, uint8_t Len, uint8_t *pBuf)
{
    return PHY1_WriteOD(pSubidx->Base, Len, pBuf);
}

// Callback functions
//////////////////////////

//////////////////////////
// Local functions

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
                ListODevent[i] = 0;
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
                ListPODevent[i] = 0;
                return (indextable_t *)&ListPOD[i];
            }
        }
    }

    return NULL;
}

// Load Default Settings
static void od_defaults(void)
{
    // PHY's Load Defaults Settings
    uint16_t uiTmp = 0xDEFA;
    OD_Write(objPHY1control, MQTTSN_FL_TOPICID_PREDEF, 2, (uint8_t *)&uiTmp);
#ifdef OD_DEV_PHY2
    OD_Write(objPHY2control, MQTTSN_FL_TOPICID_PREDEF, 2, (uint8_t *)&uiTmp);
#endif  // OD_DEV_PHY2
#ifdef OD_DEV_PHY3
    OD_Write(objPHY3control, MQTTSN_FL_TOPICID_PREDEF, 2, (uint8_t *)&uiTmp);
#endif  // OD_DEV_PHY3
#ifdef OD_DEV_PHY4
    OD_Write(objPHY4control, MQTTSN_FL_TOPICID_PREDEF, 2, (uint8_t *)&uiTmp);
#endif  // OD_DEV_PHY4

    OD_Write(objNodeName, MQTTSN_FL_TOPICID_PREDEF, 0, NULL);       // Device Name
}

// Local functions
//////////////////////////


//////////////////////////
// OD API

void OD_Init(void)
{
    uint8_t     ucTmp;
    uint16_t    uiTmp, Flag = 0;

    eepReadRaw(eepFlag, 2, (uint8_t *)&uiTmp);

    for(ucTmp = 0; ucTmp < sizeof(psDeviceTyp); ucTmp++)
    {
        Flag += psDeviceTyp[ucTmp];
    }

    if(uiTmp != Flag)
    {
        od_defaults();
        eepWriteRaw(eepFlag, 2, (uint8_t *)&Flag);
    }

    for(ucTmp = 0; ucTmp < OD_MAX_INDEX_LIST; ucTmp++)
    {
        ListOD[ucTmp].Index = 0xFFFF;
        ListODevent[ucTmp] = 0;
    }

    for(ucTmp = 0; ucTmp < (sizeof(ListPOD)/sizeof(indextable_t)); ucTmp++)
    {
        ListPODevent[ucTmp] = 0;
    }
}

e_MQTTSN_RETURNS_t OD_Read(uint16_t Index, uint8_t Flags, uint8_t *pLen, uint8_t *pBuf)
{
    uint8_t tmpLen = 0;
    if(pLen == NULL)
    {
        pLen = &tmpLen;
    }

    indextable_t * pIndex = scanIndexOD(Index, Flags);
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

e_MQTTSN_RETURNS_t OD_Write(uint16_t Index, uint8_t Flags, uint8_t Len, uint8_t *pBuf)
{
    indextable_t * pIndex = scanIndexOD(Index, Flags);
    if(pIndex == NULL)
    {
        return MQTTSN_RET_REJ_INV_ID;
    }
    
    if(pIndex->cbWrite == NULL)
    {
        return MQTTSN_RET_REJ_NOT_SUPP;
    }

    return (pIndex->cbWrite)(&pIndex->sidx, Len, pBuf);
}

void OD_SetEvent(uint16_t Index, uint8_t Flags, uint8_t Event)
{
    uint16_t i;

    Flags &= MQTTSN_FL_TOPICID_MASK;
    if(Flags == MQTTSN_FL_TOPICID_NORM)
    {
        for(i = 0; i < OD_MAX_INDEX_LIST; i++)
        {
            if(ListOD[i].Index == Index)
            {
                ListODevent[i] |= Event;
                break;
            }
        }
    }
    else if(Flags == MQTTSN_FL_TOPICID_PREDEF)
    {
        for(i = 0; i < sizeof(ListPOD)/sizeof(indextable_t); i++)
        {
            if(ListPOD[i].Index == Index)
            {
                ListPODevent[i] |= Event;
                break;
            }
        }
    }
}



/*
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

e_MQTTSN_RETURNS_t OD_ReadPack(uint16_t Id, uint8_t Flags, uint8_t *pLen, uint8_t *pBuf)
{
    *pLen = 0;
    return MQTTSN_RET_REJ_NOT_SUPP;
}

e_MQTTSN_RETURNS_t OD_WritePack(uint16_t Id, uint8_t Flags, uint8_t Len, uint8_t *pBuf)
{
    return MQTTSN_RET_REJ_NOT_SUPP;
}
*/
