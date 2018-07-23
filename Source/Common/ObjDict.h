/*
Copyright (c) 2011-2018 <firmware@enviriot.com>

This file is part of the Enviriot project.
https://enviriot.github.io/
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _OBJ_DICT_H_
#define _OBJ_DICT_H_

#ifdef __cplusplus
extern "C" {
#endif


// Predefined variables
enum
{
    // Global Settings
    objNodeName         = (uint16_t)0xFF00, // _sName<String>

    objGateID           = (uint16_t)0xFF04, // cfg/XD_GateId

    // PHY Settings
    objPHY1control      = (uint16_t)0xFF40,
    objPHY1nodeID       = (uint16_t)0xFF41,
    objPHY1actualID     = (uint16_t)0xFF58,
    objPHY1undefID      = (uint16_t)0xFF59,
    objPHY1broadID      = (uint16_t)0xFF5A,
/*
    // Read Only Variables
    objDeviceTyp        = (uint16_t)0xFFC0, // _declarer<String>
    objPHY1addr         = (uint16_t)0xFFC1, // cfg/_a_phy1
    objPHY2addr         = (uint16_t)0xFFC2, // cfg/_a_phy2
    objPHY3addr         = (uint16_t)0xFFC3, // cfg/_a_phy3
    objPHY4addr         = (uint16_t)0xFFC4, // cfg/_a_phy4
*/
    // Debug Variables
    objLogD             = (uint16_t)0xFFE0, // Data logging, log level - Debug
    objLogI             = (uint16_t)0xFFE1, // Data logging, log level - Info
    objLogW             = (uint16_t)0xFFE2, // Data logging, log level - Warning
    objLogE             = (uint16_t)0xFFE3  // Data logging, log level - Error
}eObjList_t;


// Variable description
typedef struct
{
  uint8_t   Place;
  uint8_t   Type;
  uint16_t  Base;
}subidx_t;

// Callback Read
typedef e_MQTTSN_RETURNS_t (*cbRead_t)(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf);
// Callback Write
typedef e_MQTTSN_RETURNS_t (*cbWrite_t)(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf);

// Structure for creating entries
typedef struct
{
  subidx_t    sidx;
  cbRead_t    cbRead;
  cbWrite_t   cbWrite;
  uint16_t    Index;
}indextable_t;

void OD_Init(void);

e_MQTTSN_RETURNS_t OD_Read(uint16_t Id, uint8_t Flags, uint8_t *pLen, uint8_t *pBuf);
e_MQTTSN_RETURNS_t OD_Write(uint16_t Id, uint8_t Flags, uint8_t Len, uint8_t *pBuf);

/*
uint8_t OD_MakeTopicName(uint8_t RecNR, uint8_t *pBuf);
e_MQTTSN_RETURNS_t OD_Register(MQTTSN_MESSAGE_t *pMsg);
void OD_RegAck(uint16_t index);

e_MQTTSN_RETURNS_t OD_ReadPack(uint16_t Id, uint8_t Flags, uint8_t *pLen, uint8_t *pBuf);
e_MQTTSN_RETURNS_t OD_WritePack(uint16_t Id, uint8_t Flags, uint8_t Len, uint8_t *pBuf);
*/

#ifdef __cplusplus
}
#endif
#endif
