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

    // PHY Settings
    objPHY1control      = (uint16_t)0xFF40,
    objPHY1address      = (uint16_t)0xFF41,
    objPHY1mask         = (uint16_t)0xFF42,
    objPHY1gate         = (uint16_t)0xFF43,
    objPHY1broker       = (uint16_t)0xFF44,
    objPHY1group        = (uint16_t)0xFF45,
    objPHY1channel      = (uint16_t)0xFF46,
    objPHY1power        = (uint16_t)0xFF48,
    objPHY1key          = (uint16_t)0xFF49,
    objPHY1mac          = (uint16_t)0xFF4F,
    objPHY1actualID     = (uint16_t)0xFF58,
    objPHY1undefID      = (uint16_t)0xFF59,
    objPHY1broadID      = (uint16_t)0xFF5A,
    objPHY1rssi         = (uint16_t)0xFF5F,

    // Read Only Variables
    objDeviceTyp        = (uint16_t)0xFFC0, // _declarer<String>

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
void OD_SetEvent(uint16_t Index, uint8_t Flags, uint8_t Event);


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
