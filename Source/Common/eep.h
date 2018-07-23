/*
Copyright (c) 2011-2018 <firmware@enviriot.com>

This file is part of the Enviriot project.
https://enviriot.github.io/
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _EEP_H_
#define _EEP_H_

#ifdef __cplusplus
extern "C" {
#endif


// EEPROM Object's
enum
{
// Global
    eepFlag = 0,
    eepFlagBody,

    eepNodeName,
    eepNodeNameBody = eepNodeName + MQTTSN_SIZEOF_CLIENTID,

    eepGateID,
    eepGateIDBody = eepGateID + sizeof(PHY1_ADDR_t) - 1,

/*
// ASLEEP
    eeTASleep,
    eeTASleepbody,
    eeADCaverage,
    eeADCaveragebody,
// BackUp Objects
    eelistOdbu,
    eelistOdbubody = eelistOdbu + OD_MAX_INDEX_LIST * sizeof(subidx_t) - 1,
// PHY
    eePhy1,
    eePhy1Body = eePhy1 + PHY1_SIZEOF_CFG - 1,
#ifdef PHY2_SIZEOF_CFG
    eePhy2,
    eePhy2Body = eePhy1 + PHY2_SIZEOF_CFG - 1,
#endif // EE_PHY2_SIZE
#ifdef PHY3_SIZEOF_CFG
    eePhy3,
    eePhy3Body = eePhy3 + PHY3_SIZEOF_CFG - 1,
#endif // EE_PHY3_SIZE
#ifdef PHY4_SIZEOF_CFG
    eePhy4,
    eePhy4Body = eePhy4 + PHY4_SIZEOF_CFG - 1,
#endif // EE_PHY2_SIZE
*/
/*
// LAN NODE
#ifdef LAN_NODE
    eeMACAddr,
    eeMACAddrBody = eeMACAddr + 5,
    eeIPAddr,
    eeIPAddrbody = eeIPAddr + sizeof(uint32_t) - 1,
    eeIPMask,
    eeIPMaskbody = eeIPMask + sizeof(uint32_t) - 1,
    eeIPRouter,
    eeIPRouterbody = eeIPRouter + sizeof(uint32_t) - 1,
    eeIPBroker,
    eeIPBrokerbody = eeIPBroker + sizeof(uint32_t) - 1,
#endif  //  LAN_NODE
// RF NODE
#if (defined RF_ADDR_t)
    eeNodeID,
    eeNodeIDbody = eeNodeID + sizeof(RF_ADDR_t) - 1,
    eeGateID,
    eeGateIDbody = eeGateID + sizeof(RF_ADDR_t) - 1,
    eeGroupID,
    eeGroupIDbody,
    eeChannel,
    eeRFpower,
    eeRFkey,
    eeRFkeyBody = eeRFkey + 16,
#endif  //  RF_ADDR_t
*/
/*
#ifdef EXTPLC_USED
    eePLCStackBot,
    eePLCStackBotBody = eePLCStackBot + sizeof(uint32_t) - 1,
    eePLCprogram,
    eePLCprogram_body = eePLCprogram + EXTPLC_SIZEOF_PRG - 1,
#endif  //  EXTPLC_USED
*/
    eeNextFreeAddress
} e_EEP_Addr;


void    eepWriteArray(uint16_t Addr, uint8_t Len, uint8_t * pBuf);
uint8_t eepReadArray(uint16_t Addr, uint8_t * pBuf);

void    eepWriteRaw(uint16_t Addr, uint8_t Len, uint8_t * pBuf);
void    eepReadRaw(uint16_t Addr, uint8_t Len, uint8_t * pBuf);


#ifdef __cplusplus
}
#endif
#endif
