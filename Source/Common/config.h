/*
Copyright (c) 2011-2018 <firmware@enviriot.com>

This file is part of the Enviriot project.
https://enviriot.github.io/
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// Global configuration settings

#ifndef _CONFIG_H
#define _CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// System Settings
#define POLL_TMR_FREQ                   100     // System Tick Frequenz

// MQTT-SN Section
#define MQTTSN_MSG_SIZE                 48      // Size of payload(base for all buffers)
//#define MQTTSN_USE_DHCP                 1       // Use Automatic address resolution
                                                //  Not Standard Messages
//#define MQTTSN_REBOOT_ON_LOST           1       // Reboot on connection lost

// Object Dictionary
#define OD_DEV_SWVERSH                  '4'     // Software Version
#define OD_DEV_SWVERSM                  '0'
#define OD_DEV_SWVERSL                  '0'

// Main Tick's
void        SystemTick(void);

#ifdef __cplusplus
}
#endif

#endif  //  _CONFIG_H
