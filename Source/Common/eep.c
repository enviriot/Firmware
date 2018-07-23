/*
Copyright (c) 2011-2018 <firmware@enviriot.com>

This file is part of the Enviriot project.
https://enviriot.github.io/
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#include "config.h"
#include "hal.h"
#include "mqTypes.h"
#include "eep.h"

void eepWriteArray(uint16_t Addr, uint8_t Len, uint8_t * pBuf)
{
    eeprom_write(&Len, Addr, 1);
    if(Len > 0)
    {
        eeprom_write(pBuf, Addr + 1, Len);
    }
}

uint8_t eepReadArray(uint16_t Addr, uint8_t * pBuf)
{
    uint8_t Len;
    eeprom_read(&Len, Addr, 1);
    if(Len > 0)
    {
        eeprom_read(pBuf, Addr + 1, Len);
    }
    return Len;
}

void eepWriteRaw(uint16_t Addr, uint8_t Len, uint8_t * pBuf)
{
    eeprom_write(pBuf, Addr, Len);
}

void eepReadRaw(uint16_t Addr, uint8_t Len, uint8_t * pBuf)
{
    eeprom_read(pBuf, Addr, Len);
}
