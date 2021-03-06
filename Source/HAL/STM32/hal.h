/*
Copyright (c) 2011-2016 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef __HAL_H
#define __HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#if (defined STM32F0)
#include "HW_STM32F0.h"
#elif (defined STM32F1)
#include "HW_STM32F1.h"
#elif (defined STM32F3)
#include "HW_STM32F3.h"
#elif (defined STM32L0)
#include "HW_STM32L0.h"
#elif (defined STM32L1)
#include "HW_STM32L1.h"
#else
#error Unknown Family
#endif  // uC Family

//////////////////////////////////////////////////////////////
// HAL System Section
void hal_SetSysClock(void);
void hal_prepare_gpio(void);

uint32_t hal_GetClock(uint32_t base);

void hal_rtc_init(void);

void halEnterCritical(void);
void halLeaveCritical(void);
#define ENTER_CRITICAL_SECTION      halEnterCritical
#define LEAVE_CRITICAL_SECTION      halLeaveCritical

uint32_t hal_GetDeviceID(void);

// Hardware specific options
#define portBYTE_ALIGNMENT          8
#define portPOINTER_SIZE_TYPE       uint32_t
#define configTOTAL_HEAP_SIZE       2048

// End HAL System Section
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// DIO/GPIO Section
#define DIO_PORT_POS                4
#define DIO_PORT_MASK               0x0F
#define DIO_PORT_TYPE               uint16_t

// GPIO compatibility
#define GPIO_Pin_0                 ((uint16_t)0x0001)  /*!< Pin 0 selected */
#define GPIO_Pin_1                 ((uint16_t)0x0002)  /*!< Pin 1 selected */
#define GPIO_Pin_2                 ((uint16_t)0x0004)  /*!< Pin 2 selected */
#define GPIO_Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected */
#define GPIO_Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected */
#define GPIO_Pin_5                 ((uint16_t)0x0020)  /*!< Pin 5 selected */
#define GPIO_Pin_6                 ((uint16_t)0x0040)  /*!< Pin 6 selected */
#define GPIO_Pin_7                 ((uint16_t)0x0080)  /*!< Pin 7 selected */
#define GPIO_Pin_8                 ((uint16_t)0x0100)  /*!< Pin 8 selected */
#define GPIO_Pin_9                 ((uint16_t)0x0200)  /*!< Pin 9 selected */
#define GPIO_Pin_10                ((uint16_t)0x0400)  /*!< Pin 10 selected */
#define GPIO_Pin_11                ((uint16_t)0x0800)  /*!< Pin 11 selected */
#define GPIO_Pin_12                ((uint16_t)0x1000)  /*!< Pin 12 selected */
#define GPIO_Pin_13                ((uint16_t)0x2000)  /*!< Pin 13 selected */
#define GPIO_Pin_14                ((uint16_t)0x4000)  /*!< Pin 14 selected */
#define GPIO_Pin_15                ((uint16_t)0x8000)  /*!< Pin 15 selected */

// DIO Modes
// 11-8 bits:   AF number
//  6-5 bits:   Low / Medium / Fast / High Speed
//  4-3 bits:   Input / Output / AF / Analog
//  2 bit:      Push-Pull / Open Drain
//  0-1 bits:   Float / PullUp / PullDown

#define DIO_AF_OFFS                 8

#define DIO_MODE_IN_FLOAT           0x00
#define DIO_MODE_IN_PU              0x01
#define DIO_MODE_IN_PD              0x02
#define DIO_MODE_OUT_PP             0x08
//#define DIO_MODE_OUT_OD             0x0C
#define DIO_MODE_OUT_PP_HS          0x68    // Output, push-pull, high speed
#define DIO_MODE_AF_PP              0x10
//#define DIO_MODE_AF_PU              0x11
//#define DIO_MODE_AF_PD              0x12
#define DIO_MODE_AF_OD              0x14
#define DIO_MODE_AF_PP_HS           0x70    // Alternative function, Push/pull, high speed
#define DIO_MODE_AIN                0x18

void        hal_gpio_cfg(GPIO_TypeDef * GPIOx, uint16_t Mask, uint16_t Mode);

uint8_t     hal_dio_base2pin(uint16_t base);
void        hal_dio_configure(uint8_t pin, uint16_t Mode);
uint16_t    hal_dio_read(uint8_t PortNr);
void        hal_dio_set(uint8_t PortNr, uint16_t Mask);
void        hal_dio_reset(uint8_t PortNr, uint16_t Mask);

// EXTI
#define HAL_EXTI_TRIGGER_FALLING    1
#define HAL_EXTI_TRIGGER_RISING     2
#define HAL_EXTI_TRIGGER_BOTH       3

typedef void (*cbEXTI_t)(void);

void        hal_exti_config(uint8_t pin, uint8_t Trigger, cbEXTI_t pCallback);
void        hal_exti_trig(uint8_t pin);

// DIO Section
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// SPI Section
#define HAL_SPI_MODE_0              0
#define HAL_SPI_MODE_1              1
#define HAL_SPI_MODE_2              2
#define HAL_SPI_MODE_3              3
#define HAL_SPI_MSB                 0
#define HAL_SPI_LSB                 4
#define HAL_SPI_8B                  0
#define HAL_SPI_16B                 8

void        hal_spi_cfg(uint8_t port, uint8_t mode, uint32_t speed);
uint8_t     hal_spi_exch8(uint8_t port, uint8_t data);
uint16_t    hal_spi_exch16(uint8_t port, uint16_t data);
// SPI Section
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// AIN Section
uint8_t     hal_ain_base2apin(uint16_t base);
uint8_t     hal_ain_apin2dio(uint8_t apin);
void        hal_ain_configure(uint8_t apin, uint8_t aref);
void        hal_ain_select(uint8_t apin, uint8_t aref);
int16_t     hal_ain_get(void);

#define EXTAIN_REF                  0x02        // Bit0 - Ext, Bit1 - Vcc, Bit2 - Int1, Bit3 - Int2

// AIN Section
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// TWI Section
void        hal_twi_get_pins(uint8_t * pSCL, uint8_t * pSDA);
bool        hal_twi_configure(uint8_t enable);
void        hal_twi_stop(void);
void        hal_twi_start(void);
// TWI Section
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// UART Section
#define     UART_BAUD_2K4       0
#define     UART_BAUD_4K8       1
#define     UART_BAUD_9K6       2
#define     UART_BAUD_19K2      3
#define     UART_BAUD_38K4      4
#define     UART_BAUD_200K      5
#define     UART_BAUD_MAX       4

void        hal_uart_get_pins(uint8_t port, uint8_t * pRx, uint8_t * pTx);
void        hal_uart_deinit(uint8_t port);
void        hal_uart_init_hw(uint8_t port, uint8_t nBaud, uint8_t enable);
bool        hal_uart_free(uint8_t port);
void        hal_uart_send(uint8_t port, uint8_t len, uint8_t * pBuf);
bool        hal_uart_datardy(uint8_t port);
uint8_t     hal_uart_get(uint8_t port);
// UART Section
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// SysTick IRQ subroutine
void        SysTick_Handler(void);

//////////////////////////////////////////////////////////////
// HAL API
void eeprom_init_hw(void);
void eeprom_read(uint8_t *pBuf, uint16_t Addr, uint8_t Len);
bool eeprom_write(uint8_t *pBuf, uint16_t Addr, uint8_t Len);

void _delay_us(uint16_t us);

uint16_t HAL_get_submstick(void);

//////////////////////////////////////////////////////////////
// RTC Section
void HAL_RTC_Set(uint8_t *pBuf);
uint8_t HAL_RTC_Get(uint8_t *pBuf);
uint32_t HAL_RTC_SecNow(void);
uint32_t HAL_RTC_DateNow(void);
// RTC Section
//////////////////////////////////////////////////////////////


// Common HAL Section
void        HAL_Init(void);
void        HAL_StartSystemTick(void);
uint16_t    HAL_RNG(void);
uint32_t    HAL_RNG32(void);
uint32_t    HAL_get_ms(void);
uint32_t    HAL_get_sec(void);
void        HAL_PreASleep(void);
void        HAL_ASleep(uint16_t duration);
void        HAL_AWake(void);
void        HAL_Reboot(void);



#define  HAL_Reboot     NVIC_SystemReset

#ifdef __cplusplus
}
#endif

#endif  //  __HAL_H
