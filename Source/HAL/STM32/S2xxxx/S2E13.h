/*
Copyright (c) 2011-2015 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _S2E13_H
#define _S2E13_H

// Board: S2EC13
// uC: STM32F051C8T6
// PHY1: ENC28J60

// GPIOA
// Pin  Port    CN  Func            PWM
//   0  PA0     3                  !TIM2_CH1
//   1  PA1     4                   TIM2_CH2
//   2  PA2     5   _USART2_TX      TIM15_CH1
//   3  PA3     6   _USART2_RX      TIM15_CH2
//   4  PA4     15                  TIM14_CH1
//   5  PA5     16                 !TIM2_CH1
//   6  PA6     17                  TIM3_CH1
//   7  PA7     18                  TIM3_CH2
//   8  PA8
//   9  PA9     9   _USART1_TX      TIM1_CH2
//  10  PA10    10  _USART1_RX      TIM1_CH3
//  11  PA11    7                   TIM1_CH4
//  12  PA12    8                   
//  13  PA13        SWDIO
//  14  PA14        SWCLK
//  15  PA15        ENC_IRQ
// GPIOB
//  16  PB0     19                  TIM3_CH3
//  17  PB1     20                  TIM3_CH4
//  18  PB2         LED
//  19  PB3         ENC_SCK
//  20  PB4         ENC_MISO
//  21  PB5         ENC_MOSI
//  22  PB6         ENC_SEL
//  23  PB7         ENC_RST
//  24  PB8
//  25  PB9
//  26  PB10    13  SCL2            TIM2_CH3
//  27  PB11    14  SDA2            TIM2_CH4
//  28  PB12
//  29  PB13
//  30  PB14
//  31  PB15

#ifdef __cplusplus
extern "C" {
#endif

// System Settings
//#define HAL_USE_RTC                 1
//#define HAL_RTC_USE_HSE             1

// DIO Section
//#define EXTDIO_USED                 1
//#define EXTDIO_MAXPORT_NR           2
//#define HAL_DIO_MAPPING             {17, 16, 7, 6, 5, 4, 27, 26, 10, 9, 12, 11, 3, 2, 1, 0}
// End DIO Section

// PWM Section
//#define EXTPWM_USED                 1
#define HAL_PWM_PORT2CFG             {((1<<8) |  (3<<3) | 3),    /* PB1:  AF1, TIM3_CH4  */ \
                                     ((1<<8) |  (3<<3) | 2),    /* PB0:  AF1, TIM3_CH3  */ \
                                     ((1<<8) |  (3<<3) | 1),    /* PA7:  AF1, TIM3_CH2  */ \
                                     ((1<<8) |  (3<<3) | 0),    /* PA6:  AF1, TIM3_CH1  */ \
                                     ((2<<8) |  (2<<3) | 0),    /* PA5:  AF2, TIM2_CH1  */ \
                                     ((4<<8) | (14<<3) | 0),    /* PA4:  AF4, TIM14_CH1 */ \
                                     ((2<<8) |  (2<<3) | 3),    /* PB11: AF2, TIM2_CH4  */ \
                                     ((2<<8) |  (2<<3) | 2),    /* PB10: AF2, TIM2_CH3  */ \
                                     ((2<<8) |  (1<<3) | 2),    /* PA10: AF2, TIM1_CH3  */ \
                                     ((2<<8) | ( 1<<3) | 1),    /* PA9:  AF2, TIM1_CH2  */ \
                                     255,                       /* PA12: No Config      */ \
                                     ((2<<8) |  (1<<3) | 3),    /* PA11: AF2, TIM1_CH4  */ \
                                     ((0<<8) | (15<<3) | 1),    /* PA3:  AF0, TIM15_CH2 */ \
                                     ((0<<8) | (15<<3) | 0),    /* PA2:  AF0, TIM15_CH1 */ \
                                     ((2<<8) |  (2<<3) | 1),    /* PA1:  AF2, TIM2_CH2  */ \
                                     ((2<<8) |  (2<<3) | 0)}    /* PA0:  AF2, TIM2_CH1  */

// End PWM Section

// PA0-PA7: 0 - 7
// PB0-PB1: 8 - 9
// Analogue Inputs
//#define EXTAIN_USED                 1
//#define EXTAIN_MAXPORT_NR           10
//#define HAL_AIN_BASE2APIN           {9, 8, 7, 6, 5, 4, 255, 255, 255, 255, 255, 255, 3, 2, 1, 0}
// End Analogue Inputs

// UART Section
//#define HAL_USE_USART1              0           // Mapping to logical port
//#define HAL_USE_USART2              1
//#define EXTSER_USED                 2
// End UART Section

// TWI Section
//#define HAL_TWI_BUS                 2       // I2C_Bus 1 - I2C1, 2 - I2C2
//#define EXTTWI_USED                 1
// End TWI Section

// LEDs
#define LED_On()                    GPIOB->BSRR = GPIO_BSRR_BS_2
#define LED_Off()                   GPIOB->BSRR = GPIO_BSRR_BR_2
#define LED_Init()                  hal_gpio_cfg(GPIOB, GPIO_Pin_2, DIO_MODE_OUT_PP)

// ENC Section
#define HAL_USE_SPI1                2   // SPI1, Config 2: PB3-PB5
#define ENC_USE_SPI                 1
#define ENC_NSS_PIN                 22  // PB6
#define ENC_SELECT()                GPIOB->BRR = GPIO_Pin_6
#define ENC_RELEASE()               {while(SPI1->SR & SPI_SR_BSY); GPIOB->BSRR = GPIO_Pin_6;}
#define ENC_PHY                     1
#include "PHY/ENC28J60/enc28j60_phy.h"
// End ENC Section

// Object's Dictionary Section
#define OD_MAX_INDEX_LIST           40
#define OD_DEV_UC_TYPE              'S'
#define OD_DEV_UC_SUBTYPE           '2'
#define OD_DEV_PHY1                 'E'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '3'

//#define OD_DEF_DEV_MAC              {0x00,0x04,0xA3,0x00,0x00,0x10}   // MAC MSB->LSB
//#define OD_DEF_IP_ADDR              inet_addr(192,168,10,202)
//#define OD_DEF_IP_MASK              inet_addr(255,255,255,0)
//#define OD_DEF_IP_ROUTER            inet_addr(192,168,10,1)
//#define OD_DEF_IP_BROKER            inet_addr(192,168,20,8)

#ifdef __cplusplus
}
#endif

#endif // _S2E13_H
