/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMC21 has 9 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7
#define GPIO_PIN_FUNCTION_I 8

#define VMON1 GPIO(GPIO_PORTA, 3)
#define AOMON3 GPIO(GPIO_PORTA, 4)
#define AOMON2 GPIO(GPIO_PORTA, 5)
#define AOMON1 GPIO(GPIO_PORTA, 6)
#define AOMON0 GPIO(GPIO_PORTA, 7)
#define DADC_SDA GPIO(GPIO_PORTA, 8)
#define DADC_SCL GPIO(GPIO_PORTA, 9)
#define SD_MOSI GPIO(GPIO_PORTA, 12)
#define SD_SCLK GPIO(GPIO_PORTA, 13)
#define SD_MISO GPIO(GPIO_PORTA, 15)
#define PMOD7 GPIO(GPIO_PORTA, 16)
#define PMOD5 GPIO(GPIO_PORTA, 17)
#define PMOD1 GPIO(GPIO_PORTA, 19)
#define UC_SDA GPIO(GPIO_PORTA, 22)
#define UC_SCL GPIO(GPIO_PORTA, 23)
#define CANTX GPIO(GPIO_PORTA, 24)
#define CANRX GPIO(GPIO_PORTA, 25)
#define FAIL_OFF GPIO(GPIO_PORTA, 27)
#define J35_EN GPIO(GPIO_PORTB, 2)
#define J34_EN GPIO(GPIO_PORTB, 3)
#define SPR7 GPIO(GPIO_PORTB, 8)
#define SPR8 GPIO(GPIO_PORTB, 9)
#define P_CS GPIO(GPIO_PORTB, 11)
#define UTXFRX GPIO(GPIO_PORTB, 22)
#define URXFTX GPIO(GPIO_PORTB, 23)

#endif // ATMEL_START_PINS_H_INCLUDED
