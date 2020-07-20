/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "uDACS_pins.h"	// #include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_adc_async.h>

#include <hal_spi_m_async.h>

#include <hal_i2c_m_async.h>

#include <hal_i2c_m_async.h>

#include <hal_spi_m_async.h>
#include <hal_usart_async.h>
#include <hal_timer.h>
#include <hpl_tc_base.h>
#include <hal_can_async.h>

extern struct adc_async_descriptor VMON_ADC;

extern struct spi_m_async_descriptor PMOD_SPI;

extern struct i2c_m_async_desc DADC_I2C;

extern struct i2c_m_async_desc PMON_I2C;

extern struct spi_m_async_descriptor PSD_SPI;
extern struct usart_async_descriptor USART_CTRL;
extern struct timer_descriptor       TIMER_0;
extern struct can_async_descriptor   CAN_CTRL;

void VMON_ADC_init(void);

void PMOD_SPI_PORT_init(void);
void PMOD_SPI_CLOCK_init(void);
void PMOD_SPI_init(void);

void DADC_I2C_PORT_init(void);
void DADC_I2C_CLOCK_init(void);
void DADC_I2C_init(void);

void PMON_I2C_PORT_init(void);
void PMON_I2C_CLOCK_init(void);
void PMON_I2C_init(void);

void PSD_SPI_PORT_init(void);
void PSD_SPI_CLOCK_init(void);
void PSD_SPI_init(void);

#if 0	// Replaced on usart.c
void USART_CTRL_PORT_init(void);
void USART_CTRL_CLOCK_init(void);
void USART_CTRL_init(void);
#endif
/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
