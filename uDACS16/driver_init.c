/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>

#include <hpl_adc_base.h>

/* The channel amount for ADC */
#define VMON_ADC_CH_AMOUNT 1

/* The buffer size for ADC */
#define VMON_ADC_BUFFER_SIZE 16

/* The maximal channel number of enabled channels */
#define VMON_ADC_CH_MAX 0

/*! The buffer size for USART */
#define USART_CTRL_BUFFER_SIZE 16

struct adc_async_descriptor         VMON_ADC;
struct adc_async_channel_descriptor VMON_ADC_ch[VMON_ADC_CH_AMOUNT];
struct usart_async_descriptor       USART_CTRL;
struct timer_descriptor             TIMER_0;
struct can_async_descriptor         CAN_CTRL;

static uint8_t VMON_ADC_buffer[VMON_ADC_BUFFER_SIZE];
static uint8_t VMON_ADC_map[VMON_ADC_CH_MAX + 1];
static uint8_t USART_CTRL_buffer[USART_CTRL_BUFFER_SIZE];

struct spi_m_async_descriptor PMOD_SPI;

struct i2c_m_async_desc DADC_I2C;

struct i2c_m_async_desc PMON_I2C;

struct spi_m_async_descriptor PSD_SPI;

/**
 * \brief ADC initialization function
 *
 * Enables ADC peripheral, clocks and initializes ADC driver
 */
void VMON_ADC_init(void)
{
	hri_mclk_set_APBCMASK_ADC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, CONF_GCLK_ADC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	adc_async_init(
	    &VMON_ADC, ADC0, VMON_ADC_map, VMON_ADC_CH_MAX, VMON_ADC_CH_AMOUNT, &VMON_ADC_ch[0], _adc_get_adc_async());
	adc_async_register_channel_buffer(&VMON_ADC, 0, VMON_ADC_buffer, VMON_ADC_BUFFER_SIZE);

	// Disable digital pin circuitry
	gpio_set_pin_direction(VMON1, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(VMON1, PINMUX_PA03B_ADC0_AIN1);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AOMON3, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AOMON3, PINMUX_PA04B_ADC0_AIN4);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AOMON2, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AOMON2, PINMUX_PA05B_ADC0_AIN5);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AOMON1, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AOMON1, PINMUX_PA06B_ADC0_AIN6);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AOMON0, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AOMON0, PINMUX_PA07B_ADC0_AIN7);
}

void PMOD_SPI_PORT_init(void)
{

	// Set pin direction to input
	gpio_set_pin_direction(PMOD7, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PMOD7,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PMOD7, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_level(PMOD5,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PMOD5, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PMOD5, PINMUX_PA17C_SERCOM1_PAD1);

	gpio_set_pin_level(PMOD1,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PMOD1, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PMOD1, PINMUX_PA19C_SERCOM1_PAD3);
}

void PMOD_SPI_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM1_bit(MCLK);
}

void PMOD_SPI_init(void)
{
	PMOD_SPI_CLOCK_init();
	spi_m_async_init(&PMOD_SPI, SERCOM1);
	PMOD_SPI_PORT_init();
}

void DADC_I2C_PORT_init(void)
{

	gpio_set_pin_pull_mode(DADC_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(DADC_SDA, PINMUX_PA08D_SERCOM2_PAD0);

	gpio_set_pin_pull_mode(DADC_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(DADC_SCL, PINMUX_PA09D_SERCOM2_PAD1);
}

void DADC_I2C_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM2_bit(MCLK);
}

void DADC_I2C_init(void)
{
	DADC_I2C_CLOCK_init();
	i2c_m_async_init(&DADC_I2C, SERCOM2);
	DADC_I2C_PORT_init();
}

void PMON_I2C_PORT_init(void)
{

	gpio_set_pin_pull_mode(UC_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(UC_SDA, PINMUX_PA22C_SERCOM3_PAD0);

	gpio_set_pin_pull_mode(UC_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(UC_SCL, PINMUX_PA23C_SERCOM3_PAD1);
}

void PMON_I2C_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM3_bit(MCLK);
}

void PMON_I2C_init(void)
{
	PMON_I2C_CLOCK_init();
	i2c_m_async_init(&PMON_I2C, SERCOM3);
	PMON_I2C_PORT_init();
}

void PSD_SPI_PORT_init(void)
{

	gpio_set_pin_level(SD_MOSI,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SD_MOSI, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SD_MOSI, PINMUX_PA12D_SERCOM4_PAD0);

	gpio_set_pin_level(SD_SCLK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SD_SCLK, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SD_SCLK, PINMUX_PA13D_SERCOM4_PAD1);

	// Set pin direction to input
	gpio_set_pin_direction(SD_MISO, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(SD_MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SD_MISO, PINMUX_PA15D_SERCOM4_PAD3);
}

void PSD_SPI_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_SLOW, CONF_GCLK_SERCOM4_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM4_bit(MCLK);
}

void PSD_SPI_init(void)
{
	PSD_SPI_CLOCK_init();
	spi_m_async_init(&PSD_SPI, SERCOM4);
	PSD_SPI_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_CTRL_CLOCK_init()
{

	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_SLOW, CONF_GCLK_SERCOM5_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM5_bit(MCLK);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_CTRL_PORT_init()
{

	gpio_set_pin_function(UTXFRX, PINMUX_PB22D_SERCOM5_PAD2);

	gpio_set_pin_function(URXFTX, PINMUX_PB23D_SERCOM5_PAD3);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_CTRL_init(void)
{
	USART_CTRL_CLOCK_init();
	usart_async_init(&USART_CTRL, SERCOM5, USART_CTRL_buffer, USART_CTRL_BUFFER_SIZE, (void *)NULL);
	USART_CTRL_PORT_init();
}

/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	hri_mclk_set_APBCMASK_TC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TC0_GCLK_ID, CONF_GCLK_TC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	timer_init(&TIMER_0, TC0, _tc_get_timer());
}

void CAN_CTRL_PORT_init(void)
{

	gpio_set_pin_function(CANRX, PINMUX_PA25G_CAN0_RX);

	gpio_set_pin_function(CANTX, PINMUX_PA24G_CAN0_TX);
}
/**
 * \brief CAN initialization function
 *
 * Enables CAN peripheral, clocks and initializes CAN driver
 */
void CAN_CTRL_init(void)
{
	hri_mclk_set_AHBMASK_CAN0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, CAN0_GCLK_ID, CONF_GCLK_CAN0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	can_async_init(&CAN_CTRL, CAN0);
	CAN_CTRL_PORT_init();
}

void system_init(void)
{
	init_mcu();

	// GPIO on PB02

	gpio_set_pin_level(J35_EN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(J35_EN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(J35_EN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB03

	gpio_set_pin_level(J34_EN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(J34_EN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(J34_EN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB08

	gpio_set_pin_level(SPR7,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SPR7, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SPR7, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB09

	gpio_set_pin_level(SPR8,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SPR8, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SPR8, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB11

	gpio_set_pin_level(P_CS,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(P_CS, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(P_CS, GPIO_PIN_FUNCTION_OFF);

	VMON_ADC_init();

	PMOD_SPI_init();

	DADC_I2C_init();

	PMON_I2C_init();

	PSD_SPI_init();
	USART_CTRL_init();

	TIMER_0_init();
	CAN_CTRL_init();
}
