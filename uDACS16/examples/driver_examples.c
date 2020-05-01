/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

static void convert_cb_VMON_ADC(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
}

/**
 * Example of using VMON_ADC to generate waveform.
 */
void VMON_ADC_example(void)
{
	adc_async_register_callback(&VMON_ADC, 0, ADC_ASYNC_CONVERT_CB, convert_cb_VMON_ADC);
	adc_async_enable_channel(&VMON_ADC, 0);
	adc_async_start_conversion(&VMON_ADC);
}

static struct timer_task TIMER_0_task1, TIMER_0_task2;
/**
 * Example of using TIMER_0.
 */
static void TIMER_0_task1_cb(const struct timer_task *const timer_task)
{
}

static void TIMER_0_task2_cb(const struct timer_task *const timer_task)
{
}

void TIMER_0_example(void)
{
	TIMER_0_task1.interval = 100;
	TIMER_0_task1.cb       = TIMER_0_task1_cb;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;
	TIMER_0_task2.interval = 200;
	TIMER_0_task2.cb       = TIMER_0_task2_cb;
	TIMER_0_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_add_task(&TIMER_0, &TIMER_0_task2);
	timer_start(&TIMER_0);
}

/**
 * Example of using PMOD_SPI to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_PMOD_SPI[12] = "Hello World!";

static void complete_cb_PMOD_SPI(const struct spi_m_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void PMOD_SPI_example(void)
{
	struct io_descriptor *io;
	spi_m_async_get_io_descriptor(&PMOD_SPI, &io);

	spi_m_async_register_callback(&PMOD_SPI, SPI_M_ASYNC_CB_XFER, (FUNC_PTR)complete_cb_PMOD_SPI);
	spi_m_async_enable(&PMOD_SPI);
	io_write(io, example_PMOD_SPI, 12);
}

static uint8_t DADC_I2C_example_str[12] = "Hello World!";

void DADC_I2C_tx_complete(struct i2c_m_async_desc *const i2c)
{
}

void DADC_I2C_example(void)
{
	struct io_descriptor *DADC_I2C_io;

	i2c_m_async_get_io_descriptor(&DADC_I2C, &DADC_I2C_io);
	i2c_m_async_enable(&DADC_I2C);
	i2c_m_async_register_callback(&DADC_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)DADC_I2C_tx_complete);
	i2c_m_async_set_slaveaddr(&DADC_I2C, 0x12, I2C_M_SEVEN);

	io_write(DADC_I2C_io, DADC_I2C_example_str, 12);
}

static uint8_t PMON_I2C_example_str[12] = "Hello World!";

void PMON_I2C_tx_complete(struct i2c_m_async_desc *const i2c)
{
}

void PMON_I2C_example(void)
{
	struct io_descriptor *PMON_I2C_io;

	i2c_m_async_get_io_descriptor(&PMON_I2C, &PMON_I2C_io);
	i2c_m_async_enable(&PMON_I2C);
	i2c_m_async_register_callback(&PMON_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)PMON_I2C_tx_complete);
	i2c_m_async_set_slaveaddr(&PMON_I2C, 0x12, I2C_M_SEVEN);

	io_write(PMON_I2C_io, PMON_I2C_example_str, 12);
}

/**
 * Example of using PSD_SPI to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_PSD_SPI[12] = "Hello World!";

static void complete_cb_PSD_SPI(const struct spi_m_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void PSD_SPI_example(void)
{
	struct io_descriptor *io;
	spi_m_async_get_io_descriptor(&PSD_SPI, &io);

	spi_m_async_register_callback(&PSD_SPI, SPI_M_ASYNC_CB_XFER, (FUNC_PTR)complete_cb_PSD_SPI);
	spi_m_async_enable(&PSD_SPI);
	io_write(io, example_PSD_SPI, 12);
}

/**
 * Example of using USART_CTRL to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_USART_CTRL[12] = "Hello World!";

/*static void tx_cb_USART_CTRL(const struct usart_async_descriptor *const io_descr)
{
	// Transfer completed 
}*/

void USART_CTRL_example(void)
{
	struct io_descriptor *io;

	/*usart_async_register_callback(&USART_CTRL, USART_ASYNC_TXC_CB, tx_cb_USART_CTRL);
	usart_async_register_callback(&USART_CTRL, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&USART_CTRL, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&USART_CTRL, &io);
	usart_async_enable(&USART_CTRL);

	io_write(io, example_USART_CTRL, 12);
}

void CAN_CTRL_tx_callback(struct can_async_descriptor *const descr)
{
	(void)descr;
}
void CAN_CTRL_rx_callback(struct can_async_descriptor *const descr)
{
	struct can_message msg;
	uint8_t            data[64];
	msg.data = data;
	can_async_read(descr, &msg);
	return;
}

/**
 * Example of using CAN_CTRL to Encrypt/Decrypt datas.
 */
void CAN_CTRL_example(void)
{
	struct can_message msg;
	struct can_filter  filter;
	uint8_t            send_data[4];
	send_data[0] = 0x00;
	send_data[1] = 0x01;
	send_data[2] = 0x02;
	send_data[3] = 0x03;

	msg.id   = 0x45A;
	msg.type = CAN_TYPE_DATA;
	msg.data = send_data;
	msg.len  = 4;
	msg.fmt  = CAN_FMT_STDID;
	can_async_register_callback(&CAN_CTRL, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_CTRL_tx_callback);
	can_async_enable(&CAN_CTRL);

	/**
	 * CAN_CTRL_tx_callback callback should be invoked after call
	 * can_async_write, and remote device should recieve message with ID=0x45A
	 */
	can_async_write(&CAN_CTRL, &msg);

	msg.id  = 0x100000A5;
	msg.fmt = CAN_FMT_EXTID;
	/**
	 * remote device should recieve message with ID=0x100000A5
	 */
	can_async_write(&CAN_CTRL, &msg);

	/**
	 * CAN_CTRL_rx_callback callback should be invoked after call
	 * can_async_set_filter and remote device send CAN Message with the same
	 * content as the filter.
	 */
	can_async_register_callback(&CAN_CTRL, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_CTRL_rx_callback);
	filter.id   = 0x469;
	filter.mask = 0;
	can_async_set_filter(&CAN_CTRL, 0, CAN_FMT_STDID, &filter);

	filter.id   = 0x10000096;
	filter.mask = 0;
	can_async_set_filter(&CAN_CTRL, 1, CAN_FMT_EXTID, &filter);
}
