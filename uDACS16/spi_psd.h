/************************************************************************/
/* 1:29 PM 7/8/2020 file spi_PSD.h                                      */
/************************************************************************/
#ifndef PSD_SPI_H_INCLUDED
#define PSD_SPI_H_INCLUDED
#include <hal_spi_m_async.h>
#include "subbus.h"

#define PSD_SPI_ENABLE_DEFAULT true
#define PSD_SPI_MS5607_ENABLED true
#define PSD_SPI_SD_ENABLED false

#define PSD_SPI_MAX_READ_LENGTH 32
#define PSD_SPI_BASE_ADDR 0x10
#define PSD_SPI_HIGH_ADDR 0x1E

#define ADC_LSW_OFFSET(x) (5+2*(x))
#define ADC_MSB_OFFSET(x) (6+2*(x))

// MS5607 Commands
#define MS_RESET 0x1E // ADC reset command
#define MS_ADC_READ 0x00 // ADC read command
#define MS_CONV_D1 0x40 // ADC D1 conversion command
#define MS_CONV_D2 0x50 // ADC D2 conversion command
#define MS_OSR_OFFS 0x04 // ADC OSR offset (Default = 4096)
//	#define MS_ADC_256 0x00 // ADC OSR=256
//	#define MS_ADC_512 0x02 // ADC OSR=512
//	#define MS_ADC_1024 0x04 // ADC OSR=1024
//	#define MS_ADC_2048 0x06 // ADC OSR=2056
//	#define MS_ADC_4096 0x08 // ADC OSR=4096
#define MS_PROM_RD 0xA0 // Prom read command

extern subbus_driver_t sb_spi;
void spi_enable(bool value);
struct spi_m_async_descriptor PSD_SPI;
void PSD_SPI_init(void);

#endif
