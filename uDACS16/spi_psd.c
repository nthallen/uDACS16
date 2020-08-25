/************************************************************************/
/* 3:16 PM 6/27/2020	file spi_PSD.c                                  */
/************************************************************************/
#include <peripheral_clk_config.h>
#include <hal_spi_m_async.h>
#include <hal_gpio.h>
#include <hpl_pm_base.h>
#include <hpl_gclk_base.h>
// #include <hal_ext_irq.h>  // ??
#include "uDACS_pins.h"
#include "spi_psd.h"
#include "subbus.h"
#include "rtc_timer.h"

#define pow2(X) (float)(1<<X)

static volatile bool PSD_SPI_txfr_complete = true;
static bool spi_enabled = PSD_SPI_ENABLE_DEFAULT;
// static bool ms5607_enabled = PSD_SPI_MS5607_ENABLED;
static bool sd_enabled = PSD_SPI_SD_ENABLED;

void spi_enable(bool value) {
  spi_enabled = value;
}

static inline void chip_select(uint8_t pin) {
  gpio_set_pin_level(pin, false);
}
static inline void chip_deselect(uint8_t pin) {
  gpio_set_pin_level(pin, true);
}

static void complete_cb_PSD_SPI(const struct spi_m_async_descriptor *const io_descr) {
  PSD_SPI_txfr_complete = true;
}

static uint8_t spi_read_data[PSD_SPI_MAX_READ_LENGTH];


// We will use mode 0 for MS5607
// Might need different mode for SDcard: TBD
static enum spi_transfer_mode spi_current_transfer_mode = SPI_MODE_0;

static void start_spi_transfer(uint8_t pin, uint8_t const *txbuf, int length, enum spi_transfer_mode mode) {
  assert(length <= PSD_SPI_MAX_READ_LENGTH,__FILE__,__LINE__);
  if (spi_current_transfer_mode != mode) {
    spi_m_async_disable(&PSD_SPI);
    spi_m_async_set_mode(&PSD_SPI, mode);
    spi_current_transfer_mode = mode;
    spi_m_async_enable(&PSD_SPI);
  }
  chip_select(pin);
  PSD_SPI_txfr_complete = false;
  spi_m_async_transfer(&PSD_SPI, txbuf, spi_read_data, length);
}

typedef struct {
    uint8_t cmd[3];	// cmd - Coefficient read commands
} ms5607_prom_read;

ms5607_prom_read ms_read_coef[8] = {
  { {0xA0, 0x00, 0x00} }, // Read Manuf info (?)
  { {0xA2, 0x00, 0x00} }, // Read Coeff C1
  { {0xA4, 0x00, 0x00} }, // Read Coeff C2
  { {0xA5, 0x00, 0x00} }, // Read Coeff C3
  { {0xA8, 0x00, 0x00} }, // Read Coeff C4
  { {0xAA, 0x00, 0x00} }, // Read Coeff C5
  { {0xAC, 0x00, 0x00} }, // Read Coeff C6
  { {0xAE, 0x00, 0x00} }  // Read CRC (possible future checks)
};
static uint8_t coef_num = 0;

 static uint8_t ms5607_adc_read[4] = {
  MS_ADC_READ, 		// Send ADC Read command
  0x00, 0x00, 0x00 	// read back ADC on SDO
};

static uint8_t ms_reset_cmd = MS_RESET;
static uint8_t ms_conv_D1_osr;
static uint8_t ms_conv_D2_osr;

/* These addresses belong to the PSD_SPI module
 * 0x10 R:  PL: 16b Compensated Pressure LSW
 * 0x11 R:  PM: 16b Compensated Pressure MSB
 * 0x12 R:  TL: 16b Compensated Temperature LSW
 * 0x13 R:  TM: 16b Compensated Temperature MSB
 * 0x14 R:  C1: 16b Pressure sensitivity | SENST1
 * 0x15 R:  C2: 16b Pressure offset | OFFT1
 * 0x16 R:  C3: 16b Temperature coeff. of pressure sensitivity | TCS
 * 0x17 R:  C4: 16b Temperature coefficient of pressure offset | TCO
 * 0x18 R:  C5: 16b Reference temperature | TREF
 * 0x19 R:  C6: 16b Temperature coefficient of the temperature | TEMPSENS
 * 0x1A R:  D1L:16b Raw Pressure LSW
 * 0x1B R:  D1M:16b Raw Pressure MSB
 * 0x1C R:  D2L:16b Raw Temperature LSW
 * 0x1D R:  D2M:16b Raw Temperature MSB
 * 0x1E R:  OSR:16b OSR select (0:256, 1:512, 2:1024, 3:2048, 4:4096)
 */
static subbus_cache_word_t psd_spi_cache[PSD_SPI_HIGH_ADDR-PSD_SPI_BASE_ADDR+1] = {
  { 0, 0, true,  false, false,  false, false }, // Offset 0x00: R: Compensated Pressure LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x01: R: Compensated Pressure MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x02: R: Compensated Temperature LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x03: R: Compensated Temperature MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x04: R: C1: Pressure sensitivity | SENST1
  { 0, 0, true,  false, false,  false, false }, // Offset 0x05: R: C2: Pressure offset | OFFT1
  { 0, 0, true,  false, false,  false, false }, // Offset 0x06: R: C3: Temperature coefficient of pressure sensitivity | TCS
  { 0, 0, true,  false, false,  false, false }, // Offset 0x07: R: C4: Temperature coefficient of pressure offset | TCO
  { 0, 0, true,  false, false,  false, false }, // Offset 0x08: R: C5: Reference temperature | TREF
  { 0, 0, true,  false, false,  false, false }, // Offset 0x09: R: C6: Temperature coefficient of the temperature | TEMPSENS
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0A: R: Raw Pressure LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0B: R: Raw Pressure MSB
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0C: R: Raw Temperature LSW
  { 0, 0, true,  false, false,  false, false }, // Offset 0x0D: R: Raw Temperature MSB
  { 4, 0, true,  false,  true,  false, false }, // Offset 0x0E: RW: OSR select (0:256, 1:512, 2:1024, 3:2048, 4:4096)
// .cache	.wvalue	.readable	.was_read	.writable	.written	.dynamic
};

/*
 ****************************************************
 *	MS5607 Driver State Machine
 *	ms5607_init - Reset MS5607 to initialize
 *	ms5607_readcal - Read Calibration data (6)
 *	ms5607_convp - Send Convert Pressure Command
 *	ms5607_readp - Read Pressure
 *	ms5607_convt - Send Convert Temperature Command
 *	ms5607_readt - Read Temperature
 */
enum ms5607_state_t {
        ms5607_init, ms5607_init_tx, ms5607_init_delay,
        ms5607_readcal, ms5607_readcal_tx,
        ms5607_convp, ms5607_convp_tx, ms5607_convp_delay,
        ms5607_readp, ms5607_readp_tx,
        ms5607_convt, ms5607_convt_tx, ms5607_convt_delay,
        ms5607_readt, ms5607_readt_tx};

typedef struct {
  bool enabled;
  enum ms5607_state_t state;
  uint8_t cs_pin;
  uint32_t D1;		// Raw Pressure
  uint32_t D2;		// Raw Temperature
  uint16_t cal[8];
  float P; 	// Compensated Pressure
  float T; 	// Compensated Temperature
  uint32_t endtime;
  uint32_t delay;
//  uint16_t current;
} ms5607_poll_def;

static ms5607_poll_def ms5607 = {
    PSD_SPI_MS5607_ENABLED, ms5607_init, P_CS
//	, 0, 0
//	, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
//	, 0, 0
};

/**
 * poll_ms5607() is only called when PSD_SPI_txfr_complete is non-zero
 *    and SPI bus is free
 * @return true if we are relinquishing the SPI bus
 */
static bool poll_ms5607() {
  // uint32_t delay = 0x00000000;
  float dT; 	// difference between actual and measured temperature
  float OFF; 	// offset at actual temperature
  float SENS; 	// sensitivity at actual temperature
  if (!PSD_SPI_MS5607_ENABLED) return true;
  switch (ms5607.state) {
    case ms5607_init:
      start_spi_transfer(ms5607.cs_pin, &ms_reset_cmd, 1, SPI_MODE_0);
      ms5607.state = ms5607_init_tx;
      return false;

    case ms5607_init_tx:
      ms5607.endtime = rtc_current_count + ( 3 * RTC_COUNTS_PER_MSEC ); // >2.8mS
      ms5607.state = ms5607_init_delay;
      return false;

    case ms5607_init_delay:
      if ( rtc_current_count <= ms5607.endtime ) return false;
      // for (int j=0; j < 5500; ++j);  // 5500 about 3ms delay
      chip_deselect(ms5607.cs_pin);
      ms5607.state = ms5607_readcal;
      return true;

    case ms5607_readcal:
      // need to send 8 / receive 12 bytes to get
      // 1 x 8b Manuf info (TBD)
      // 6 x 16b coefficients
      // 1 x 8b CRC (TBD)
      start_spi_transfer(ms5607.cs_pin, (&ms_read_coef[coef_num].cmd[0]), 3, SPI_MODE_0);
      ms5607.state = ms5607_readcal_tx;
      return false;

    case ms5607_readcal_tx:
      // read coeff from spi_read_data
      // possible check here for 0xFE on every third byte
      ms5607.cal[coef_num] = ( // update ms5607 struct
         (((uint16_t)spi_read_data[1])<<8)
          | ((uint16_t)spi_read_data[2]));
      if (coef_num > 0) psd_spi_cache[coef_num + 3].cache = ms5607.cal[coef_num]; // update cache
      chip_deselect(ms5607.cs_pin);
      ms5607.state = ms5607_readp;
      if (coef_num++ < 7) ms5607.state = ms5607_readcal;
      return true;

// Probably need _delay state here...

    // return loop here
    case ms5607_convp:
      ms_conv_D1_osr = MS_CONV_D1 + (2 * psd_spi_cache[0x0E].cache); // Update CONV_D1 cmd with OSR offset
      start_spi_transfer(ms5607.cs_pin, &ms_conv_D1_osr, 1, SPI_MODE_0); // Send Convert D1 (P)
      //  ADC OSR=256 	600us ~1ms
      //  ADC OSR=512 	1.17ms ~2ms
      //  ADC OSR=1024	2.28ms ~3ms
      //  ADC OSR=2056	4.54ms ~5ms
      //  ADC OSR=4096	9.04ms ~10ms
      switch (psd_spi_cache[0x0E].cache) {
        case 0:	ms5607.delay = 1 * RTC_COUNTS_PER_MSEC ; break; // 1mS
        case 1:	ms5607.delay = 2 * RTC_COUNTS_PER_MSEC ; break; // 2mS
        case 2:	ms5607.delay = 3 * RTC_COUNTS_PER_MSEC ; break; // 3mS
        case 3:	ms5607.delay = 5 * RTC_COUNTS_PER_MSEC ; break; // 5mS
        default: ms5607.delay = 10 * RTC_COUNTS_PER_MSEC ; break; // 10mS : case 4 or default
      }
      ms5607.state = ms5607_convp_tx;
      return false;

    case ms5607_convp_tx:
      ms5607.endtime = rtc_current_count + ms5607.delay ;
      ms5607.state = ms5607_convp_delay;
      return false;

    case ms5607_convp_delay:
  	  if ( rtc_current_count <= ms5607.endtime ) return false;
      // for (int j=0; j < 16500; ++j); // 16500 about 10ms delay
      chip_deselect(ms5607.cs_pin);
      ms5607.state = ms5607_readp;
      return true;

    case ms5607_readp:
      start_spi_transfer(ms5607.cs_pin, ms5607_adc_read, 4, SPI_MODE_0);
      ms5607.state = ms5607_readp_tx;
      return false;

    case ms5607_readp_tx:
      psd_spi_cache[0x0A].cache = (  // read P LSW from spi_read_data and update cache
         (((uint16_t)spi_read_data[2])<<8)
        | ((uint16_t)spi_read_data[3]));
      psd_spi_cache[0x0B].cache = (  // read P MSB from spi_read_data and update cache
          ((uint16_t)spi_read_data[1]));
      ms5607.D1 = (((uint32_t)psd_spi_cache[0x0B].cache)<<16)
        | ((uint32_t)psd_spi_cache[0x0A].cache);  // Update ms5607.D1 for P calculation
      chip_deselect(ms5607.cs_pin);
      ms5607.state = ms5607_convt;
      return true;

    case ms5607_convt:
      ms_conv_D2_osr = MS_CONV_D2 + (2 * psd_spi_cache[0x0E].cache); // Update CONV_D1 cmd with OSR offset
      start_spi_transfer(ms5607.cs_pin, &ms_conv_D2_osr, 1, SPI_MODE_0); // Send Convert D2 (T)
      ms5607.state = ms5607_convt_tx;
      return false;

    case ms5607_convt_tx:
      ms5607.endtime = rtc_current_count + ms5607.delay ;
      ms5607.state = ms5607_convt_delay;
      return false;

    case ms5607_convt_delay:
  	  if ( rtc_current_count <= ms5607.endtime ) return false;
      // for (int j=0; j < 16500; ++j); // 16500 about 10ms delay
      chip_deselect(ms5607.cs_pin);
      ms5607.state = ms5607_readt;
      return true;

    case ms5607_readt:
      start_spi_transfer(ms5607.cs_pin, ms5607_adc_read, 4, SPI_MODE_0);
      ms5607.state = ms5607_readt_tx;
      return false;

    case ms5607_readt_tx:
      psd_spi_cache[0x0C].cache = (
         (((uint16_t)spi_read_data[2])<<8)
        | ((uint16_t)spi_read_data[3])); // read T LSW from spi_read_data and update cache
      psd_spi_cache[0x0D].cache = (
          ((uint16_t)spi_read_data[1])); // read T MSB from spi_read_data and update cache
      ms5607.D2 = ((uint32_t)psd_spi_cache[0x0D].cache)<<16
        | ((uint32_t)psd_spi_cache[0x0C].cache);	// Update ms5607.D2 for T calculation

      // Perform Compensation calculations here and update cache
      dT = ((float)ms5607.D2) - ((float)(ms5607.cal[5]) * pow2(8));
      OFF = ((float)(ms5607.cal[2]) * pow2(17)) + (dT * ((float)(ms5607.cal[4]))) / pow2(6);
      SENS = ((float)(ms5607.cal[1]) * pow2(16)) + (dT * ((float)(ms5607.cal[3]))) / pow2(7);
      ms5607.T = ( 2000 + ((dT * (float)(ms5607.cal[6])) / pow2(23))) / 100;  // degC
      ms5607.P = ((((float)(ms5607.D1) * SENS) / pow2(21)) - OFF) / pow2(15) / 100; // mBar

      sb_cache_update32(psd_spi_cache, 0, &ms5607.P);	// Update cache P
      sb_cache_update32(psd_spi_cache, 2, &ms5607.T);	// Update cache T

      chip_deselect(ms5607.cs_pin);
      ms5607.state = ms5607_convp;	// return to perform next P reading
      return true;

    default:
      assert(false, __FILE__, __LINE__);
   }
   return true;
}

// For P and T
// start_spi_transfer to start P conversion then -check
//   MS_CONV_D1 + (2 * MS_OSR_OFFS) // OFFS can change for different OSR. would be on cache.
//
// set endtime -check
// delay until done then start_spi_transfer to read P addr 0x00 -check

// store read P results AND start_spi_transfer to start T conversion
// set new endtime
// delay until done then start_spi_transfer to read T addr 0x00
// store read T results go back to Read P : give up the BUS


/**
 * Only called when SPI bus is free
 * @return true if we have relinquished the bus and cleared our chip select
 */
static bool poll_sd(void) {
  if (!sd_enabled) return true;
//	sd card engine
  return true;
}

enum spi_state_t {spi_ms5607, spi_sd};
static enum spi_state_t spi_state = spi_ms5607;

void psd_spi_poll(void) {
  if (!spi_enabled) return;
  if (PSD_SPI_txfr_complete) {
    switch (spi_state) {
      case spi_ms5607:
        if (poll_ms5607()) {
          spi_state = spi_sd;
        }
        break;
      case spi_sd:
        if (poll_sd()) {
          spi_state = spi_ms5607;
        }
        break;
      default:
        assert(false, __FILE__, __LINE__);
    }
  }
}

static void psd_spi_reset(void) {
  if (!sb_spi.initialized) {
    PSD_SPI_init();
    spi_m_async_register_callback(&PSD_SPI, SPI_M_ASYNC_CB_XFER, (FUNC_PTR)complete_cb_PSD_SPI);	// PSD_SPI_... ???
    spi_m_async_enable(&PSD_SPI);
    sb_spi.initialized = true;
  }
}

subbus_driver_t sb_spi = {
  PSD_SPI_BASE_ADDR, PSD_SPI_HIGH_ADDR, // address range
  psd_spi_cache,
  psd_spi_reset,
  psd_spi_poll,
  0, // Dynamic function
  false
};
