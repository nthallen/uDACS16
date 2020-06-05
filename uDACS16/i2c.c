/** @file i2c.c uDACS16 */
#include "driver_temp.h"
#include <utils.h>
#include <hal_init.h>
#include <hal_i2c_m_async.h>
#include <stdint.h>
#include "uDACS_pins.h"
#include "i2c.h"
#include "subbus.h"

static bool i2c_enabled = I2C_ENABLE_DEFAULT;
static struct io_descriptor *I2C_io;
static volatile bool I2C_txfr_complete = true;
static volatile bool I2C_error_seen = false;
/** i2c error codes are defined in hal/include/hpl_i2c_m_sync.h
 *  named I2C_ERR_* and I2C_OK
 */
static volatile int32_t I2C_error = I2C_OK;
static volatile uint8_t pm_ov_status = 0;
#define PM_SLAVE_ADDR 0x67
#define PM_OVERFLOW 1
#define PM_UNDERFLOW 2

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes);
static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes);

/**
 * These addresses belong to the I2C module
 * 0x20 R:  I2C_Status
 * 0x21 R:  AIN0
 * 0x22 R:  AIN1
 * 0x23 R:  AIN2
 * 0x24 R:  AIN3
 * 0x25 R:  AIN4
 * 0x26 R:  AIN5
 * 0x27 R:  AIN6
 * 0x28 R:  AIN7
 * 0x29 R:  AD5665 TBD
 */
static subbus_cache_word_t i2c_cache[I2C_HIGH_ADDR-I2C_BASE_ADDR+1] = {
  // Value, Wvalue, readable, was_read, writable, written, dynamic
  // I2C Status I2C_STATUS_NREGS
  { 0, 0, true,  false,  false, false, false }, // Offset 0: R: I2C Status
  // ADS registers I2C_ADS_NREGS
  { 0, 0, true,  false, false, false, false },  // AIN0
  { 0, 0, true,  false, false, false, false },  // AIN1
  { 0, 0, true,  false, false, false, false },  // AIN2
  { 0, 0, true,  false, false, false, false },  // AIN3
  { 0, 0, true,  false, false, false, false },  // AIN4
  { 0, 0, true,  false, false, false, false },  // AIN5
  { 0, 0, true,  false, false, false, false },  // AIN6
  { 0, 0, true,  false, false, false, false },  // AIN7
  { 0, 0, true,  false, false, false, false },  // N_reads before conversion complete
  // AD5665: TBD
};

enum ads_state_t {ads_t1_init, ads_t1_init_tx, ads_t1_read_cfg,
                  ads_t1_read_cfg_tx, ads_t1_reg0, ads_t1_read_adc,
                  ads_t1_read_adc_tx,
                  ads_t2_init, ads_t2_init_tx, ads_t2_read_cfg,
                  ads_t2_read_cfg_tx, ads_t2_reg0, ads_t2_read_adc,
                  ads_t2_read_adc_tx};
static enum ads_state_t ads_state = ads_t1_init;
static uint16_t ads_n_reads;
/** Write to config register [01] 0x83 0x03:
 *   0x01: Pointer register value specifying config register
 *   0x8303:
 *     OS[15] = 1: Single Conversion
 *     MUX[14:12] = 000: AIN0/AIN1 => HeaterV
 *     PGA[11:9] = 001: FSR = +/- 4.096V
 *     MODE[8] = 1: Single shot conversion
 *     DR[7:5] = 000: 8 SPS
 *     COMP_MODE[4] = 0: Default/Don't Care
 *     COMP_POL[3] = 0: Default/Don't Care
 *     COMP_LAT[2] = 0: Default/Don't Care
 *     COMP_QUE[1:0] = 11: Disable comparator and set ALERT to high impedance
 */
static uint8_t ads_t1_cmd[3] = { 0x01, 0x83, 0x03 };
/** Write to config register [01] 0xB3 0x03:
 *   0x01: Pointer register value specifying config register
 *   0xB303:
 *     OS[15] = 1: Single Conversion
 *     MUX[14:12] = 011: AIN2/AIN3 => IRSetV
 *     PGA[11:9] = 001: FSR = +/- 4.096V
 *     MODE[8] = 1: Single shot conversion
 *     DR[7:5] = 000: 8 SPS
 *     COMP_MODE[4] = 0: Default/Don't Care
 *     COMP_POL[3] = 0: Default/Don't Care
 *     COMP_LAT[2] = 0: Default/Don't Care
 *     COMP_QUE[1:0] = 11: Disable comparator and set ALERT to high impedance
 */
static uint8_t ads_t2_cmd[3] = { 0x01, 0xB3, 0x03 };
static uint8_t ads_r0_prep[1] = { 0x00 };
static uint8_t ads_ibuf[2];
static uint8_t ads_slave_addr[4] = { 0x48, 0x49, 0x4A, 0x4B }; // 7-bit device address
static int ads_slave_index = 0 ;

/**
 * @return true if the bus is free and available for another device
 */
static bool ads1115_poll(void) {
  switch (ads_state) {
    case ads_t1_init: // Start to convert HeaterV
      ads_n_reads = 0;
      i2c_write(ads_slave_addr[ads_slave_index], ads_t1_cmd, 3);
      ads_state = ads_t1_init_tx;
      return false;
    case ads_t1_init_tx: // Release bus after starting write
	  if (ads_slave_index <3 ) {
		  ++ads_slave_index;
		  ads_state = ads_t1_init;
	  } else {
		  ads_slave_index = 0;
		  ads_state = ads_t1_read_cfg;
	  }
      return true;
    case ads_t1_read_cfg: // Start read from config register
      i2c_read(ads_slave_addr[ads_slave_index], ads_ibuf, 2);
      ads_state = ads_t1_read_cfg_tx;
      return false;
    case ads_t1_read_cfg_tx: // If high bit is set, conversion is complete
      if (ads_ibuf[0] & 0x80) {
        ads_state = ads_t1_reg0;
      } else {
        ++ads_n_reads;
        ads_state = ads_t1_read_cfg;
      }
      return true;
    case ads_t1_reg0: // Write pointer register to read from conversion reg [0]
      i2c_write(ads_slave_addr[ads_slave_index], ads_r0_prep, 1);
      ads_state = ads_t1_read_adc;
      return false;
    case ads_t1_read_adc: // Start read from conversion reg
      i2c_read(ads_slave_addr[ads_slave_index], ads_ibuf, 2);
      ads_state = ads_t1_read_adc_tx;
      return false;
    case ads_t1_read_adc_tx: // Save converted value
      sb_cache_update(i2c_cache, I2C_ADS_OFFSET + ads_slave_index * 2 + (ads_slave_index >= 2 ? 1 : 0), 
		(ads_ibuf[0] << 8) | ads_ibuf[1]);
      sb_cache_update(i2c_cache, I2C_ADS_OFFSET+8, ads_n_reads);
	  if (ads_slave_index <3 ) {
		  ++ads_slave_index;
		  ads_state = ads_t1_read_cfg;
	  } else {
		  ads_slave_index = 0;
		  ads_state = ads_t2_init;
	  }
      return true;
    case ads_t2_init:
      ads_n_reads = 0;
      i2c_write(ads_slave_addr[ads_slave_index], ads_t2_cmd, 4);
      ads_state = ads_t2_init_tx;
      return false;
    case ads_t2_init_tx:
	  if (ads_slave_index <3 ) {
		  ++ads_slave_index;
		  ads_state = ads_t2_init;
	  } else {
		  ads_slave_index = 0;
		  ads_state = ads_t2_read_cfg;
	  }
      return true;
    case ads_t2_read_cfg:
      i2c_read(ads_slave_addr[ads_slave_index], ads_ibuf, 2);
      ads_state = ads_t2_read_cfg_tx;
      return false;
    case ads_t2_read_cfg_tx:
      if (ads_ibuf[0] & 0x80) {
        ads_state = ads_t2_reg0;
      } else {
        ++ads_n_reads;
        ads_state = ads_t2_read_cfg;
      }
      return true;
    case ads_t2_reg0:
      i2c_write(ads_slave_addr[ads_slave_index], ads_r0_prep, 1);
      ads_state = ads_t2_read_adc;
      return false;
    case ads_t2_read_adc:
      i2c_read(ads_slave_addr[ads_slave_index], ads_ibuf, 2);
      ads_state = ads_t2_read_adc_tx;
      return false;
    case ads_t2_read_adc_tx:
      sb_cache_update(i2c_cache, I2C_ADS_OFFSET + ads_slave_index * 2 + (ads_slave_index < 2 ? 1 : 0), 
		(ads_ibuf[0] << 8) | ads_ibuf[1]);
      sb_cache_update(i2c_cache, I2C_ADS_OFFSET+8, ads_n_reads);
	  if (ads_slave_index <3 ) {
		  ++ads_slave_index;
		  ads_state = ads_t2_read_cfg;
	  } else {
		  ads_slave_index = 0;
		  ads_state = ads_t1_init;
	  }
      return true;
    default:
      assert(false, __FILE__, __LINE__);
  }
  return true;
}

static bool ad5665_poll() {
  return true;
}

static void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&DADC_I2C, i2c_addr, I2C_M_SEVEN);
  io_write(I2C_io, obuf, nbytes);
}

static void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&DADC_I2C, i2c_addr, I2C_M_SEVEN);
  io_read(I2C_io, ibuf, nbytes);
}

void i2c_enable(bool value) {
  i2c_enabled = value;
}

#define I2C_INTFLAG_ERROR (1<<7)

static void I2C_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  I2C_txfr_complete = true;
  I2C_error_seen = true;
  I2C_error = error;
  if (sb_cache_was_read(i2c_cache, I2C_STATUS_OFFSET)) {
    sb_cache_update(i2c_cache, I2C_STATUS_OFFSET, 0);
  }
  if (I2C_error >= -7 && I2C_error <= -2) {
    uint16_t val = i2c_cache[I2C_STATUS_OFFSET].cache;
    val |= (1 << (7+I2C_error));
    sb_cache_update(i2c_cache, I2C_STATUS_OFFSET, val);
  }
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(DADC_I2C.device.hw, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(DADC_I2C.device.hw, I2C_INTFLAG_ERROR);
  }
}

static void I2C_txfr_completed(struct i2c_m_async_desc *const i2c) {
  I2C_txfr_complete = true;
}

static void i2c_reset() {
  if (!sb_i2c.initialized) {
    // I2C_init(); // Called from driver_init
    i2c_m_async_get_io_descriptor(&DADC_I2C, &I2C_io);
    i2c_m_async_enable(&DADC_I2C);
    i2c_m_async_register_callback(&DADC_I2C, I2C_M_ASYNC_ERROR, (FUNC_PTR)I2C_async_error);
    i2c_m_async_register_callback(&DADC_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);
    i2c_m_async_register_callback(&DADC_I2C, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)I2C_txfr_completed);

    sb_i2c.initialized = true;
  }
}

// static void i2c_may_use() {
  // i2c_m_async_set_slaveaddr(&I2C, 0x12, I2C_M_SEVEN);
  // io_write(I2C_io, I2C_example_str, 12);
// }

enum i2c_state_t {i2c_ads1115, i2c_ad5665 };
static enum i2c_state_t i2c_state = i2c_ads1115;

void i2c_poll(void) {
  enum i2c_state_t input_state = i2c_state;
  while (i2c_enabled && I2C_txfr_complete) {
    switch (i2c_state) {
      case i2c_ads1115:
        if (ads1115_poll()) {
          i2c_state = i2c_ad5665;
        }
        break;
      case i2c_ad5665:
        if (ad5665_poll()) {
          i2c_state = i2c_ads1115;
        }
        break;
      default:
        assert(false, __FILE__, __LINE__);
    }
    if (i2c_state == input_state) break;
  }
}

subbus_driver_t sb_i2c = {
  I2C_BASE_ADDR, I2C_HIGH_ADDR, // address range
  i2c_cache,
  i2c_reset,
  i2c_poll,
  0, // Dynamic function
  false // initialized
};
