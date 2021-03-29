/** @file i2c_icm20948.c for uDACS16 */
#include "driver_temp.h"
#include <utils.h>
#include <hal_init.h>
#include <hal_i2c_m_async.h>
#include <stdint.h>
#include "uDACS_pins.h"
#include "i2c_icm20948.h"
#include "subbus.h"
#include "rtc_timer.h"

static bool i2c_icm_enabled = I2C_ICM_ENABLE_DEFAULT;
static struct io_descriptor *I2C_ICM_io;
static volatile bool I2C_ICM_txfr_complete = true;
static volatile bool I2C_ICM_error_seen = false;
/** i2c error codes are defined in hal/include/hpl_i2c_m_sync.h
 *  named I2C_ERR_* and I2C_OK
 */
static volatile int32_t I2C_ICM_error = I2C_OK;
/*  Is this needed??
static volatile uint8_t pm_ov_status = 0;
#define PM_SLAVE_ADDR 0x67  // ??
#define PM_OVERFLOW 1 // ??
#define PM_UNDERFLOW 2 // ??
 */
static void i2c_icm_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes);
static void i2c_icm_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes);

/**
 * These addresses belong to the I2C ICM module
 * 0x60 R : Offset 0x00: R: I2C Status
 * 0x61 R : Offset 0x01: R: ACCEL_X
 * 0x62 R : Offset 0x02: R: ACCEL_Y
 * 0x63 R : Offset 0x03: R: ACCEL_Z
 */
 
static subbus_cache_word_t i2c_icm_cache[I2C_ICM_HIGH_ADDR-I2C_ICM_BASE_ADDR+1] = {
  // Cache , Wvalue, readable, was_read, writable, written, dynamic
  // I2C Status I2C_ICM_STATUS_NREGS
  { 0, 0, true,  false,  false, false, false }, // Offset 0x00: R: I2C Status
  // ICM registers I2C_ICM_NREGS
  { 0, 0, true,  false, false, false, false },  // Offset 0x01: R: ACCEL_X
  { 0, 0, true,  false, false, false, false },  // Offset 0x02: R: ACCEL_Y
  { 0, 0, true,  false, false, false, false },  // Offset 0x03: R: ACCEL_Z
//  { 0, 0, true,  false, false, false, false },  // 
};

enum icm_state_t {icm_init, icm_init_tx, 
				  icm_reset, icm_reset_delay, 
				  icm_rdacc, icm_rdacc_tx};

static enum icm_state_t icm_state = icm_init;
 
static uint8_t icm_bank0_cmd[2] = { REG_BANK_SEL, 0x00 }; // Select Reg Bank 0: [0x7F] 0x00:
static uint8_t icm_reset_cmd[2] = { PWR_MGMT_1, 0x81 }; // ICM20948 Reset cmd: [0x06] 0x81:
static uint8_t icm_wake_cmd[2] = { PWR_MGMT_1, 0x01 }; // ICM20948 Reset cmd: [0x06] 0x81:
static uint8_t icm_accx_cmd[1] = { ACCEL_XOUT_H }; // First Accel Data Reg Addr [0x2D] 

static uint8_t icm_accel_ibuf[6]; // Could/Should put in icm_poll_def struct?
uint32_t endtime = 0x00000000; // Could/Should put in icm_poll_def struct?
uint8_t whoami = 0x00;  // ICM20948 whoami response

/**
 * @return true if the bus is free and available for another device
 */
static bool icm20948_poll(void) {
  if (!i2c_icm_enabled) return true;
  switch (icm_state) {
    case icm_init:   // Check for ICM20948
	// Possibly check for multiple ICM20948
      i2c_icm_write( ICM20948_ADDR, icm_bank0_cmd, 2); // Sel Reg Bank 0
//      endtime = rtc_current_count + ( 10 * RTC_COUNTS_PER_MSEC ); // delay 10ms 
      icm_state = icm_init_tx;
      return false;
	
	case icm_init_tx:  // Skip Check for ICM20948 for now
//      if ( rtc_current_count <= endtime ) return false;
      i2c_icm_write( ICM20948_ADDR, icm_wake_cmd, 2); // Just wake up
//      endtime = rtc_current_count + ( 10 * RTC_COUNTS_PER_MSEC ); // delay 10ms 
      icm_state = icm_reset; 
	  return true;
	
	case icm_reset:  // NO Reset to start yet
//      if ( rtc_current_count <= endtime ) return false;
      i2c_icm_write( ICM20948_ADDR, icm_bank0_cmd, 2);
//      endtime = rtc_current_count + ( 10 * RTC_COUNTS_PER_MSEC ); // delay 10ms 
      icm_state = icm_reset_delay;
      return true; // Release bus after starting write
	  
    case icm_reset_delay: // When delay over, send accel register addr 0x2D
//      if ( rtc_current_count <= endtime ) return false;
      i2c_icm_write( ICM20948_ADDR, icm_accx_cmd, 1);
//      endtime = rtc_current_count + ( 1 * RTC_COUNTS_PER_MSEC ); // delay 1ms 
      icm_state = icm_rdacc;
      return false;
	  
    case icm_rdacc: // Read accel values and cache
//      if ( rtc_current_count <= endtime ) return false;
	  i2c_icm_read( ICM20948_ADDR, icm_accel_ibuf, 6);
      i2c_icm_cache[0x01].cache = (  // get 16b Accel x and update cache
         (((uint16_t)icm_accel_ibuf[0])<<8)
        | ((uint16_t)icm_accel_ibuf[1]));
      i2c_icm_cache[0x02].cache = (  // get 16b Accel y and update cache
         (((uint16_t)icm_accel_ibuf[2])<<8)
        | ((uint16_t)icm_accel_ibuf[3]));
      i2c_icm_cache[0x03].cache = (  // get 16b Accel z and update cache
         (((uint16_t)icm_accel_ibuf[4])<<8)
        | ((uint16_t)icm_accel_ibuf[5]));
	  icm_state = icm_reset_delay; 
      return true;	  
    default:
      assert(false, __FILE__, __LINE__);
  }
  return true;
}

static void i2c_icm_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes) {
  assert(I2C_ICM_txfr_complete, __FILE__, __LINE__);
  I2C_ICM_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&PMON_I2C, i2c_addr, I2C_M_SEVEN);
  io_write(I2C_ICM_io, obuf, nbytes);
}

static void i2c_icm_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes) {
  assert(I2C_ICM_txfr_complete, __FILE__, __LINE__);
  I2C_ICM_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&PMON_I2C, i2c_addr, I2C_M_SEVEN);
  io_read(I2C_ICM_io, ibuf, nbytes);
}

void i2c_icm_enable(bool value) {
  i2c_icm_enabled = value;
}

#define I2C_ICM_INTFLAG_ERROR (1<<7)

static void I2C_ICM_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  I2C_ICM_txfr_complete = true;
  I2C_ICM_error_seen = true;
  I2C_ICM_error = error;
  if (sb_cache_was_read(i2c_icm_cache, I2C_ICM_STATUS_OFFSET)) {
    sb_cache_update(i2c_icm_cache, I2C_ICM_STATUS_OFFSET, 0);
  }
  if (I2C_ICM_error >= -7 && I2C_ICM_error <= -2) {
    uint16_t val = i2c_icm_cache[I2C_ICM_STATUS_OFFSET].cache;
    val |= (1 << (7+I2C_ICM_error));
    sb_cache_update(i2c_icm_cache, I2C_ICM_STATUS_OFFSET, val);
  }
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(PMON_I2C.device.hw, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(PMON_I2C.device.hw, I2C_ICM_INTFLAG_ERROR);
  }
}

static void I2C_ICM_txfr_completed(struct i2c_m_async_desc *const i2c) {
  I2C_ICM_txfr_complete = true;
}

static void i2c_icm_reset() {
  if (!sb_i2c_icm.initialized) {
    // PMON_I2C_init(); // Called from driver_temp
    i2c_m_async_get_io_descriptor(&PMON_I2C, &I2C_ICM_io);
    i2c_m_async_enable(&PMON_I2C);
    i2c_m_async_register_callback(&PMON_I2C, I2C_M_ASYNC_ERROR, (FUNC_PTR)I2C_ICM_async_error);
    i2c_m_async_register_callback(&PMON_I2C, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)I2C_ICM_txfr_completed);
    i2c_m_async_register_callback(&PMON_I2C, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)I2C_ICM_txfr_completed);

    sb_i2c_icm.initialized = true;
  }
}

void i2c_icm_poll(void) {
  if (i2c_icm_enabled && I2C_ICM_txfr_complete) {
    icm20948_poll(); // might need to make use of returned bool
  }
}

subbus_driver_t sb_i2c_icm = {
  I2C_ICM_BASE_ADDR, I2C_ICM_HIGH_ADDR, // address range
  i2c_icm_cache,
  i2c_icm_reset,
  i2c_icm_poll,
  0, // Dynamic function
  false // initialized
};
