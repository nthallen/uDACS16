/** @file i2c_icm20948.c for uDACS16 */
#include "driver_temp.h"
#include <utils.h>
#include <hal_init.h>
#include <hal_i2c_m_async.h>
#include <stdint.h>
#include "serial_num.h"
#include "i2c_icm20948.h"
#include "subbus.h"
#include "rtc_timer.h"

#ifdef HAVE_VIBE_SENSOR

#if CONF_SERCOM_5_USART_BAUD != 115200
#error Baud rate for USB/Serial connection not configure to 115200
#endif

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
 * 0x64 RW: Offset 0x04: RW: ICM_MODE
 * 0x65 R : Offset 0x07: R: ICM_FIFO_DIAG
 * 0x66 R : Offset 0x05: R: ICM_FIFO_COUNT
 * 0x67 R : Offset 0x06: R: ICM_FIFO
 */

static subbus_cache_word_t i2c_icm_cache[I2C_ICM_HIGH_ADDR-I2C_ICM_BASE_ADDR+1] = {
  // Cache , Wvalue, readable, was_read, writable, written, dynamic
  // I2C Status I2C_ICM_STATUS_NREGS
  { 0, 0, true, false,  false, false, false }, // Offset 0x00: R: I2C Status
  // ICM registers I2C_ICM_NREGS
  { 0, 0, true, false, false, false, false },  // Offset 0x01: R: ACCEL_X
  { 0, 0, true, false, false, false, false },  // Offset 0x02: R: ACCEL_Y
  { 0, 0, true, false, false, false, false },  // Offset 0x03: R: ACCEL_Z
  // ICM FIFO registers: I2C_ICM_FIFO_NREGS
  { 0, 0, true, false,  true, false, false },  // Offset 0x04: R/W: ICM_MODE
  { 0, 0, true, false, false, false, false },  // Offset 0x05: R: ICM_FIFO_COUNT
  { 0, 0, true, false, false, false,  true }   // Offset 0x06: R: ICM_FIFO
};

/**
 * head points to the next output word. Must be less
 *   than I2C_ICM_FIFO_SIZE
 * tail is where the next input word is stored. Must
 *   be less than I2C_ICM_FIFO_SIZE
 * nw records the number of words in the fifo
 * Empty condition: head == tail && nw == 0
 * Full condition: head == tail && nw != 0 (I2C_ICM_FIFO_SIZE)
 */
static struct {
  int head, tail, nw, alloc;
  uint16_t fifo[I2C_ICM_FIFO_SIZE];
  uint8_t status;
} icm_fifo;
#define I2C_ICM_FIFO_FULL 1
#define I2C_ICM_FIFO_WRAPPED 2

static void fifo_init(void) {
  icm_fifo.head = icm_fifo.tail = icm_fifo.nw =
    icm_fifo.alloc = 0;
}

/**
 * Decision to discard data will not be made here but in
 * the state machine.
 * @param n_words requested allocation
 * @return The number of words that can be written into the fifo
 */
static uint16_t fifo_allocate(uint16_t n_words) {
  int available;
  if (icm_fifo.nw == 0) {
    icm_fifo.head = icm_fifo.tail = 0;
    icm_fifo.status = 0;
    available = I2C_ICM_FIFO_SIZE;
  } if (icm_fifo.status & I2C_ICM_FIFO_WRAPPED) {
    available = I2C_ICM_FIFO_SIZE - icm_fifo.tail;
  } else if (icm_fifo.status & I2C_ICM_FIFO_FULL) {
    available = 0;
  }
  if (available > n_words) {
    available = n_words;
  }
  icm_fifo.alloc = available;
  return available;
}

/**
 * @param n_words the number of words to commit
 * Data has previously been written into empty space starting
 * at icm_fifo.tail, not beyond the end of the buffer.
 * n_words must have been previously allocated via
 * fifo_allocate()
 */
static void i2c_icm_fifo_commit(uint16_t n_words) {
  assert(n_words <= icm_fifo.alloc,__FILE__,__LINE__);
  icm_fifo.alloc = 0;
  icm_fifo.nw += n_words;
  for (;n_words > 0; --n_words) {
    icm_swap16(icm_fifo.fifo[icm_fifo.tail]);
    ++icm_fifo.tail;
  }
  if (icm_fifo.tail >= I2C_ICM_FIFO_SIZE) {
    icm_fifo.tail = 0;
    icm_fifo.status |= I2C_ICM_FIFO_WRAPPED;
  }
  sb_cache_update(i2c_icm_cache, I2C_ICM_FIFO_COUNT_OFFSET,
    0x3FFF & (uint16_t)icm_fifo.nw);
}

static uint16_t i2c_icm_fifo_pop(void) {
  uint16_t val;
  if (icm_fifo.nw < 1) {
    val = 0;
    // set underflow bit
  } else {
    --icm_fifo.nw;
    val = icm_fifo.fifo[icm_fifo.head++];
    if (icm_fifo.head >= I2C_ICM_FIFO_SIZE)
      icm_fifo.head = 0;
  }
  return val;
}

enum icm_state_t {
    icm_mode0_init, icm_mode0_init_1, icm_mode0_init_2,
    icm_mode0_operate,
    icm_mode1_init, icm_mode1_init_1, icm_mode1_init_2,
    icm_mode1_operate, icm_mode1_operate_1, icm_mode1_operate_2,
    icm_mode2_init, icm_mode2_init_1, icm_mode2_init_2, icm_mode2_init_3,
      icm_mode2_init_4, icm_mode2_init_5, icm_mode2_init_6, icm_mode2_init_7,
    icm_mode2_operate, icm_mode2_operate_1, icm_mode2_operate_2,
       icm_mode2_operate_3, icm_mode2_operate_4, icm_mode2_operate_5,
       icm_mode2_operate_6,
    icm_mode2_shutdown, icm_mode2_shutdown_1, icm_mode2_shutdown_2,
       icm_mode2_shutdown_3,
    icm_set_accel_cfg, icm_set_accel_cfg_1, icm_set_accel_cfg_2,
    icm_no_state};

static enum icm_state_t icm_state = icm_mode0_init;
static enum icm_state_t icm_accelcfg_rtn = icm_mode0_init;
static enum icm_state_t new_mode_init = icm_mode0_init;
static enum icm_state_t icm_shutdown_mode = icm_no_state;

static uint8_t icm_bank0_cmd[2] = { REG_BANK_SEL, 0x00 }; // Select Reg Bank 0: [0x7F] 0x00:
static uint8_t icm_bank2_cmd[2] = { REG_BANK_SEL, 2<<4 }; // Select Reg Bank 2: [0x7F] 0x00:
// static uint8_t icm_reset_cmd[2] = { PWR_MGMT_1, 0x81 }; // ICM20948 Reset cmd: [0x06] 0x81:
static uint8_t icm_disable_gyro[2] = { PWR_MGMT_2, 0x07 }; // Disable Gyro = 7, Disable Accel = 7<<3
static uint8_t icm_wake_cmd[2] = { PWR_MGMT_1, 0x09 }; // Sleep = 0x40, Best clock = 0x01, TEMP_DIS = 0x08
static uint8_t icm_sleep_cmd[2] = { PWR_MGMT_1, 0x49 }; // Sleep = 0x40, Best clock = 0x01, TEMP_DIS = 0x08
static uint8_t icm_accx_cmd[1] = { ACCEL_XOUT_H }; // First Accel Data Reg Addr [0x2D]
static uint8_t icm_accel_cfg[2] = { ACCEL_CONFIG, ICM_MODE1_ACCEL_CFG }; // Default, for mode 1.
static uint8_t icm_int_en_2[2] = { INT_ENABLE_2, 1 };
static uint8_t icm_accx2fifo[3] = { FIFO_EN_2, 0x10, 1 }; // [0x67] + FIFO_RST
static uint8_t icm_fifo_dis[3] = { FIFO_EN_2, 0, 1 }; // [0x67] + FIFO_RST
static uint8_t icm_fifo_nrst[2] = { FIFO_RST, 0 }; // [0x68] FIFO_RST
static uint8_t icm_fifo_mode[2] = {FIFO_MODE, 1};
static uint8_t icm_int_dis[2] = { INT_ENABLE_2, 0 };
static uint8_t icm_int_status[1] = {INT_STATUS_2};
static uint8_t icm_fifo_count_reg[1] = {FIFO_COUNTH};
static uint8_t icm_fifo_reg[1] = {FIFO_R_W};

/* Variable related to Mode 1 operation */
static uint8_t icm_accel_ibuf[6]; // Could/Should put in icm_poll_def struct?

/* Variables for Mode 2 operation */
static uint8_t icm_int_status_buf;
static uint16_t icm_fifo_count;

uint32_t endtime = 0x00000000; // Could/Should put in icm_poll_def struct?
uint8_t whoami = 0x00;  // ICM20948 whoami response


/* Variables and functions related to changing mode and full scale
 * range.
 */
static uint8_t icm_accel_config = 0x01;
static uint8_t icm_fs_cfg = ICM_FS_2G;
static uint16_t icm_new_mode = ICM_MODE_NO;
static uint16_t icm_cur_mode = ICM_MODE_NO;
static uint16_t icm_new_fs = ICM_FS_2G;
static uint16_t icm_have_new_mode = 0;
static uint16_t icm_have_new_fs = 0;

static void set_accel_cfg_cmd(void) {
  icm_accel_cfg[1] = icm_accel_config | (icm_fs_cfg<<1);
}
static void set_accel_fs(uint8_t fs) {
  icm_fs_cfg = fs&3;
  set_accel_cfg_cmd();
}
static void set_accel_cfg(uint8_t cfg) {
  icm_accel_config = cfg & 0xF9;
  set_accel_cfg_cmd();
}
static void set_cur_mode(uint8_t mode) {
  icm_cur_mode = mode & 7;
}

static void update_cache_mode(void) {
  sb_cache_update(i2c_icm_cache, I2C_ICM_FIFO_OFFSET,
    (icm_cur_mode&7) | (icm_fs_cfg<<3));
}

static void cfg_icm_accel(enum icm_state_t rtn) {
  icm_accelcfg_rtn = rtn;
  icm_state = icm_set_accel_cfg;
}

/**
 * Public mode change interface.
 * Uses the ICM_MODE_* definitions
 */
void i2c_icm_set_mode(uint16_t mode) {
  icm_new_mode = mode;
  icm_have_new_mode = 1;
}
/**
 * Public full scale range change interface.
 * Uses the ICM_FS_* definitions
 */
void i2c_icm_set_fs(uint16_t fs) {
  icm_new_fs = fs;
  icm_have_new_fs = 1;
}

/**
 * Called from within the polling function state machine code.
 * During the mode initialization, icm_shutdown_mode can be
 * defined to a state
 * @return true if a new state has been designated
 */
static bool check_icm_mode(void) {
  if (icm_have_new_mode) {
    icm_have_new_mode = 0;
    switch (icm_new_mode) {
      case ICM_MODE_NO:
        new_mode_init = icm_mode0_init;
        break;
      case ICM_MODE_SLOW:
        new_mode_init = icm_mode1_init;
        break;
      case ICM_MODE_FAST:
        new_mode_init = icm_mode2_init;
        break;
      default: // restore current value
        // Invalid mode, do nothing
        return false;
    }
    set_cur_mode(icm_new_mode);
    icm_state = (icm_shutdown_mode != icm_no_state) ?
      icm_shutdown_mode : new_mode_init;
    return true;
  }
  if (icm_have_new_fs) {
    icm_have_new_fs = 0;
    set_accel_fs(icm_new_fs);
    cfg_icm_accel(icm_state);
    update_cache_mode();
    return true;
  }
  return false;
}


/**
 * @return true if the bus is free and available for another
 *   device or will be when the current transfer is completed.
 */
static bool icm20948_poll(void) {
  if (!i2c_icm_enabled) return true;
  switch (icm_state) {
    case icm_mode0_init:
      set_cur_mode(ICM_MODE_NO);
      update_cache_mode();
      icm_shutdown_mode = icm_no_state;
      i2c_icm_write(ICM20948_ADDR, icm_bank0_cmd, 2);
      icm_state = icm_mode0_init_1;
      return false;
    case icm_mode0_init_1:
      i2c_icm_write(ICM20948_ADDR, icm_disable_gyro, 2);
      icm_state = icm_mode0_init_2;
      return false;
    case icm_mode0_init_2:
      i2c_icm_write(ICM20948_ADDR, icm_sleep_cmd, 2);
      icm_state = icm_mode0_operate;
      return true;

    case icm_mode0_operate:
      check_icm_mode();
      return true;

    case icm_mode1_init:   // Check for ICM20948
      set_cur_mode(ICM_MODE_SLOW);
      update_cache_mode();
      i2c_icm_write( ICM20948_ADDR, icm_bank0_cmd, 2); // Sel Reg Bank 0
      icm_shutdown_mode = icm_no_state;
      icm_state = icm_mode1_init_1;
      return false;
    case icm_mode1_init_1:  // Skip Check for ICM20948 for now
      i2c_icm_write( ICM20948_ADDR, icm_wake_cmd, 2); // Just wake up
      icm_state = icm_mode1_init_2;
      return false;
    case icm_mode1_init_2:
      i2c_icm_write( ICM20948_ADDR, icm_bank0_cmd, 2);
      icm_state = icm_mode1_operate;
      return false;

    case icm_mode1_operate:
      if (check_icm_mode())
        return true;
      i2c_icm_write(ICM20948_ADDR, icm_accx_cmd, 1);
      icm_state = icm_mode1_operate_1;
      return false;
    case icm_mode1_operate_1: // Read accel values and cache
      // if ( rtc_current_count <= endtime ) return false;
      i2c_icm_read(ICM20948_ADDR, icm_accel_ibuf, 6);
      icm_state = icm_mode1_operate_2;
      return false;
    case icm_mode1_operate_2: // Read is finished
      sb_cache_update(i2c_icm_cache, 0x01,
        (((uint16_t)icm_accel_ibuf[0])<<8) | ((uint16_t)icm_accel_ibuf[1]));
      sb_cache_update(i2c_icm_cache, 0x02,
        (((uint16_t)icm_accel_ibuf[2])<<8) | ((uint16_t)icm_accel_ibuf[3]));
      sb_cache_update(i2c_icm_cache, 0x03,
        (((uint16_t)icm_accel_ibuf[4])<<8) | ((uint16_t)icm_accel_ibuf[5]));
      icm_state = icm_mode1_operate;
      return true;

    case icm_mode2_init:
      set_cur_mode(ICM_MODE_FAST);
      update_cache_mode();
      fifo_init();
      i2c_icm_write( ICM20948_ADDR, icm_bank0_cmd, 2); // Sel Reg Bank 0
      icm_shutdown_mode = icm_mode2_shutdown;
      icm_state = icm_mode2_init_1;
      return false;
    case icm_mode2_init_1:
      i2c_icm_write(ICM20948_ADDR, icm_disable_gyro, 2);
      icm_state = icm_mode2_init_2;
      return false;
    case icm_mode2_init_2:
      i2c_icm_write(ICM20948_ADDR, icm_wake_cmd, 2);
      icm_state = icm_mode2_init_3;
      return false;
    case icm_mode2_init_3:
      set_accel_cfg(ICM_MODE2_ACCEL_CFG);
      cfg_icm_accel(icm_mode2_init_4);
      return false;
    case icm_mode2_init_4:
      i2c_icm_write(ICM20948_ADDR, icm_int_en_2, 2);
      icm_state = icm_mode2_init_5;
      return false;
    case icm_mode2_init_5:
      i2c_icm_write(ICM20948_ADDR, icm_fifo_mode, 2);
      icm_state = icm_mode2_init_6;
      return false;
    case icm_mode2_init_6:
      i2c_icm_write(ICM20948_ADDR, icm_accx2fifo, 3);
      icm_state = icm_mode2_init_7;
      return false;
    case icm_mode2_init_7:
      i2c_icm_write(ICM20948_ADDR, icm_fifo_nrst, 2);
      icm_state = icm_mode2_operate;
      return false;
    
    case icm_mode2_operate:
      if (check_icm_mode())
        return true;
      i2c_icm_write(ICM20948_ADDR, icm_int_status, 1);
      icm_state = icm_mode2_operate_1;
      return false;
    case icm_mode2_operate_1: // Read accel values and cache
      i2c_icm_read(ICM20948_ADDR, &icm_int_status_buf, 1);
      icm_state = icm_mode2_operate_2;
      return false;
    case icm_mode2_operate_2:
      if (icm_int_status_buf&0xF) {
        new_mode_init = icm_mode0_init;
        icm_state = icm_mode2_shutdown;
        return false;
      }
      i2c_icm_write(ICM20948_ADDR, icm_fifo_count_reg, 1);
      icm_state = icm_mode2_operate_3;
      return false;
    case icm_mode2_operate_3:
      i2c_icm_read(ICM20948_ADDR, (uint8_t*)&icm_fifo_count, 2);
      icm_state = icm_mode2_operate_4;
      return false;
    case icm_mode2_operate_4:
      icm_swap16(icm_fifo_count);
      icm_fifo_count &= 0x1FFF;
      icm_fifo_count /= 2; // Now in words
      icm_fifo_count = fifo_allocate(icm_fifo_count);
      icm_fifo_count /= I2C_ICM_FIFO_WORDS_PER_SAMPLE; // Now complete in records
      if (icm_fifo_count) {
        i2c_icm_write(ICM20948_ADDR, icm_fifo_reg, 1);
        icm_state = icm_mode2_operate_5;
        return false;
      } else {
        icm_state = icm_mode2_operate;
        return true;
      }
    case icm_mode2_operate_5:
      i2c_icm_read(ICM20948_ADDR,
        (uint8_t*)&icm_fifo.fifo[icm_fifo.tail],
        icm_fifo_count*6);
      icm_state = icm_mode2_operate_6;
      return false;
    case icm_mode2_operate_6:
      i2c_icm_fifo_commit(icm_fifo_count*3);
      icm_state = icm_mode2_operate;
      return true;

    case icm_mode2_shutdown:
      set_accel_cfg(ICM_MODE1_ACCEL_CFG);
      cfg_icm_accel(icm_mode2_shutdown_1);
      return false;
    case icm_mode2_shutdown_1:
      i2c_icm_write(ICM20948_ADDR, icm_int_dis, 2);
      icm_state = icm_mode2_shutdown_2;
      return false;
    case icm_mode2_shutdown_2:
      i2c_icm_write(ICM20948_ADDR, icm_fifo_dis, 3);
      icm_state = icm_mode2_shutdown_3;
      return false;
    case icm_mode2_shutdown_3:
      i2c_icm_write(ICM20948_ADDR, icm_fifo_nrst, 2);
      icm_state = new_mode_init;
      fifo_init();
      return true;
 
    // icm_set_accel_cfg is a subroutine invoke via
    // cmf_icm_accel(cfg, rtn);
    case icm_set_accel_cfg:
      i2c_icm_write( ICM20948_ADDR, icm_bank2_cmd, 2);
      icm_state = icm_set_accel_cfg_1;
      return false;
    case icm_set_accel_cfg_1:
      i2c_icm_write( ICM20948_ADDR, icm_accel_cfg, 2);
      icm_state = icm_set_accel_cfg_2;
      return false;
    case icm_set_accel_cfg_2:
      i2c_icm_write( ICM20948_ADDR, icm_bank0_cmd, 2);
      icm_state = icm_accelcfg_rtn;
      return false;
      
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
    set_accel_fs(0);

    sb_i2c_icm.initialized = true;
  }
}

/**
 * @param offset within i2c_icm_cache
 */
static void i2c_icm_action(uint16_t offset) {
  if (sb_cache_was_read(i2c_icm_cache, I2C_ICM_FIFO_REG_OFFSET)) {
    uint16_t next_val = i2c_icm_fifo_pop();
    sb_cache_update(i2c_icm_cache, I2C_ICM_FIFO_COUNT_OFFSET,
      0x3FFF & (uint16_t)icm_fifo.nw);
    sb_cache_update(i2c_icm_cache, I2C_ICM_FIFO_REG_OFFSET, next_val);
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
  i2c_icm_action, // Dynamic function
  false // initialized
};

#endif
