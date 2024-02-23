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

/* In the current configuration, we are using the PMON_I2C to talk to the ICM20948 IMU.
 * In the most recent Atmel START config, this uses SERCOM3, and the clock rate is 100 KHz.
 * This can be changed by defining the symbol CONF_SERCOM_3_I2CM_BAUD, which is currently
 * defined to be 100000.
 */

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
  { 0, 0, true, false, false, false,  true },  // Offset 0x06: R: ICM_FIFO
  { 0, 0, true, false, false, false,  true },  // Offset 0x06: R: ICM_FIFO_DIAG
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
  uint8_t status;
  uint16_t fifo[I2C_ICM_FIFO_SIZE];
} icm_fifo;
#define I2C_ICM_FIFO_FULL 1
#define I2C_ICM_FIFO_WRAPPED 2

static void fifo_init(void) {
  icm_fifo.head = icm_fifo.tail = icm_fifo.nw =
    icm_fifo.alloc = icm_fifo.status = 0;
  sb_cache_update(i2c_icm_cache, I2C_ICM_FIFO_COUNT_OFFSET,
    0x3FFF & (uint16_t)icm_fifo.nw);
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
    assert(icm_fifo.head == icm_fifo.tail,__FILE__,__LINE__);
    icm_fifo.head = icm_fifo.tail = 0;
    icm_fifo.status = 0;
    available = I2C_ICM_FIFO_SIZE;
  } if (icm_fifo.status & I2C_ICM_FIFO_WRAPPED) {
    available = icm_fifo.head - icm_fifo.tail;
  } else {    
    available = I2C_ICM_FIFO_SIZE - icm_fifo.tail;
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
  if (n_words == 0) return;
  icm_fifo.alloc = 0;
  icm_fifo.nw += n_words;
  for (;n_words > 0; --n_words) {
    // icm_swap16(icm_fifo.fifo[icm_fifo.tail]);
    ++icm_fifo.tail;
  }
  if (icm_fifo.tail >= I2C_ICM_FIFO_SIZE) {
    icm_fifo.tail = 0;
    icm_fifo.status |= I2C_ICM_FIFO_WRAPPED;
  }
  if (icm_fifo.head == icm_fifo.tail) {
    icm_fifo.status |= I2C_ICM_FIFO_FULL;
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
    if (icm_fifo.head >= I2C_ICM_FIFO_SIZE) {
      icm_fifo.head = 0;
      icm_fifo.status &= ~(I2C_ICM_FIFO_FULL|I2C_ICM_FIFO_WRAPPED);
    } else {
      icm_fifo.status &= ~I2C_ICM_FIFO_FULL;
    }
  }
  return val;
}

enum icm_state_t {
    s_mode0_init, s_mode0_init_1, s_mode0_init_1a, s_mode0_init_2,
      s_mode0_init_3, s_mode0_init_4, s_mode0_init_5,
    s_mode0_operate, s_mode0_operate_2, s_mode0_operate_4,
    s_mode1_init, s_mode1_init_1, s_mode1_init_2,
    s_mode1_operate, s_mode1_operate_1, s_mode1_operate_2,
    s_mode2_init, s_mode2_init_0, s_mode2_init_1, s_mode2_init_2,
      s_mode2_init_3, s_mode2_init_4, s_mode2_init_4a, s_mode2_init_5,
      s_mode2_init_6, s_mode2_init_7,
    s_mode2_operate, s_mode2_operate_1, s_mode2_operate_2,
       s_mode2_operate_3, s_mode2_operate_4, s_mode2_operate_5,
       s_mode2_operate_6,
    s_mode2_shutdown, s_mode2_shutdown_1, s_mode2_shutdown_2,
       s_mode2_shutdown_2a,
       s_mode2_shutdown_3, s_mode2_shutdown_4, s_mode2_shutdown_5,
       s_mode2_shutdown_6,
    s_set_accel_cfg, s_set_accel_cfg_1, s_set_accel_cfg_2,
    s_set_accel_cfg_1a, s_set_accel_cfg_1b, s_set_accel_cfg_1c,
    s_icm_read,
    s_no_state};

static enum icm_state_t icm_state = s_mode0_init;
static enum icm_state_t icm_accelcfg_rtn = s_mode0_init;
static enum icm_state_t new_mode_init = s_mode0_init;
static enum icm_state_t icm_shutdown_mode = s_no_state;

/* Variable related to Mode 1 operation */
static uint8_t icm_accel_ibuf[6]; // Could/Should put in icm_poll_def struct?

/* Variables for Mode 2 operation */
static uint8_t icm_int_status_buf;
static uint16_t icm_fifo_count;
static uint8_t icm_accel_cfg = ICM_MODE1_ACCEL_CFG; // Default, for mode 1.
static uint8_t icm_accel_cfg_rb; // readback

static uint32_t endtime = 0x00000000; // Could/Should put in icm_poll_def struct?
// static uint8_t whoami = 0x00;  // ICM20948 whoami response


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
  icm_accel_cfg = icm_accel_config | (icm_fs_cfg<<1);
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

static int cfg_icm_accel(enum icm_state_t rtn) {
  icm_accelcfg_rtn = rtn;
  icm_state = s_set_accel_cfg;
  return false;
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
        new_mode_init = s_mode0_init;
        break;
      case ICM_MODE_SLOW:
        new_mode_init = s_mode1_init;
        break;
      case ICM_MODE_FAST:
        new_mode_init = s_mode2_init;
        break;
      default: // restore current value
        // Invalid mode, do nothing
        return false;
    }
    set_cur_mode(icm_new_mode);
    icm_state = (icm_shutdown_mode != s_no_state) ?
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

static uint8_t icm_write_123_buf[3];
/**
 * Initiates the write of a one-byte address to the ICM20948_ADDR device and
 * updates icm_state.
 * @param reg_addr The register address in the current page
 * @param next The next state for icm_state
 * @return false as a syntactical convenience
 */
static int icm_write_1(uint8_t reg_addr, enum icm_state_t next) {
  icm_write_123_buf[0] = reg_addr;
  i2c_icm_write(ICM20948_ADDR, icm_write_123_buf, 1);
  icm_state = next;
  return 0;
}
/**
 * Initiates the write of a two-byte word to the ICM20948_ADDR device and
 * updates icm_state.
 * @param reg_addr The register address in the current page
 * @param val The value to write to the register
 * @param next The next state for icm_state
 * @return false as a syntactical convenience
 */
static int icm_write_2(uint8_t reg_addr, uint8_t val, enum icm_state_t next) {
  icm_write_123_buf[0] = reg_addr;
  icm_write_123_buf[1] = val;
  i2c_icm_write(ICM20948_ADDR, icm_write_123_buf, 2);
  icm_state = next;
  return 0;
}
/**
 * Initiates the write of a two-byte word to the ICM20948_ADDR device and
 * updates icm_state.
 * @param reg_addr The register address in the current page
 * @param val1 The value to write to the register
 * @param val2 The value to write to the following register
 * @param next The next state for icm_state
 * @return false as a syntactical convenience
 */
static int icm_write_3(uint8_t reg_addr, uint8_t val1, uint8_t val2, enum icm_state_t next) {
  icm_write_123_buf[0] = reg_addr;
  icm_write_123_buf[1] = val1;
  icm_write_123_buf[2] = val2;
  i2c_icm_write(ICM20948_ADDR, icm_write_123_buf, 3);
  icm_state = next;
  return 0;
}

static uint8_t *icm_read_dest;
static int icm_read_size;
static enum icm_state_t icm_read_next;
static int icm_read(uint8_t reg_addr, uint8_t *dest, int size, enum icm_state_t next) {
  icm_read_dest = dest;
  icm_read_size = size;
  icm_read_next = next;
  return icm_write_1(reg_addr, s_icm_read);
}  
      

/**
 * @return true if the bus is free and available for another
 *   device or will be when the current transfer is completed.
 */
static bool icm20948_poll(void) {
  if (!i2c_icm_enabled) return true;
  switch (icm_state) {
    case s_mode0_init:
      set_cur_mode(ICM_MODE_NO);
      update_cache_mode();
      icm_shutdown_mode = s_no_state;
      return icm_write_2(REG_BANK_SEL, 0, s_mode0_init_1);
    case s_mode0_init_1:  // Skip Check for ICM20948 for now
      return icm_write_2(PWR_MGMT_1, 0x09, s_mode0_init_2);
    case s_mode0_init_2:
      return icm_write_2(PWR_MGMT_2, 0x07, s_mode0_init_3);
    case s_mode0_init_3:
      set_accel_cfg(ICM_MODE0_ACCEL_CFG);
      return cfg_icm_accel(s_mode0_init_4);
    case s_mode0_init_4:
      return icm_write_2(LP_CONFIG, 0x40,s_mode0_init_5);
    case s_mode0_init_5:
      icm_write_2(PWR_MGMT_1, 0x49, s_mode0_operate);
      return true;

    case s_mode0_operate:
      if (check_icm_mode())
        return true;
      return icm_read(INT_STATUS_2, &icm_int_status_buf, 1, s_mode0_operate_2);
    case s_mode0_operate_2:
      return icm_read(FIFO_COUNTH, (uint8_t*)&icm_fifo_count, 2, s_mode0_operate_4);
    case s_mode0_operate_4:
      icm_swap16(icm_fifo_count);
      icm_fifo_count &= 0x1FFF;
      if (icm_int_status_buf&0xF)
        icm_fifo_count |= 0x8000;
      sb_cache_update(i2c_icm_cache, I2C_ICM_FIFO_DIAG_OFFSET, icm_fifo_count);
      icm_state = s_mode0_operate;
      return true;

    case s_mode1_init:   // Check for ICM20948
      set_cur_mode(ICM_MODE_SLOW);
      update_cache_mode();
      icm_shutdown_mode = s_no_state;
      return icm_write_2(REG_BANK_SEL, 0, s_mode0_init_1);
    case s_mode1_init_1:  // Skip Check for ICM20948 for now
      return icm_write_2(PWR_MGMT_1, 0x09, s_mode1_init_2); // Just wake up
    case s_mode1_init_2:
      set_accel_cfg(ICM_MODE1_ACCEL_CFG);
      return cfg_icm_accel(s_mode1_operate);

    case s_mode1_operate:
      if (check_icm_mode())
        return true;
      return icm_read(ACCEL_XOUT_H, icm_accel_ibuf, 6, s_mode1_operate_2);
    case s_mode1_operate_2: // Read is finished
      sb_cache_update(i2c_icm_cache, 0x01,
        (((uint16_t)icm_accel_ibuf[0])<<8) | ((uint16_t)icm_accel_ibuf[1]));
      sb_cache_update(i2c_icm_cache, 0x02,
        (((uint16_t)icm_accel_ibuf[2])<<8) | ((uint16_t)icm_accel_ibuf[3]));
      sb_cache_update(i2c_icm_cache, 0x03,
        (((uint16_t)icm_accel_ibuf[4])<<8) | ((uint16_t)icm_accel_ibuf[5]));
      icm_state = s_mode1_operate;
      return true;

    case s_mode2_init:
      set_cur_mode(ICM_MODE_FAST);
      update_cache_mode();
      fifo_init();
      icm_shutdown_mode = s_mode2_shutdown;
      return icm_write_2(REG_BANK_SEL, 0, s_mode2_init_0);
    case s_mode2_init_0:
      return icm_write_2(LP_CONFIG, 0x20, s_mode2_init_1);
    case s_mode2_init_1:
      return icm_write_2(PWR_MGMT_1, 0x09, s_mode2_init_2);
    case s_mode2_init_2:
      return icm_write_2(PWR_MGMT_2, 0x07, s_mode2_init_3);
    case s_mode2_init_3:
      set_accel_cfg(ICM_MODE2_ACCEL_CFG);
      return cfg_icm_accel(s_mode2_init_4);
    case s_mode2_init_4:
      return icm_write_2(INT_ENABLE_2, 1, s_mode2_init_4a);
    case s_mode2_init_4a:
      return icm_write_2(USER_CTRL, 0x40, s_mode2_init_5);
    case s_mode2_init_5:
      return icm_write_2(FIFO_MODE, 1, s_mode2_init_6);
    case s_mode2_init_6:
      return icm_write_3(FIFO_EN_2, 0x10, 1, s_mode2_init_7);
    case s_mode2_init_7:
      return icm_write_2(FIFO_RST, 0, s_mode2_operate);
    
    case s_mode2_operate:
      if (check_icm_mode())
        return true;
      return icm_read(INT_STATUS_2, &icm_int_status_buf, 1, s_mode2_operate_2);
    case s_mode2_operate_2:
      if (icm_int_status_buf&0xF) {
        new_mode_init = s_mode0_init;
        icm_state = s_mode2_shutdown;
        return false;
      }
      return icm_read(FIFO_COUNTH, (uint8_t*)&icm_fifo_count, 2, s_mode2_operate_4);
    case s_mode2_operate_4:
      icm_swap16(icm_fifo_count);
      icm_fifo_count &= 0x1FFF;
      icm_fifo_count /= 2; // Now in words
      icm_fifo_count = fifo_allocate(icm_fifo_count);
      icm_fifo_count /= I2C_ICM_FIFO_WORDS_PER_SAMPLE; // Now complete in records
      if (icm_fifo_count) {
        return icm_read(FIFO_R_W, (uint8_t*)&icm_fifo.fifo[icm_fifo.tail],
          icm_fifo_count*6, s_mode2_operate_6);
      } else {
        icm_state = s_mode2_operate;
        return true;
      }
    case s_mode2_operate_6:
      sb_cache_update(i2c_icm_cache, I2C_ICM_FIFO_DIAG_OFFSET,
         icm_fifo_count+i2c_icm_cache[I2C_ICM_FIFO_DIAG_OFFSET].cache);
      // i2c_icm_fifo_commit(icm_fifo_count*3);
      icm_state = s_mode2_operate;
      return true;

    case s_mode2_shutdown:
      set_accel_cfg(ICM_MODE0_ACCEL_CFG);
      return cfg_icm_accel(s_mode2_shutdown_1);
    case s_mode2_shutdown_1:
      // i2c_icm_write(ICM20948_ADDR, icm_int_dis, 2);
      icm_state = s_mode2_shutdown_2;
      return false;
    case s_mode2_shutdown_2:
      endtime = rtc_current_count + 3000*RTC_COUNTS_PER_MSEC;
      icm_write_3(FIFO_EN_2, 0, 1, s_mode2_shutdown_2a);
      return true;
    case s_mode2_shutdown_2a:
      return icm_read(INT_STATUS_2, &icm_int_status_buf, 1, s_mode2_shutdown_3);
    case s_mode2_shutdown_3:
      if ((icm_int_status_buf & 0xF) && rtc_current_count < endtime) {
        icm_state = s_mode2_shutdown_2a;
        return false;
      }
      return icm_read(FIFO_COUNTH, (uint8_t*)&icm_fifo_count, 2, s_mode2_shutdown_5);
    case s_mode2_shutdown_5:
      icm_swap16(icm_fifo_count);
      icm_fifo_count &= 0x1FFF;
      sb_cache_update(i2c_icm_cache, I2C_ICM_FIFO_DIAG_OFFSET, icm_fifo_count);
      if ((icm_fifo_count == 0) || (rtc_current_count >= endtime)) {
        icm_state = s_mode2_shutdown_6;
      } else {
        icm_state = s_mode2_shutdown_3;
      }
      return false;
    case s_mode2_shutdown_6:
      fifo_init();
      sb_cache_update(i2c_icm_cache, I2C_ICM_FIFO_DIAG_OFFSET, 0);
      icm_write_2(FIFO_RST, 0, new_mode_init);
      return true;
 
    // s_set_accel_cfg is a subroutine invoke via
    // cmf_icm_accel(cfg, rtn);
    case s_set_accel_cfg:
      return icm_write_2(REG_BANK_SEL, 2<<4, s_set_accel_cfg_1);
    case s_set_accel_cfg_1:
      return icm_write_2(ACCEL_CONFIG, icm_accel_cfg, s_set_accel_cfg_1a);
    case s_set_accel_cfg_1a:
      return icm_read(ACCEL_CONFIG, &icm_accel_cfg_rb, 1, s_set_accel_cfg_1c);
    case s_set_accel_cfg_1c:
      // sb_cache_update(i2c_icm_cache, I2C_ICM_FIFO_DIAG_OFFSET, icm_accel_cfg_rb);
      // fall through
    case s_set_accel_cfg_2:
      return icm_write_2(REG_BANK_SEL, 0x00, icm_accelcfg_rtn);
      
    case s_icm_read:
      i2c_icm_read(ICM20948_ADDR, icm_read_dest, icm_read_size);
      icm_state = icm_read_next;
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
  /* FOR DEBUG PURPOSES ONLY: */
  if (sb_cache_was_read(i2c_icm_cache, I2C_ICM_FIFO_DIAG_OFFSET)) {
    sb_cache_update(i2c_icm_cache, I2C_ICM_FIFO_DIAG_OFFSET, 0);
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
