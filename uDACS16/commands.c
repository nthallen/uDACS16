#include "commands.h"
#include "serial_num.h"
#include "subbus.h"
#include "rtc_timer.h"
#ifdef HAVE_VIBE_SENSOR
#include "i2c_icm20948.h"
#endif

static void commands_init(void) {
#if SUBBUS_BOARD_ID == 1
  gpio_set_pin_level(J34_CNTL, false);
  gpio_set_pin_direction(J34_CNTL, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(J34_CNTL, GPIO_PIN_FUNCTION_OFF);

  gpio_set_pin_level(PPWR_CNTL, false);
  gpio_set_pin_direction(PPWR_CNTL, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(PPWR_CNTL, GPIO_PIN_FUNCTION_OFF);

  gpio_set_pin_direction(MM_ST1, GPIO_DIRECTION_IN);
  gpio_set_pin_function(MM_ST1, GPIO_PULL_OFF);  // ???

  gpio_set_pin_direction(MM_ST2, GPIO_DIRECTION_IN);
  gpio_set_pin_function(MM_ST2, GPIO_PULL_OFF); // ???

  gpio_set_pin_level(MM_CMD1, true);
  gpio_set_pin_direction(MM_CMD1, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(MM_CMD1, GPIO_PIN_FUNCTION_OFF);

  gpio_set_pin_level(MM_CMD2, true);
  gpio_set_pin_direction(MM_CMD2, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(MM_CMD2, GPIO_PIN_FUNCTION_OFF);
#endif
}

static void update_status(uint16_t *status, uint8_t pin, uint16_t bit) {
  if (gpio_get_pin_level(pin)) {
    *status |= bit;
  } else {
    *status &= ~bit;
  }
}

/**
 * This file should include a memory map. The current one is In Evernote.
 * 0x10-0x13 R: ADC Flow values
 * 0x14-0x17 RW: DAC Flow Setpoints
 * 0x18 R: CmdStatus W: Command
 * 0x19 R: ADC_U2_T
 * 0x1A R: ADC_U3_T
 */
static subbus_cache_word_t cmd_cache[CMD_HIGH_ADDR-CMD_BASE_ADDR+1] = {
  { 0, 0, true,  false, true, false, false } // Offset 0: R: ADC Flow 0
};

#if SUBBUS_BOARD_ID == 1
  #define N_CMD_PINS 4
  static uint8_t cmd_pins[N_CMD_PINS] = { SPR7, SPR7, J34_CNTL, PPWR_CNTL }; // SPR7 twice ???
#endif
#if SUBBUS_BOARD_ID == 5
  #define N_CMD_PINS 4
  static uint8_t cmd_pins[N_CMD_PINS] = { SPR7, SPR8, J34_EN, J35_EN };
#endif


#ifdef TIMED_COMMANDS

typedef struct {
  int when;
  uint16_t cmd;
} timed_cmd_t;

static timed_cmd_t timed_cmds[] = TIMED_COMMANDS;
#define N_TIMED_CMDS (sizeof(timed_cmds)/sizeof(timed_cmd_t))
static int timed_cmds_executed = 0;

#endif

static void cmd_poll(void) {
  uint16_t cmd;
  uint16_t status;

#ifdef N_TIMED_CMDS
  bool have_cmd = false;
  if (timed_cmds_executed < N_TIMED_CMDS && rtc_current_count >= timed_cmds[timed_cmds_executed].when) {
    cmd = timed_cmds[timed_cmds_executed++].cmd;
    have_cmd = true;
  } else if (subbus_cache_iswritten(&sb_cmd, CMD_BASE_ADDR, &cmd)) {
    have_cmd = true;
  }
  if (have_cmd) {
#else
  if (subbus_cache_iswritten(&sb_cmd, CMD_BASE_ADDR, &cmd)) {
#endif
#if SUBBUS_BOARD_ID == 1
      if (cmd/2 < N_CMD_PINS) {
        uint8_t pin = cmd_pins[cmd/2];
        gpio_set_pin_level(pin, cmd & 1);
      } else {
        switch (cmd) {
          case 8: gpio_set_pin_level(MM_CMD1, true); break; // Mini Moudi Valve Close
          case 9: gpio_set_pin_level(MM_CMD1, false); break; // Mini Moudi Valve Open
          default: break;
        }
      }
#endif
#if SUBBUS_BOARD_ID == 2
      switch (cmd) {
        default:
          break;
      }
#endif
#if SUBBUS_BOARD_ID == 5
      if (cmd/2 < N_CMD_PINS) {
        uint8_t pin = cmd_pins[cmd/2];
        gpio_set_pin_level(pin, cmd & 1);
    } else
      switch (cmd) {
        default:
          break;
      }
#endif
#ifdef HAVE_VIBE_SENSOR
    switch (cmd) {
      case 40: i2c_icm_set_mode(ICM_MODE_NO); break;
      case 41: i2c_icm_set_mode(ICM_MODE_SLOW); break;
      case 42: i2c_icm_set_mode(ICM_MODE_FAST); break;
      case 50: i2c_icm_set_fs(ICM_FS_2G); break;
      case 51: i2c_icm_set_fs(ICM_FS_4G); break;
      case 52: i2c_icm_set_fs(ICM_FS_8G); break;
      case 53: i2c_icm_set_fs(ICM_FS_16G); break;
      default: break;
    }
#endif
  }

  status = 0;
  update_status(&status, SPR7, 0x01);
  update_status(&status, SPR8, 0x02);
#if SUBBUS_BOARD_ID == 1
  update_status(&status, J34_CNTL, 0x04);
  update_status(&status, PPWR_CNTL, 0x08);
  update_status(&status, MM_CMD1, 0x10);
  update_status(&status, MM_CMD2, 0x20);
  update_status(&status, MM_ST1, 0x40);
  update_status(&status, MM_ST2, 0x80);
#endif
#if SUBBUS_BOARD_ID == 5
  update_status(&status, J34_EN, 0x04);
  update_status(&status, J35_EN, 0x08);
#endif
  sb_cache_update(cmd_cache, 0, status); // Make status bits true in high
}

static void cmd_reset(void) {
  commands_init();
  if (!sb_cmd.initialized) {
    sb_cmd.initialized = true;
  }
}

subbus_driver_t sb_cmd = {
  CMD_BASE_ADDR, CMD_HIGH_ADDR, // address range
  cmd_cache,
  cmd_reset,
  cmd_poll,
  false
};
