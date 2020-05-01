#include "commands.h"
#include "subbus.h"
#include "spi.h"

static void commands_init(void) {
  #ifdef uDACS_B
    gpio_set_pin_level(PPMP_CNTL, false);
    gpio_set_pin_direction(PPMP_CNTL, GPIO_DIRECTION_OUT);
    gpio_set_pin_function(PPMP_CNTL, GPIO_PIN_FUNCTION_OFF);

    gpio_set_pin_direction(PPMPS, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(PPMPS, GPIO_PULL_UP);
    gpio_set_pin_function(PPMPS, GPIO_PIN_FUNCTION_OFF);

    gpio_set_pin_level(BPMP_CNTL, false);
    gpio_set_pin_direction(BPMP_CNTL, GPIO_DIRECTION_OUT);
    gpio_set_pin_function(BPMP_CNTL, GPIO_PIN_FUNCTION_OFF);

    gpio_set_pin_direction(BPMPS, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(BPMPS, GPIO_PULL_UP);
    gpio_set_pin_function(BPMPS, GPIO_PIN_FUNCTION_OFF);
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

static void cmd_poll(void) {
  uint16_t cmd;
  uint16_t status;
  if (subbus_cache_iswritten(&sb_cmd, CMD_BASE_ADDR, &cmd)) {
    switch (cmd) {
      case 0: // both pumps off
      case 1: // both pumps on
        gpio_set_pin_level(PPMP_CNTL, cmd);
        gpio_set_pin_level(BPMP_CNTL, cmd);
        break;
      case 2: // POPS Pump
      case 3:
        gpio_set_pin_level(PPMP_CNTL, cmd & 1);
        break;
      case 4:
      case 5:
        gpio_set_pin_level(BPMP_CNTL, cmd & 1);
        break;
      default:
        break;
    }
  }
  status = 0;
  update_status(&status, PPMP_CNTL, 0x01);
  update_status(&status, PPMPS, 0x02);
  update_status(&status, BPMP_CNTL, 0x04);
  update_status(&status, BPMPS, 0x08);
  sb_cache_update(cmd_cache, 0, status ^ 0xA); // Make status bits true in high
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
