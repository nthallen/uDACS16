#include "Moudi.h"

/**
 ****************************************************
 */
enum moudi_state_t {
  moudi_idle, moudi_close, moudi_close_1,
  moudi_open, moudi_open_1, moudi_open_2,
  moudi_shutdown, moudi_shutdown_1
};

typedef struct {
  bool enabled;
  enum moudi_state_t state;
  uint16_t pending_cmd;
  uint32_t endtime;
  uint32_t delay;
//  uint16_t current;
} moudi_poll_def;

bool moudi_bypass_status;

static moudi_poll_def moudi = {
    true, moudi_idle,
//	, 0, 0, 0
};


#ifdef J6_HAS_DRV8871
#define set_bypass_open(val) gpio_set_pin_level(MM_BYPASS_IN1, val); // to true to initiate bypass open pulse
#define set_bypass_closeA(val) gpio_set_pin_level(MM_BYPASS_IN2A, val); // to true to initiate bypass close pulse
#define set_bypass_closeB(val) gpio_set_pin_level(MM_BYPASS_IN2B, val); // to true to initiate bypass close pulse
#define set_bypass_closeC(val) gpio_set_pin_level(MM_BYPASS_IN2C, val); // to true to initiate bypass close pulse
#define set_bypass_close(val) set_bypass_closeA(val); set_bypass_closeB(val); set_bypass_closeC(val);
#else
#define set_bypass_open(val)
#define set_bypass_close(val)
#endif


/**
 * moudi_poll(cmd) is called with cmd values of 8, 9 or 10 when the command
 * is written to the command register. It is also called with cmd of 0
 * every time cmd_poll() is called in order to handle command sequences.
 */
void moudi_poll(uint16_t cmd) {
  if (!moudi.enabled) return;
  if (cmd) {
    moudi.pending_cmd = cmd;
  }

  while (true) {
    switch (moudi.state) {
      case moudi_idle:
        cmd = moudi.pending_cmd;
        moudi.pending_cmd = 0;
        switch (cmd) {
          case 0: return;
          case 8: moudi.state = moudi_close; break;
          case 9: moudi.state = moudi_open; break;
          case 10: moudi.state = moudi_shutdown; break;
          default: return; // Invalid command
        }
        break;
      case moudi_close:
        gpio_set_pin_level(MM_CMD1, true); // to close Hanbay
        set_bypass_open(true);
        moudi.endtime = rtc_current_count + 50 * RTC_COUNTS_PER_MSEC;
        moudi.state = moudi_close_1;
        return;
      case moudi_close_1:
        if ( rtc_current_count <= moudi.endtime ) return;
        set_bypass_open(false);
        moudi_bypass_status = true;
        moudi.state = moudi_idle;
        return;
      case moudi_open:
        gpio_set_pin_level(MM_CMD1, false); // to open Hanbay
        moudi.endtime = rtc_current_count + 2000 * RTC_COUNTS_PER_MSEC;
        moudi.state = moudi_open_1;
        return;
      case moudi_open_1:
        if ( rtc_current_count <= moudi.endtime ) return;
        set_bypass_close(true);
        moudi.endtime = rtc_current_count + 50 * RTC_COUNTS_PER_MSEC;
        moudi.state = moudi_open_2;
        return;
      case moudi_open_2:
      case moudi_shutdown_1:
        if ( rtc_current_count <= moudi.endtime ) return;
        set_bypass_close(false);
        moudi_bypass_status = false;
        moudi.state = moudi_idle;
        return;
      case moudi_shutdown:
        gpio_set_pin_level(MM_CMD1, true); // to close Hanbay
        set_bypass_close(true);
        moudi.endtime = rtc_current_count + 50 * RTC_COUNTS_PER_MSEC;
        moudi.state = moudi_shutdown_1;
        return;
      default: // Invalid state!
        moudi.state = moudi_idle;
        return;
    }
  }
}

/*
          case 8: gpio_set_pin_level(MM_CMD1, true); break; // Mini Moudi Valve Close
          case 9: gpio_set_pin_level(MM_CMD1, false); break; // Mini Moudi Valve Open
*/
