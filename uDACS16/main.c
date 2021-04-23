//#include <atmel_start.h>
//#include "utils.h"
#include <hal_init.h>
#include "usart.h"
#include "subbus.h"
#include "control.h"
#include "spi_psd.h"
#include "i2c.h"
#include "i2c_icm20948.h"
#include "rtc_timer.h"
#include "driver_temp.h"	// in place for driver_init
#include "commands.h"

#ifdef CAN_BOARD_ID
#include "can_control.h"
#endif

int main(void)
{
  /* Initializes MCU, drivers and middleware */
  //atmel_start_init();
  system_init();

  if (subbus_add_driver(&sb_base)
    || subbus_add_driver(&sb_fail_sw)
    || subbus_add_driver(&sb_board_desc)
    || subbus_add_driver(&sb_control)
    || subbus_add_driver(&sb_spi)
    || subbus_add_driver(&sb_i2c)
    || subbus_add_driver(&sb_cmd)
    || subbus_add_driver(&sb_rtc)
#ifdef HAVE_VIBE_SENSOR
    || subbus_add_driver(&sb_i2c_icm)
#endif
#ifdef CAN_BOARD_ID
    || subbus_add_driver(&sb_can)
#endif
    ) {
    while (true) ; // some driver is mis-configured.
  }
  subbus_reset();
  while (1) {
    subbus_poll();
  }
}
