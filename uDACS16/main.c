//#include <atmel_start.h>
//#include "utils.h"
#include <hal_init.h>
#include "usart.h"
#include "i2c.h"
#include "subbus.h"
#include "control.h"
#include "driver_temp.h"	// in place for driver_init
#include "commands.h"

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	//atmel_start_init();
	system_init();
	
  if (subbus_add_driver(&sb_base) 
	  || subbus_add_driver(&sb_fail_sw) 
	  || subbus_add_driver(&sb_board_desc)
	  || subbus_add_driver(&sb_control)
	  || subbus_add_driver(&sb_cmd)
      || subbus_add_driver(&sb_i2c)
      // || subbus_add_driver(&sb_can)
	  // || subbus_add_driver(&sb_spi)
	  // || subbus_add_driver(&sb_rtc)
	  ) {
	  while (true) ; // some driver is misconfigured.
  }
  subbus_reset();
  while (1) {
	  subbus_poll();
  }
}
