#ifndef MOUDI_H_INCLUDED
#define MOUDI_H_INCLUDED

#include <stdint.h>
#include <stdbool.h> 
#include "serial_num.h"
#include "uDACS_pins.h"

/**
 * @param cmd 0, 8, 9 or 10
 */
extern void moudi_poll(uint16_t cmd);
extern bool moudi_bypass_status;

#endif
