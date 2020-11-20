/** @file serial_num.h
 * This file must define:
 *  CAN_BOARD_ID: The CAN Identifier for the board. This must be unique on a specific CAN Bus
 *  SUBBUS_BOARD_SN: The serial number of this board among boards of the same SUBBUS_BOARD_TYPE
 *  SUBBUS_BUILD_NUM:
 *  SUBBUS_SUBFUNCTION: The board type code as defined in "SYSCON Memory Maps and Subbus Board IDs"
 *  SUBBUS_BOARD_ID: Board Identification number (*not* the CAN_BOARD_ID). Defines the type of board
 *     SUBBUS_SUBFUNCTION may be the same as SUBBUS_BOARD_ID if there is no significant
 *     configuration difference between boards. If different, the SUBBUS_BOARD_ID values
 *     should be documented along with the SUBBUS_BOARD_SN etc. in the board's Specifications document
 *  SUBBUS_BOARD_INSTRUMENT_ID: Number that maps to Instrument name. (not yet used as of 4/18/19)
 *  SUBBUS_BOARD_REV: String encapsulating almost anything here
 */
#ifndef SERIAL_NUM_H_INCLUDED
#define SERIAL_NUM_H_INCLUDED
#include "uDACS_pins.h"

// These parameters are common to all boards built with this code
#define SUBBUS_BOARD_FIRMWARE_REV "V1.0"
#define SUBBUS_BOARD_BUILD_NUM 1
// #define SUBBUS_BOARD_SN 1
//#define HAVE_RTC

/**
 * Build definitions
 * 1: Initial build
 */
#if ! defined(SUBBUS_BOARD_SN)
#error Must define SUBBUS_BOARD_SN in Build Properties
#endif

#define SUBBUS_SUBFUNCTION 15
#define SUBBUS_SUBFUNCTION_HEX F
#define SUBBUS_BOARD_BOARD_REV "Rev A"
#define SUBBUS_BOARD_INSTRUMENT_ID 7
#define SUBBUS_BOARD_INSTRUMENT "DPOPS"
#define SUBBUS_BOARD_BOARD_TYPE "uDACS16"

#if SUBBUS_BOARD_SN == 1
  #define SUBBUS_BOARD_ID 1 // uDACS "A"
#elif SUBBUS_BOARD_SN == 2
  #define SUBBUS_BOARD_ID 2 // uDACS "B"
#elif SUBBUS_BOARD_SN == 3
  #define SUBBUS_BOARD_ID 1 // uDACS "A"
#endif

#if ! defined(SUBBUS_BOARD_ID)
#error Must define SUBBUS_BOARD_ID
#endif

/**
 * Configuration variables for uDACS16
 * HAVE_RTC: If defined, enables time-based fail light control
 * USING_RTC: If defined, indicates rtc_timer module is using an RTC rather
 *    than a TC hardware component
 * SB_FAIL_PIN: If defined, specifies pin for the !fail signal output
 * SB_FAIL_PIN2: If defined, specifies a second pin for the !fail signal output
 * SB_FAIL_TIMEOUT_SECS: If defined, overrides the 2-minute default duration for the fail timeout
 *   Used for testing. Should not be defined for deployment.
 */

#if SUBBUS_BOARD_ID == 1 // uDACS "A"
  #define HAVE_RTC
  #define SB_FAIL_PIN FAIL_OFF
  // #define SB_FAIL_TIMEOUT_SECS 20
  #define J34_CNTL J34_EN  // J34 may be used for DPOPS box FAIL LED power
  #define PPWR_CNTL J35_EN	// J35 is POPS Instrument Power
#endif

#ifndef SUBBUS_SUBFUNCTION_HEX
#define SUBBUS_SUBFUNCTION_HEX SUBBUS_SUBFUNCTION
#endif

#ifdef CAN_BOARD_ID

#define SUBBUS_BOARD_REV_STR(SN,ID) SUBBUS_BOARD_INSTRUMENT " " SUBBUS_BOARD_BOARD_TYPE " " \
SUBBUS_BOARD_BOARD_REV " " SUBBUS_BOARD_FIRMWARE_REV \
" S/N:" #SN " CAN ID:" #ID " " SUBBUS_BOARD_LOCATION
#define SUBBUS_BOARD_REV_XSTR(SUBBUS_BOARD_SN,CAN_BOARD_ID) SUBBUS_BOARD_REV_STR(SUBBUS_BOARD_SN,CAN_BOARD_ID)
#define SUBBUS_BOARD_REV SUBBUS_BOARD_REV_XSTR(SUBBUS_BOARD_SN,CAN_BOARD_ID)

#else

#define SUBBUS_BOARD_REV_STR(SN,SF) "V" #SF ":0:" SUBBUS_BOARD_INSTRUMENT " " SUBBUS_BOARD_BOARD_TYPE " " \
SUBBUS_BOARD_BOARD_REV " " SUBBUS_BOARD_FIRMWARE_REV \
" S/N:" #SN
#define SUBBUS_BOARD_REV_XSTR(SN,SF) SUBBUS_BOARD_REV_STR(SN,SF)
#define SUBBUS_BOARD_REV SUBBUS_BOARD_REV_XSTR(SUBBUS_BOARD_SN,SUBBUS_SUBFUNCTION_HEX)

#endif

#endif
