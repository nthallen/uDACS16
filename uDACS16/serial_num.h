/** @file serial_num.h
 * This file must define:
 *  CAN_BOARD_ID: The CAN Identifier for the board. This must be unique on a specific CAN Bus
 *  SUBBUS_BOARD_SN: The serial number of this board among boards of the same SUBBUS_BOARD_TYPE
 *  SUBBUS_BUILD_NUM:
 *  SUBBUS_SUBFUNCTION: The board type code as defined in "SYSCON Memory Maps and Subbus Board IDs"
 *  SUBBUS_SUBFUNCTION_HEX: The hexadecimal encoding of SUBBUS_SUBFUNCTION, used in SUBBUS_BOARD_REV
 *  SUBBUS_BOARD_BOARD_REV: The physical board revision
 *  SUBBUS_BOARD_ID: Board Identification number (*not* the CAN_BOARD_ID). Defines the type of board
 *     0: Test
 *     1: DPOPS "A"
 *     2: DPOPS "B"
 *     3: ALPHA-SB Port (Left) Engine Assy
 *     4: ALPHA-SB Starboard (Right) Engine Assy
 *	   5: Halogens TRU Interface Box
 *     SUBBUS_SUBFUNCTION may be the same as SUBBUS_BOARD_ID if there is no significant
 *     configuration difference between boards. If different, the SUBBUS_BOARD_ID values
 *     should be documented along with the SUBBUS_BOARD_SN etc. in the board's Specifications document
 *       SUBBUS_SUBFUNCTION 15: default ?
 *  SUBBUS_BOARD_INSTRUMENT_ID: Number that maps to Instrument name. (not yet used as of 4/18/19)
 *     1: ALPHA-SB
 *     4: Test
 *     5: Halogens
 *     7: DPOPS
 *     8: SMoudi (SABRE Mini Moudi)
 *  SUBBUS_BOARD_REV: String encapsulating almost anything here
 *
 * This file MAY define:
 *  SB_FAIL_PIN used in subbus.c
 *  SB_FAIL_PIN2 used in subbus.c
 *  TIMED_COMMANDS used in commands.c
 *  J8_IS_MODE_SWITCH used in subbus.c
 *  MODE_PIN_0 used in subbus.c
 *  MODE_PIN_1 used in subbus.c
 *  HAVE_RTC
 *  J4_3_IS_FAIL_BAR
 *  J4_IS_VIBE_SENSOR
 *  HAVE_VIBE_SENSOR
 */
#ifndef SERIAL_NUM_H_INCLUDED
#define SERIAL_NUM_H_INCLUDED
#include "config/hpl_can_config.h"
#include "uDACS_pins.h"

// These parameters are common to all boards built with this code
// 2/7/24 Build #9 V1.7.0 Testing Vibration Sensor streaming
// 5/10/24 Build #10 V1.7.1 Vibration sensor mode 3
#define SUBBUS_BOARD_FIRMWARE_REV "V1.7.1"
#define SUBBUS_BOARD_BUILD_NUM 10
#define HAVE_RTC

/**
 * Build definitions
 * 1: Initial build
 */
#if ! defined(SUBBUS_BOARD_SN)
#error Must define SUBBUS_BOARD_SN in Build Properties
#endif

#define SUBBUS_SUBFUNCTION 15	// uDACS16
#define SUBBUS_SUBFUNCTION_HEX F
#define SUBBUS_BOARD_BOARD_REV "Rev A"

#if SUBBUS_BOARD_SN == 1
  #define SUBBUS_BOARD_ID 1 // Test: configured as DPOPS uDACS "A" (Lab test Only)
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS16"
  #define SUBBUS_BOARD_INSTRUMENT_ID 8
  #define SUBBUS_BOARD_INSTRUMENT "SMoudi"
  #define SUBBUS_BOARD_LOCATION "SMoudi Test"
#elif SUBBUS_BOARD_SN == 2
  #define SUBBUS_BOARD_ID 1 // uDACS "A"
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS16"
  #define SUBBUS_BOARD_INSTRUMENT_ID 7
  #define SUBBUS_BOARD_INSTRUMENT "DPOPS"
#elif SUBBUS_BOARD_SN == 3
  #define SUBBUS_BOARD_ID 1 // uDACS "A" (backup)
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS16"
  #define SUBBUS_BOARD_INSTRUMENT_ID 7
  #define SUBBUS_BOARD_INSTRUMENT "DPOPS"
#elif SUBBUS_BOARD_SN == 4
  #define SUBBUS_BOARD_ID 1 // SMoudi (SABRE Mini Moudi) Configured as DPOPS uDACS "A"
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS16"
  #define SUBBUS_BOARD_INSTRUMENT_ID 8
  #define SUBBUS_BOARD_INSTRUMENT "SMoudi"
  #define SUBBUS_BOARD_LOCATION "SMoudi Backup"
  // #define CAN_BOARD_ID 14 // for Test board
#elif SUBBUS_BOARD_SN == 5
  #define SUBBUS_BOARD_ID 1 // SMoudi (SABRE Mini Moudi) Configured as DPOPS uDACS "A"
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS16"
  #define SUBBUS_BOARD_INSTRUMENT_ID 8
  #define SUBBUS_BOARD_INSTRUMENT "SMoudi"
  #define SUBBUS_BOARD_LOCATION "SMoudi Flight"
  // #define CAN_BOARD_ID 1 // for Test board
#elif SUBBUS_BOARD_SN == 6
  #if CONF_SERCOM_5_USART_BAUD != 380400
    #error SERCOM5 Baud rate must be 380400 for vibration sensor data
  #endif
  #if CONF_SERCOM_3_I2CM_BAUD != 400000
    #error SERCOM3 I2C bit rate must be 400000 for vibration sensor data
  #endif
  #if CONF_CAN0_BTP_BRP != 16
    #error CAN0 Baud rate prescaler must be 16 for vibration sensor data
  #endif
  #define SUBBUS_BOARD_ID 3 // ALPHA-SB Port Engine Assy
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS16"
  #define SUBBUS_BOARD_INSTRUMENT_ID 1
  #define SUBBUS_BOARD_INSTRUMENT "ALPHA-SB"
  #define SUBBUS_BOARD_LOCATION "ALPHA-SB Port Engine Assy"
  #define J4_IS_VIBE_SENSOR
  #define HAVE_VIBE_SENSOR
  #define CAN_BOARD_ID 1
#elif SUBBUS_BOARD_SN == 7
  #if CONF_SERCOM_5_USART_BAUD != 380400
    #error SERCOM5 Baud rate must be 380400 for vibration sensor data
  #endif
  #if CONF_SERCOM_3_I2CM_BAUD != 400000
    #error SERCOM3 (CAN) Baud rate must be 400000 for vibration sensor data
  #endif
  /* BRP == 4 should yield bitrate of 200KHz. BRP==2 produced comm errors
   * Reverting to START value of 16 for 50 KHz bit rate
   */
  #if CONF_CAN0_BTP_BRP != 16
    #error CAN0 Baud rate prescaler must be 16 for vibration sensor data
  #endif
  #define SUBBUS_BOARD_ID 4 // ALPHA-SB Starboard Engine Assy
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS16"
  #define SUBBUS_BOARD_INSTRUMENT_ID 1
  #define SUBBUS_BOARD_INSTRUMENT "ALPHA-SB"
  #define SUBBUS_BOARD_LOCATION "ALPHA-SB Starboard Engine Assy"
  #define J4_IS_VIBE_SENSOR
  #define HAVE_VIBE_SENSOR
  #define CAN_BOARD_ID 2
#elif SUBBUS_BOARD_SN == 8
  #define SUBBUS_BOARD_ID 5 // Halogens TRU Interface box (spare)
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS16"
  #define SUBBUS_BOARD_INSTRUMENT_ID 5 // Halogens
  #define SUBBUS_BOARD_INSTRUMENT "Halogens"
  #define SUBBUS_BOARD_LOCATION "Halogens TRU Interface Box"
#elif SUBBUS_BOARD_SN == 9
  #define SUBBUS_BOARD_ID 5 // Halogens TRU Interface box
  #define SUBBUS_BOARD_BOARD_TYPE "uDACS16"
  #define SUBBUS_BOARD_INSTRUMENT_ID 5 // Halogens
  #define SUBBUS_BOARD_INSTRUMENT "Halogens"
  #define SUBBUS_BOARD_LOCATION "Halogens TRU Interface Box"
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
  #include "rtc_timer.h"
  #define SB_FAIL_PIN FAIL_OFF
  #define SB_FAIL_PIN2 UC_SCL
  #define J4_3_IS_FAIL_BAR
  #define MODE_PIN_0 SPR8
  // #define SB_FAIL_TIMEOUT_SECS 20
  #define J34_CNTL J34_EN  // J34 may be used for DPOPS box FAIL LED power
  #define PPWR_CNTL J35_EN	// J35 is POPS Instrument Power
  #define J8_IS_MODE_SWITCH
  // Command 1 is J7 On for Operate LED
  // Command 5 is J34 On for Raspberry Pi delay power on
  #define TIMED_COMMANDS {{0,1},{5*RTC_COUNTS_PER_SECOND,5}}
  // Mini Moudi Hanbay Valve Actuator
  #define MM_CMD1 PMOD2
  #define MM_CMD2 PMOD4
  #define MM_ST1 PMOD6
  #define MM_ST2 PMOD8
#endif

#if defined(J4_3_IS_FAIL_BAR) && defined(J4_IS_VIBE_SENSOR)
#error J4 cannot support both I2C and Fail output
#endif

#ifndef SUBBUS_SUBFUNCTION_HEX
#define SUBBUS_SUBFUNCTION_HEX SUBBUS_SUBFUNCTION
#endif

#ifdef CAN_BOARD_ID

#define SUBBUS_BOARD_DESC_STR(SN,ID) SUBBUS_BOARD_INSTRUMENT " " SUBBUS_BOARD_BOARD_TYPE " " \
  SUBBUS_BOARD_BOARD_REV " " SUBBUS_BOARD_FIRMWARE_REV \
  " S/N:" #SN " CAN ID:" #ID " " SUBBUS_BOARD_LOCATION
#define SUBBUS_BOARD_DESC_XSTR(SUBBUS_BOARD_SN,CAN_BOARD_ID) SUBBUS_BOARD_DESC_STR(SUBBUS_BOARD_SN,CAN_BOARD_ID)
#define SUBBUS_BOARD_DESC SUBBUS_BOARD_DESC_XSTR(SUBBUS_BOARD_SN,CAN_BOARD_ID)

#else

#define SUBBUS_BOARD_DESC_STR(SN) SUBBUS_BOARD_INSTRUMENT " " SUBBUS_BOARD_BOARD_TYPE " " \
  SUBBUS_BOARD_BOARD_REV " " SUBBUS_BOARD_FIRMWARE_REV \
  " S/N:" #SN " " SUBBUS_BOARD_LOCATION
#define SUBBUS_BOARD_DESC_XSTR(SUBBUS_BOARD_SN) SUBBUS_BOARD_DESC_STR(SUBBUS_BOARD_SN)
#define SUBBUS_BOARD_DESC SUBBUS_BOARD_DESC_XSTR(SUBBUS_BOARD_SN)

#endif // CAN_BOARD_ID


#define SUBBUS_BOARD_REV_STR(SN,SF) "V" #SF ":0:" SUBBUS_BOARD_DESC
#define SUBBUS_BOARD_REV_XSTR(SN,SF) SUBBUS_BOARD_REV_STR(SN,SF)
#define SUBBUS_BOARD_REV SUBBUS_BOARD_REV_XSTR(SUBBUS_BOARD_SN,SUBBUS_SUBFUNCTION_HEX)

#endif
