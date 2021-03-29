#ifndef I2C_ICM_H_INCLUDED
#define I2C_ICM_H_INCLUDED
#include "subbus.h"

#define I2C_ICM_BASE_ADDR 0x60
#define I2C_ICM_STATUS_OFFSET 0x00
#define I2C_ICM_STATUS_NREGS 1

#define I2C_ICM_ACC_OFFSET (I2C_ICM_STATUS_OFFSET+I2C_ICM_STATUS_NREGS)
#define I2C_ICM_ACC_NREGS 3 // x, y, z

#define I2C_ICM_NREGS (I2C_ICM_ACC_OFFSET+I2C_ICM_ACC_NREGS)
#define I2C_ICM_HIGH_ADDR (I2C_ICM_BASE_ADDR+I2C_ICM_NREGS-1)

#define I2C_ICM_ENABLE_DEFAULT true

/** ICM20948 sensor Addresses/values */
#define ICM20948_ADDR       	(0x69)     // 7-bit I2C address for ICM20948
#define ICM20948_ID         	(0xEA)     // ICM20948 Device ID
#define ICM20948_MAG_ADDR   	(0x0C)     // 7-bit I2C address for ICM20948 Mag Sensor
#define ICM20948_MAG_ID     	(0x09)     // 0000 1001

// SELECT USER BANK 0 REGISTER MAP ADDRESSES
#define WHO_AM_I 0x00
#define PWR_MGMT_1 0x06
#define ACCEL_XOUT_H 0x2D	// 0010 1101
#define REG_BANK_SEL 0x7F
 
// Added _icm
extern subbus_driver_t sb_i2c_icm;
void i2c_icm_enable(bool value);

#endif
