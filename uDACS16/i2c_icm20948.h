#ifndef I2C_ICM_H_INCLUDED
#define I2C_ICM_H_INCLUDED
#include "subbus.h"

#define I2C_ICM_BASE_ADDR 0x60
#define I2C_ICM_STATUS_OFFSET 0x00
#define I2C_ICM_STATUS_NREGS 1

#define I2C_ICM_ACC_OFFSET (I2C_ICM_STATUS_OFFSET+I2C_ICM_STATUS_NREGS)
#define I2C_ICM_ACC_NREGS 3 // x, y, z

#define I2C_ICM_FIFO_OFFSET (I2C_ICM_ACC_OFFSET+I2C_ICM_ACC_NREGS)
#define I2C_ICM_FIFO_NREGS 4 // mode, count, fifo, diag
#define I2C_ICM_FIFO_SAMPLES 1000
#define I2C_ICM_FIFO_WORDS_PER_SAMPLE 3 // x, y, z
#define I2C_ICM_FIFO_SIZE (I2C_ICM_FIFO_SAMPLES*I2C_ICM_FIFO_WORDS_PER_SAMPLE)
#define I2C_ICM_FIFO_COUNT_OFFSET I2C_ICM_FIFO_OFFSET+1
#define I2C_ICM_FIFO_REG_OFFSET I2C_ICM_FIFO_OFFSET+2

// Values to be written to the ICM_MODE register at I2C_ICM_FIFO_OFFSET
#define ICM_MODE_NO   (0)
#define ICM_MODE_SLOW (1)
#define ICM_MODE_FAST (2)
#define ICM_MODE_MASK ((uint16_t)(7))
#define ICM_FS_2G  0
#define ICM_FS_4G  1
#define ICM_FS_8G  2
#define ICM_FS_16G 3
#define ICM_MODE1_ACCEL_CFG 0x01
#define ICM_MODE2_ACCEL_CFG 0x39
#define ICM_MODE_FS_MASK 0x18
#define ICM_MAX_FIFO_MASK ((uint16_t)0x1FFF)

#define I2C_ICM_NREGS (I2C_ICM_FIFO_OFFSET+I2C_ICM_FIFO_NREGS)
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
#define PWR_MGMT_2 0x07
#define INT_ENABLE_2 0x12
#define INT_STATUS_2 0x1B
#define ACCEL_XOUT_H 0x2D	// 0010 1101
#define FIFO_EN_1 0x66
#define FIFO_EN_2 0x67
#define FIFO_RST  0x68
#define FIFO_R_W  0x72
#define REG_BANK_SEL 0x7F
// USER BANK 2 REGISTER MAP ADDRESSES
#define ACCEL_CONFIG 0x14
#define ACCEL_DLPFCFG_FCHOICE ((7<<3)|1)
#define ACCEL_CFG(x) (((x)<<1)|ACCEL_DLPCFG_FCHOICE)

// Added _icm
extern subbus_driver_t sb_i2c_icm;
void i2c_icm_enable(bool value);
void i2c_icm_set_mode(uint16_t mode);
void i2c_icm_set_fs(uint16_t fs);

#define icm_swap16(x) x = ((((x)&0xFF00)>>8)|(((x)&0xFF)<<8))

#endif
