
// accel8491.c -- Accelerometer support
//
//  Copyright (c) 2013 John Greb <github.com/hexameron>

// Based on example code from Freescale, released by Element 14.

#include "common.h"
#include "hal_i2c.h"

#define MMA8491_ADD          (0x55<<1)
#define MMA8491_STATUS       (0)
#define MMA8491Q_Enable()    (GPIOA_PSOR |= 1<<13)
#define MMA8491Q_DisEnable() (GPIOA_PCOR |= 1<<13)
#define MMA8491_3AXIS        (1<<3)

static short acc8491data[3] = {4096,0,0};

void accel_read()
{
    uint8_t status;
    char    rawdata[8];
    short   loops = 5;

    // 8491 needs 0.9ms to power on for every read.
    MMA8491Q_Enable();
    do {
	delay (1); // FIXME: 1ms  busy wait, using low power timer.
	status = hal_i2c_read(I2C1_B, MMA8491_ADD, MMA8491_STATUS);
    } while ( --loops && !status );

    if ( (status & MMA8491_3AXIS) && (status <= 0x0f)){
	status = hal_i2c_BulkRead(I2C1_B, MMA8491_ADD, 1, 6, rawdata);
	acc8491data[0] = ((rawdata[0]<<8) + rawdata[1]) ;
	acc8491data[1] = ((rawdata[2]<<8) + rawdata[3]) ;
	acc8491data[2] = ((rawdata[4]<<8) + rawdata[5]) ;
    }

    // Power off after every read
    MMA8491Q_DisEnable();
}

void accel_init(void)
{
    // Setup 8491 Enable on pin A13
    PORTA_PCR13 = PORT_PCR_MUX(1);
    GPIOA_PCOR |= 1 << 13;
    GPIOA_PDDR |= 1 << 13;
}

// Read acceleration values for all axis in bulk when asked for X
// 8491 only supports 8g mode, so return 16 bit values to match 8451 

int16_t accel_x(void) { return acc8491data[0]&~3; }
int16_t accel_y(void) { return acc8491data[1]&~3; }
int16_t accel_z(void) { return acc8491data[2]&~3; }

