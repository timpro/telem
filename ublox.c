
// ublox.c -- Max 7C GPS support
//
// Copyright (c) 2013 John Greb <github.com/hexameron>
//
// Code in habduino.h subject to GPL licence

#include "common.h"
#include "hal_i2c.h"
#include "habduino.h"

#define UBLOX_ADD  (0x42<<1)
#define UBLOX_DATA (0xFF)
#define UBLOX_LEN  (0xFD)


short ublox_read()
{
    short  len = 0;
    char   rawdata[64];

    // may want to wait until data appears
    len |= hal_i2c_read(I2C0_B, UBLOX_ADD, UBLOX_LEN);
    len <<= 8;
    len |= hal_i2c_read(I2C0_B, UBLOX_ADD, UBLOX_LEN + 1);

    if (!len) return 0; // buffer empty

    if ( len > 60 ) len = 60;
    hal_i2c_BulkRead(I2C0_B, UBLOX_ADD, UBLOX_DATA, len, rawdata);
    return len;
}

void ublox_clear()
{
    do {
	delay (1);
    } while ( ublox_read() );
}

void ublox_init(void)
{
    // Setup I2C0 on pins B0,B1
    PORTB_PCR0 = PORT_PCR_MUX(2);
    PORTB_PCR0 = PORT_PCR_MUX(2);
}


