
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


// Reads data into "char buf[64];" from habduino.h
short ublox_read()
{
    short  len = 0;

    // may want to wait until data appears
    len |= hal_i2c_read(I2C0_B, UBLOX_ADD, UBLOX_LEN);
    len <<= 8;
    len |= hal_i2c_read(I2C0_B, UBLOX_ADD, UBLOX_LEN + 1);

    if (len <= 0) return 0; // buffer empty
    if ( len > 60 ) len = 60;

    hal_i2c_BulkRead(I2C0_B, UBLOX_ADD, UBLOX_DATA, len, buf);
    return len;
}

void ublox_clear()
{
    do {
	delay (1);
    } while ( ublox_read() );
}

short ublox_test()
{
    short count = 0;
    short new;
    do {
	new = ublox_read();
	count += new;
    } while (new > 0) ;
    return count;
}

void ublox_init(void)
{
    // Setup I2C0 on pins B0,B1
    PORTB_PCR0 = PORT_PCR_MUX(2);
    PORTB_PCR0 = PORT_PCR_MUX(2);
}

void sendUBX(uint8_t *data, uint8_t len )
{
    short i;
    i2c_start(I2C0_B);

    i2c_write_byte(I2C0_B, UBLOX_ADD|I2C_WRITE);
    i2c_wait(I2C0_B);
    i2c_get_ack(I2C0_B);

    for (i=0; i<len; i++) {
	i2c_write_byte(I2C0_B, data[i]);
	i2c_wait(I2C0_B);
	i2c_get_ack(I2C0_B);
    }

    i2c_stop(I2C0_B);
    // I2C likes a rest now
    for(i=0; i<100; i++)
        asm("nop");
}

