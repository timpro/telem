
// ublox.c -- Max 7C GPS support
//
// Copyright (c) 2013 John Greb <github.com/hexameron>
//
// Code in habduino.h subject to GPL licence

#include "common.h"
#include "hal_i2c.h"
#include "habduino.h"

#define UBLOX_ADD  (0x42<<1)
#define UBLOX_LENH (0xFD)
#define UBLOX_LENL (0xFE)
#define UBLOX_DATA (0xFF)

// Reads data into "char buf[64];" from habduino.h

short ublox_check_i2c()
{
    short  len = 0;

    // may want to wait until data appears
    len |= hal_i2c_read(I2C0_B, UBLOX_ADD, UBLOX_LENH);
    len <<= 8;
    len |= hal_i2c_read(I2C0_B, UBLOX_ADD, UBLOX_LENL);
    return len;
}

void gps_get_data()
{
    hal_i2c_BulkRead(I2C0_B, UBLOX_ADD, UBLOX_DATA, 60, buf);
}

void ublox_clear_i2c()
{
    do {
	gps_get_data();
	delay (1);
    } while ( ublox_check_i2c() );
}

short testcount = 0;
short ublox_test()
{
    short len;
//    gps_get_time();
    setupGPS();
    delay(100);
    len = ublox_check_i2c();
    return len;
}

void ublox_init(void)
{
    // Use MUX(1) to disable ADC on pins used for tx/rx
    // Use MUX(4) for UART0, but that is usb host port
    PORTE_PCR20 = PORT_PCR_MUX(1); //tx
    PORTE_PCR21 = PORT_PCR_MUX(4); //rx

    // Setup I2C0 on pins B0,B1
    // PORTB_PCR2 = PORT_PCR_MUX(2);
    // PORTB_PCR3 = PORT_PCR_MUX(2);
}

void sendUBX(uint8_t *data, uint8_t len )
{
    short i;
    i2c_start(I2C0_B);

    i2c_write_byte(I2C0_B, UBLOX_ADD|I2C_WRITE);
    i2c_wait(I2C0_B);
    i2c_get_ack(I2C0_B);

    // Manual says just send multiple data 
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

