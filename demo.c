//
// demo.c -- Simple demonstration program
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
//  Copyright (c) 2013-2014 John Greb <github.com/hexameron>
//

#include <stdio.h>
#include "freedom.h"
#include "common.h"

extern char *_sbrk(int len);
static char *heap_end;

// Main program
int main(void)
{
    unsigned int pat;
    long time;
    short ax, ay, az;
    short int_temp;
    short pitch, roll;
    short compass;
    short lat, lon, alt, sats;
    unsigned short force, pressure;
    unsigned short checksum = 0xdead;
    unsigned short seq = 100;

    // Initialize all modules
    uart_init(9600);
    hal_i2c_init(I2C1_BASE_PTR);	// Setup I2C for EVK
    accel_init();
    baro_init();
    mag_init();
    ublox_init();
    setvbuf(stdin, NULL, _IONBF, 0);        // No buffering

    // Unused here, but necessary.
    heap_end = _sbrk(0);

    RGB_LED( 100, 0, 0 );
    delay( 100 );
    RGB_LED( 0, 0, 0 );

    // Welcome banner
    iprintf("\r\n\r\n====== Freescale Freedom FRDM-KL25Z\r\n");
    iprintf("\r\nBuilt: %s %s\r\n", __DATE__, __TIME__);
    iprintf("Ident, Count, time, lat, lon, alt, sats, error, G, mag field, pressure, temp *Chksum\r\n");

    for(;;) {
	pat = 1 << 15;
	while (pat) {
		pat >>= 1;
		delay(128);
	}
	accel_read();
	ax = accel_x();
	ay = accel_y();
	az = accel_z();
	force = magnitude( ax, ay, az );

	// We do not need pitch/roll, only the sin/cos
	// could pass ax,ay,az directly to compass code
	// saving 2 lookup tables and getting better accuracy
	pitch = findArctan( ax, ay, az );
	roll  = findArctan( az, ay, 0 );
	force += (force >> 1) + (force >> 4);
	force >>= 6; // force as percentage of 1G

	compass = mag_compass(pitch, roll);
	pressure = get_pressure();
	int_temp = baro_temp();
	gps_update();
	time = gps_time();
	lon = gps_lon();
	lat = gps_lat();
	alt = gps_alt();
	sats = gps_sats();
	iprintf("%02d%02d%02d,",(char)(time>>16)&31, (char)(time>>8)&63, (char)time&63 );
	iprintf("%04d,%04d,%d,%d,%d\r\n", lat, lon, alt, sats, gps_error() );
	if (++seq & 7) continue;
	iprintf("$$HEX,%d,%02d%02d%02d,",seq>>3,(char)(time>>16)&31, (char)(time>>8)&63, (char)time&63 );
	iprintf("%04d,%04d,%d,%d,%d,", lat, lon, alt, sats, gps_error() );
	iprintf("%d,%d,%d,%d*%x\r\n", force, compass, pressure, int_temp, checksum);

    }
}
