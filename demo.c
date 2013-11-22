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
    short red, green, blue;
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
    // Welcome banner
    iprintf("\r\n\r\n====== Freescale Freedom FRDM-KL25Z\r\n");
    iprintf("\r\nBuilt: %s %s\r\n", __DATE__, __TIME__);
    iprintf("Ident, Count, time, lat, lon, alt, sats, error, G, mag field, pressure, temp *Chksum\r\n");

    for(;;) {
	pat = 1 << 23;
	while (pat) {
		accel_read();
		ax = accel_x();
		ay = accel_y();
		az = accel_z();
		force = magnitude( ax, ay, az );
		pitch = findArctan( ax, ay, az );
		roll  = findArctan( az, ay, 0 );
		force += (force >> 1) + (force >> 4);
		force >>= 6; // force as percentage of 1G

		red = (force - 100);
		if (red < 0) red = -red;
		blue = pitch;
		if (blue < 0) blue = -blue;
		green = roll >> 1;
		if (green < 0) green = -green;

		if (green > 80) green = 80;
		if (blue > 80) blue = 80;
		if (red > 80) red = 80;

		RGB_LED( red, green, blue );

		pat >>= 1;
		delay(128);

	}

	compass = mag_compass(pitch, roll);
	pressure = get_pressure();
	int_temp = baro_temp();
	time = gps_time();
	lon = gps_lon();
	lat = gps_lat();
	alt = gps_alt();
	sats = gps_sats();
	gps_update();
	iprintf("$$HEX,%d,%02d%02d%02d,",seq++,(char)(time>>16)&31, (char)(time>>8)&63, (char)time&63 );
	iprintf("%04d,%04d,%d,%d,%d,", lat, lon, alt, sats, gps_error() );
	iprintf("%d,%d,%d,%d*%x\r\n", force, compass, pressure, int_temp, checksum);

    }
}
