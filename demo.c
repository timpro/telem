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
    short ax, ay, az;
    short int_temp, battery;
    short pitch, roll;
    short compass;
    unsigned short force, pressure;

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

    // Welcome banner
    iprintf("\r\nBuilt: %s %s\r\n", __DATE__, __TIME__);
    iprintf("Ident, Count, time, lat, lon, alt, sats, error, G, yaw, press, temp, batt *Chksum\r\n");

    delay( 250 );
    RGB_LED( 0, 0, 0 );

    for(;;) {	
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
	battery = 0;

	gps_output(force, compass, pressure, int_temp, battery);

    }
}
