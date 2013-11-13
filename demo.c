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
    short ax, ay, az;
    short tx, ty, tz;
    short t1, t2, temp;
    short red, green, blue;
    short pitch, roll;
    short compass;
    unsigned short force, pressure;
    unsigned short checksum = 0xdead;
    unsigned short seq = 100;

    // Initialize all modules
    uart_init(115200);
    hal_i2c_init(I2C0_BASE_PTR);	// Setup I2C for ublox
//    hal_i2c_init(I2C1_BASE_PTR);	// Setup I2C for EVK
//    accel_init();
//    baro_init();
//    mag_init();
    setvbuf(stdin, NULL, _IONBF, 0);        // No buffering

    // Unused here, but necessary.
    heap_end = _sbrk(0);

    RGB_LED( 0, 40, 0 );
    delay( 100 );
    // Welcome banner
    iprintf("\r\n\r\n====== Freescale Freedom FRDM-KL25Z\r\n");
    iprintf("\r\nBuilt: %s %s\r\n", __DATE__, __TIME__);
    iprintf("Ident, Count, force,pitch,roll, mag field, pressure,temp *Chksum\r\n");

    ax = ay = az = 0;
    for(;;) {
	pat = 1 << 7;
	while (pat) {
		accel_read();
		// Oversample acceleration to steady compass
		tx = accel_x();
		ax = (ax + tx) >> 1;
		ty = accel_y();
		ay = (ay + ty) >> 1;
		tz = accel_z();
		az = (az + tz) >> 1;
		force = magnitude( ax, ay, az );
		pitch = findArctan( ax, ay, az );
		roll  = findArctan( az, ay, 0 );
		ax = tx;
		ay = ty;
		az = tz;
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
		delay(120);
	}

//	pressure = get_pressure();
//	temp = baro_temp();
//	compass = mag_compass(pitch, roll);
//	iprintf("$$HEX,%d,%3d,%3d,%3d,",seq++, force, pitch, roll);
//	iprintf("%3d,%d0,%d,*%x\r\n", compass, pressure, temp, checksum);
    }
}
