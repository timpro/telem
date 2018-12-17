//
// demo.c -- Simple demonstration program
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
//  Copyright (c) 2013-2015 John Greb <github.com/hexameron>
//

#include <stdio.h>
#include "common.h"

extern char *_sbrk(int len);
static char *heap_end;

#include "habtext.h"
// Main program
int main(void)
{
    short j;
    char msg[96];
    short ax, ay, az;
    short red, green, blue;
    short pitch, roll;
    unsigned short i, force;
    //unsigned short checksum = 0xdead;
    //unsigned short seq = 100;

    // Initialize all modules
    uart_init(115200);
    rfm98_init();
    accel_init();
    touch_init((1 << 9) | (1 << 10));       // Channels 9 and 10
    setvbuf(stdin, NULL, _IONBF, 0);        // No buffering

    // Unused here, but necessary.
    heap_end = _sbrk(0);

    RGB_LED( 0, 40, 0 );
    delay( 100 );
    // Welcome banner
    iprintf("\r\n\r\n====== Freescale Freedom FRDM-KL25Z\r\n");
    iprintf("\r\nBuilt: %s %s\r\n", __DATE__, __TIME__);

    lora_init();
    
    j = 0;
    for(;;) {
	for (i=0; i<68; i++) {
		msg[i] = habtext[i + 68*j];
//		white |= (white << 9 ^ white << 4) & (1<<9);
//		white >>= 1;
	}
	j++;
	if (j >= 8)
		j = 0;;

	for (i = 0; i < 64; i++) {
		ax = accel_x();
		ay = accel_y();
		az = accel_z();
		force = magnitude( ax, ay, az );
		pitch = findArctan( ax, ay, az );
		roll  = findArctan( az, ay, 0 );
		force >>= 2;

		blue = pitch;
		if (blue < 0) blue = -blue;
		if (blue > 80) blue = 80;
		green = roll;
		if (green < 0) green = -green;
		if (green > 80) green = 80;

		red = blue;

		RGB_LED( red, green, blue );
		delay(250);
#if 0 
//receive 4Hz
		lora_rx();	
	}
#else
//transmit 0.25 Hz
	}
	//temp = 15 - (int8_t)(uint8_t)rfm98_temp();

	//iprintf("$$HEX,%d,%2d,%4d,%3d,%3d,",seq++, temp, force, pitch, roll);
	//iprintf("*%x\r\n", checksum);

	//lora_tx("00001111000011110000111100001111");
	lora_tx(msg);
#endif
    }
}
