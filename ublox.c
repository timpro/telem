
// ublox.c -- Max 7C GPS support
//
// Copyright (c) 2013 John Greb <github.com/hexameron>
//
// Code in habduino.h subject to GPL licence

#include <stdio.h>
#include "common.h"
#include "habduino.h"

// output buffer. Needs to be large enough for sprintf output
char txstring[80];

//padding text: 2^7 strings of 24 chars == 3072
char padding[3072] = {
#include "snowcrash.txt"
};

long  utime = 0, ulat = 0, ulon = 0, ualt = 0;
short usats = 0;
char  lock = 0, tslf = 0, checkfail = 0, psm = 0;
short navmode = 0, psm_status = 0;

#if 0
  // Switch to/from Flight mode by altitude
  if ( !(++tslf & 15) ) {
      // should not need to check very often.
      gps_check_nav();
      gps_check_mode();
  }
#endif

// Need to copy good values before updating
void gps_process(void)
{
       utime = ublox_time();
       ulon  = ublox_lon();
       ulat  = ublox_lat();
       ualt  = ublox_alt();
       usats = ublox_sats();
}

void gps_update(void)
{
	// Need to test checksum before copying data and processing
	checkfail = ublox_update();
	if ( !checkfail ) gps_process();

	// switch between usb and ublox on same uart
	PORTA_PCR2 =  PORT_PCR_MUX(1); // to usb off
	PORTE_PCR20 = PORT_PCR_MUX(4); // to ublox on
	// request new data
	ublox_pvt();
	delay( 128 ); // delay only needed for usb debug
	PORTE_PCR20 = PORT_PCR_MUX(1); // to ublox off
	PORTA_PCR2 =  PORT_PCR_MUX(2); // to usb on
}

void ublox_init(void)
{
	// Use MUX(1) to disable pins used for tx/rx
	// Use MUX(4) for UART0, but that is usb host port
        // Setup UART for Ublox input
	PORTA_PCR2 =  PORT_PCR_MUX(1); // usb off
	PORTE_PCR21 = PORT_PCR_MUX(4); //rx
	PORTE_PCR20 = PORT_PCR_MUX(4); //tx

	setupGPS(); // turn off all strings
	delay( 128 );
	// May need to start in Flight Mode for mid-air reboots
	// Can set Portable Mode after getting fix
	// setGPS_DynamicMode6();
	setGPS_PowerSaveMode(); // Don`t really want this too soon.

	// allow Uart to empty queue, then switch Ublox to send.
	delay( 128 );
        PORTE_PCR20 = PORT_PCR_MUX(1); // stop tx to gps
        PORTA_PCR2 =  PORT_PCR_MUX(2); // start tx to usb
}

void sendUBX(char *data, char len )
{
	char wakeup = 0xff;
	//need to wake ublox, then wait 100ms (or 500?)
	uart_write( &wakeup, 1);
	delay( 128 );
	uart_write( data, len);
}

short lon_int, lon_dec, lat_int, lat_dec;
void find_pos()
{
	long lon =  ulon;
	// 4 bytes of latitude/longitude (1e-7)
	// divide by 1000 to leave degrees + 4 digits and +/-5m accuracy
	if (lon < 0) {
		lon -= 500;
		lon /= 1000;
		lon_int = (short) (lon / 10000);
		lon_dec = (short) ((long)lon_int*10000 - lon);
	} else {
		lon += 500;
		lon /= 1000;
		lon_int = (short) (lon / 10000);
		lon_dec = (short) (lon - (long)lon_int*10000);
	}

	long lat =  ulat;
	// may be less than zero, which would be bad
	lat += 500;
	lat /= 1000;
	lat_int = (short) (lat/10000);
	lat_dec = (short) (lat - (long)lat_int*10000) ;
}

unsigned short seq = 100;
unsigned short padcount = 0;
unsigned short checksum = 0xdead;
void gps_output(short force, short compass, short pressure,
		short temperature, short battery )
{
	short errorcode, quick, len, i;
	unsigned short stringcount;
	long alt;

	gps_update();
	find_pos();
	errorcode = checkfail;

	// 4 bytes of altitude above MSL (mm)
	// Scale to meters (Within accuracy of GPS)
	alt = ualt >> 8;
	alt *= 2097;
	alt >>= 13;

	quick = (3&seq++);
	txstring[0] = 0x0;
	if (quick) {
		stringcount = 24 * (127 & padcount++);
                for (i = 0; i < 24; i++) 
                        txstring[i] = padding[stringcount++];
                txstring[i] = 0x00;
	} else siprintf(txstring,"$$$17A,%d", seq>>2);
	
        siprintf(txstring,"%s,%02d%02d%02d",txstring, (char)(utime>>16)&31, (char)(utime>>8)&63, (char)utime&63 );
        siprintf(txstring,"%s,%d.%04d,%d.%04d",txstring, lat_int, lat_dec, lon_int, lon_dec );
	len = siprintf(txstring, "%s,%d,%d,%d,", txstring, (short)alt, usats, errorcode );
	if (!quick)
	        len = siprintf(txstring,"%s%d,%d,%d,%d,%d", txstring, force, compass, pressure, temperature, battery);

	checksum = gps_CRC16_checksum (txstring, len);
	len = siprintf(txstring,"%s*%04x\r\n", txstring, checksum);

	char single;
	i = 0;
	while (len--) {
		single = txstring[i++];
		iprintf("%c", single);
		delay(128);
	}
}
