
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

gps_struct gpsdata;
long utime = 0;
short lon_int = 0, lon_dec = 0;
short lat_int = 0, lat_dec = 0;
short altitude = 0, usats = 0;
char  ufix = 0, tslf = 0, upsm = 0, flags;

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
	utime = gpsdata.utc;
	long ulon  = gpsdata.lon;
        // 4 bytes of latitude/longitude (1e-7)
        // divide by 1000 to leave degrees + 4 digits and +/-5m accuracy
        if (ulon < 0) {
                ulon -= 500;
                ulon /= 1000;
                lon_int = (short) (ulon / 10000);
                lon_dec = (short) ((long)lon_int*10000 - ulon);
        } else {
                ulon += 500;
                ulon /= 1000;
                lon_int = (short) (ulon / 10000);
                lon_dec = (short) (ulon - (long)lon_int*10000);
        }

	long ulat  = gpsdata.lat;
	ulat += 500;
	ulat /= 1000;
	lat_int = (short) (ulat/10000);
	lat_dec = (short) (ulat - (long)lat_int*10000) ;

	long ualt  = gpsdata.alt;
	// 4 bytes of altitude above MSL (mm)
	// Scale to meters (Within accuracy of GPS)
	ualt >>= 8;
	ualt *= 2097;
	altitude = (short)(ualt >> 13);

	usats = gpsdata.sats;
	upsm  = gpsdata.power;
	ufix  = gpsdata.fix & 0x7;
	// three bits for powersave, three for fix type
	// 2x is a 2d fix, 3x is a 3d fix (none,bad,2d,3d,3d+,time only)
	// x3 is tracking, x4 is lowpower (none,on,active,tracking,low power,inactive)
	flags = (ufix * 10) + ((upsm >> 2) & 7);
}

void gps_update(void)
{
	// Need to test checksum before processing new data
	flags = ublox_update(&gpsdata);
	if ( 0 == flags ) gps_process();

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
	// May need to start in Flight Mode for mid-air reboots
	// setGPS_DynamicMode6();
	setGPS_PowerSaveMode(); // Seems to work well even indoors

	// allow Uart to empty queue, then switch ouput to USB.
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

unsigned short seq = 400;
unsigned short padcount = 0;
void gps_output(short force, short compass, short pressure,
		short temperature, short battery )
{
	short quick, len, i;
	unsigned short checksum, stringcount;

	gps_update();

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
	len = siprintf(txstring, "%s,%d,%d,%d,", txstring, altitude, usats, flags );
	if (!quick)
	        len = siprintf(txstring,"%s%d,%d,%d,%d,%d", txstring, force, compass, pressure, temperature, battery);

	checksum = gps_CRC16_checksum (txstring, len);
	len = siprintf(txstring,"%s*%04x\r\n", txstring, checksum);

	iprintf("%s",txstring);
	char single;
	i = 0;
	while (len--) {
		single = txstring[i++];
		domino_tx(single);
	}
}
