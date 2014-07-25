
// ublox.c -- Max 7C GPS support
//
// Copyright (c) 2013 John Greb <github.com/hexameron>
//
// Code in habduino.h subject to GPL licence

#include <stdio.h>
#include "common.h"
#include "habduino.h"

gps_struct gpsdata;
long utime = 0;
short lon_sign = 0;
unsigned short lon_int = 0, lon_dec = 0;
unsigned short lat_int = 0, lat_dec = 0;
unsigned short altitude = 0, usats = 0;
char  ufix = 0, upsm = 0, flags=0;

// Need to copy good values before updating
void gps_process(void)
{
	utime = gpsdata.utc;
	long ulon  = gpsdata.lon;
        // 4 bytes of latitude/longitude (1e-7)
        // divide by 1000 to leave degrees + 4 digits and +/-5m accuracy
	// - which could be done more efficiently on the server
        if (ulon < 0) {
		lon_sign = -1;
                ulon = -ulon;
        } else
		lon_sign = 1;
        ulon += 500;
        ulon /= 1000;
        lon_int = (unsigned short) (ulon / 10000);
        lon_dec = (unsigned short) (ulon - (long)lon_int*10000);

	long ulat  = gpsdata.lat;
	if (ulat < 0)
		ulat = 0; //WTF
	ulat += 500;
	ulat /= 1000;
	lat_int = (unsigned short) (ulat/10000);
	lat_dec = (unsigned short) (ulat - (long)lat_int*10000) ;

	long ualt  = gpsdata.alt;
	if (ualt < 0)
		ualt = 0;
	// 4 bytes of altitude above MSL (mm)
	// Scale to meters (Within accuracy of GPS)
	ualt >>= 8;
	ualt *= 2097;
	altitude = (unsigned short)(ualt >> 13);

	usats = gpsdata.sats;
	upsm  = (gpsdata.power >> 2) & 0x7;
	ufix  = gpsdata.fix & 0x7;
	// last digit for powersave, first digit for fix type
	// 2x is a 2d fix, 3x is a 3d fix (none,bad,2d,3d,3d+,time only)
	// y3 is tracking, y4 is lowpower (none,on,active,tracking,low power,inactive)
	flags = (ufix * 10) + upsm;
}

// Uart 0 on pins E20,E21
void ublox_init(void)
{
	short i;
	char wakeup = 0xff;

	radio_tx(0x7E);
	// Ublox needs time to wake up
	for (i = 0; i < 8; i++) {
		uart_write( &wakeup, 1);
		radio_tx(0x30 + i);
	}
	setupGPS(); // turn off all strings
	radio_tx(0x0A);
	setGPS_PowerManagement();

	// ** rebooting above 12000m needs Flightmode to get lock
	// setGPS_DynamicMode6();

	// will not enter Powersave without a lock, try later
	// setGPS_PowerSaveMode();
}

// Need to wake ublox then wait 100ms, not apparent on Max7C
void sendUBX(char *data, short len)
{
	short i;
	char chk0 = 0;
	char chk1 = 0;

	if (len < 8) return; // need a header and a checksum

	// calculate checksum, even if it was pre-calculated
	for (i = 2; i < (len - 2); i++)
	{
		chk0 += data[i];
		chk1 += chk0;
	}
	data[i++] = chk0;
	data[i++] = chk1;

	uart_write( data, len );
}

// replacing newlib version
void myprintf(unsigned short s, short d)
{
	long x;
	short z, p, e;
	e = 1; // number of digits
	x = 1; // scale
	while (x*10 <= s) {
		x *= 10;
		e++;
	}
	if (e < d) e = d;
	while (--e >= 0){
		p = 0;
		x = 1;
		z = 0;
		while (z++ < e) x *= 10;
		while (s >= x) {
			p++;
			s -= x;
		}
		radio_tx(0x30 + (char)(p&15));
	}
}

void signprintf(short s, short d) {
	if ( s < 0 ) {
		s = -s;
		radio_tx(0x2d);
	}
	myprintf(s, d);
}

unsigned short seq = 400;
void gps_output(sensor_struct *sensor)
{
	short quick;

	radio_tx(0x7E);  // mostly stop bits

	// check modes every 8 minutes
	if ( !(seq & 31) ){
		// check powersaving mode
		if (0 == upsm){
			radio_tx(0x50); // P.ower
			setGPS_PowerSaveMode();
		}
		// check altitude to set flight mode
		if (altitude > 2000){
			radio_tx(0x46); // F.light
			setGPS_DynamicMode6();
		}
		// leave flight mode when good fix on the ground
		if ((3 == ufix) && (altitude < 1000)){
			radio_tx(0x47); //G.round
			setGPS_DynamicMode3();
		}
	}

	quick = (0 & seq++);
	if (!quick){
//		siprintf(txstring,"$$17A,%d", seq>>2);
		radio_tx(0x24);
		radio_tx(0x24);

		sendChecksum(0);
		radio_tx(0x31);
		radio_tx(0x37);
		radio_tx(0x41);
		radio_tx(0x2c);

		myprintf( seq, 3 );
		radio_tx(0x2c);
	}

	// process data, if it has properly updated
	flags = ublox_update(&gpsdata);
        if ( 0 == flags ) gps_process();

	myprintf( (char)(utime>>16)&31,2 );
	//radio_tx(0x3a);
	myprintf( (char)(utime>>8)&63, 2 );
	//radio_tx(0x3a);
	myprintf( (char)(utime>>0)&63, 2 );
	radio_tx(0x2c);

	myprintf( lat_int, 1 );
	radio_tx(0x2e);
	myprintf( lat_dec, 4 );
	radio_tx(0x2c);
	if ( lon_sign < 0 )
		radio_tx(0x2d);
	myprintf( lon_int, 1 );
	radio_tx(0x2e);
	myprintf( lon_dec, 4 );
	radio_tx(0x2c);
	myprintf( altitude, 3 );

	if (!quick)
	{
	radio_tx(0x2c);
	myprintf( usats, 1 );
	radio_tx(0x2c);
	myprintf( flags, 2 );
	radio_tx(0x2c);
	myprintf( sensor->force, 1 );
	radio_tx(0x2c);
	signprintf( sensor->compass, 3 );
	radio_tx(0x2c);
	myprintf( sensor->pressure, 1 );
	radio_tx(0x2c);
	signprintf( sensor->temperature, 1 );
	radio_tx(0x2c);
	signprintf( sensor->battery, 1 );
	sendChecksum(1);
	}

	// request new data
	ublox_pvt();

	radio_tx(0x0a);
}
