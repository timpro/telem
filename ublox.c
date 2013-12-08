
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

gps_struct gpsdata;
long utime = 0;
short lon_int = 0, lon_dec = 0;
short lat_int = 0, lat_dec = 0;
short altitude = 0, usats = 0;
char  ufix = 0, tslf = 0, upsm = 0, flags;

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
	upsm  = (gpsdata.power >> 2) & 0x7;
	ufix  = gpsdata.fix & 0x7;
	// last digit for powersave, first digit for fix type
	// 2x is a 2d fix, 3x is a 3d fix (none,bad,2d,3d,3d+,time only)
	// x3 is tracking, x4 is lowpower (none,on,active,tracking,low power,inactive)
	flags = (ufix * 10) + upsm;
}

void gps_update(void)
{
	// Need to test checksum before processing new data
	flags = ublox_update(&gpsdata);
	if ( 0 == flags ) gps_process();

	// request new data
	ublox_pvt();
}

// Uart 0 on pins E20,E21
void ublox_init(void)
{
	short i;
	char wakeup = 0xff;

	// Ublox needs time to wake up
        uart_write( &wakeup, 1);
	for (i = 0; i < 10; i++)
		radio_tx(0x41 + i);
	setupGPS(); // turn off all strings
	radio_tx(0x30);

	// ** rebooting above 12000m needs Flightmode to get lock
	// setGPS_DynamicMode6();

	// will not enter Powersave without a lock, try later
	// setGPS_PowerSaveMode();
}

// need to wake ublox, then wait 100ms
void sendUBX(char *data, char len )
{
	char wakeup = 0xff;
	uart_write( &wakeup, 1);

	// Timer may have overflowed by now, so this char will be corrupted.
	// .. resync stream and give delay for ublox
	radio_tx(0x2E);
	uart_write( data, len);
}

unsigned short seq = 450;
void gps_output(sensor_struct *sensor)
{
	short quick, len, i;
	unsigned short checksum;

	// check modes every 15 minutes
	if ( !(seq & 63) ){
		// check powersaving mode - may indicate reboot
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
	gps_update();

	quick = (3 & seq++);
	txstring[0] = 0x00; // init a null string
	if (!quick)
 		siprintf(txstring,"$$$17A,%d", seq>>2);
	
        siprintf(txstring,"%s,%02d%02d%02d",txstring, (char)(utime>>16)&31, (char)(utime>>8)&63, (char)utime&63 );
        siprintf(txstring,"%s,%d.%04d,%d.%04d",txstring, lat_int, lat_dec, lon_int, lon_dec );
	len = siprintf(txstring, "%s,%d,%d,%d,", txstring, altitude, usats, flags );
	if (!quick)
	        len = siprintf(txstring,"%s%d,%d,%d,%d,%d", txstring, sensor->force, sensor->compass,
					sensor->pressure, sensor->temperature, sensor->battery);

	checksum = gps_CRC16_checksum (txstring, len);
	len = siprintf(txstring,"%s*%04x\n", txstring, checksum);


	// timer delay will have overflowed by now, need to resync

	//iprintf("%s",txstring);
	char single;
	i = 0;
	while (len--) {
		single = txstring[i++];
		radio_tx(single);
	}
}
