
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

	// request new data
	ublox_pvt();
}

// Uart 0 on pins E20,E21
void ublox_init(void)
{
	short i;
	// Use MUX(1) to disable pins used for tx/rx
	// Use MUX(4) for UART0, normally used for usb debug port
        // Setup UART for Ublox input
	PORTE_PCR21 = PORT_PCR_MUX(4); // uart tx, gps rx
	PORTE_PCR20 = PORT_PCR_MUX(4); // uart rx, gps tx

	// Ublox needs time to wake up
	for (i = 0; i < 26; i++)
		radio_tx(0x40 + i);
	setupGPS(); // turn off all strings
	radio_tx(0x30);

	// ** rebooting above 12000m needs Flightmode to get lock
	// setGPS_DynamicMode6();

	setGPS_PowerSaveMode(); // Seems to work well even indoors
	radio_tx(0x31);
}

// need to wake ublox, then wait 100ms
void sendUBX(char *data, char len )
{
	char wakeup = 0xff;
	uart_write( &wakeup, 1);
	radio_tx(0x2E); // perhaps 1/5th of a second
	uart_write( data, len);
}

unsigned short seq = 401;
void gps_output(sensor_struct *sensor)
{
	short quick, len, i;
	unsigned short checksum;

	// check modes every 10 minutes
	if ( !(seq & 63) ){
		// check powersaving mode - may indicate reboot
		if (0 == upsm)
			ublox_init();

		// check altitude to set flight mode
		if (altitude > 2000){
			radio_tx(0x46);
			setGPS_DynamicMode6();
		}
		if (altitude < 1000){
			radio_tx(0x47);
			void setGPS_DynamicMode3();
		}
		radio_tx(0x5a);
	}
	gps_update();

	quick = (3 & seq++);
	txstring[0] = 0x21; // "!"
	txstring[1] = 0x00;
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

	//iprintf("%s",txstring);
	char single;
	i = 0;
	while (len--) {
		single = txstring[i++];
		radio_tx(single);
	}
}
