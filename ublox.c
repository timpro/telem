
// ublox.c -- Max 7C GPS support
//
// Copyright (c) 2013 John Greb <github.com/hexameron>
//
// Code in habduino.h subject to GPL licence

#include "common.h"
#include "habduino.h"

void ublox_update(void)
{
	PORTA_PCR2 =  PORT_PCR_MUX(1); // usb off
	PORTE_PCR20 = PORT_PCR_MUX(4); // ublox on
	ublox_pvt();
	delay( 128 ); 
	PORTE_PCR20 = PORT_PCR_MUX(1); // ublox off
	PORTA_PCR2 =  PORT_PCR_MUX(2); // usb on
}

void ublox_init(void)
{
	// Use MUX(1) to disable ADC on pins used for tx/rx
	// Use MUX(4) for UART0, but that is usb host port
        // Setup UART for Ublox input
	PORTE_PCR21 = PORT_PCR_MUX(1); //rx
	PORTE_PCR20 = PORT_PCR_MUX(4); //tx

	setupGPS(); // turn off all strings
	delay( 128 );
	// Need to start in Flight Mode for mid-air reboots
	// Can set Portable Mode after getting fix
	// setGPS_DynamicMode6();
	setGPS_PowerSaveMode(); // Don`t really want this too soon.

	// allow Uart to empty queue, then switch Ublox to send.
	delay( 128 );
        PORTE_PCR20 = PORT_PCR_MUX(1); //tx
        PORTE_PCR21 = PORT_PCR_MUX(4); //rx
}

void sendUBX(uint8_t *data, uint8_t len )
{
	char wakeup = 0xff;
	//need to wake ublox, then wait 100ms (or 500?)
	uart_write( &wakeup, 1);
	delay( 128 );
	uart_write( data, len);
}

