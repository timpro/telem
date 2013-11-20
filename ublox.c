
// ublox.c -- Max 7C GPS support
//
// Copyright (c) 2013 John Greb <github.com/hexameron>
//
// Code in habduino.h subject to GPL licence

#include "common.h"
#include "habduino.h"


void gps_get_data()
{
}

void ublox_init(void)
{
    // Use MUX(1) to disable ADC on pins used for tx/rx
    // Use MUX(4) for UART0, but that is usb host port
    PORTE_PCR20 = PORT_PCR_MUX(1); //tx
    PORTE_PCR21 = PORT_PCR_MUX(4); //rx
}

void sendUBX(uint8_t *data, uint8_t len )
{
}

