//
//  Copyright (c) 2013-2014 John Greb <github.com/hexameron>
//

//#include <stdio.h>
#include "common.h"

extern char *_sbrk(int len);
static char *heap_end;
sensor_struct sensor;

// ADC on pin E29
void adc_init(void)
{
        SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
        SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
        ADC0_CFG1 = 0x84; // low power 12 bit
        ADC0_CFG2 = 1<<4; // select channel B
        ADC0_SC2 =  0x00; // software trigger
        ADC0_SC3 =  0x04; // single shot, average of 4
        ADC0_SC1A = 0x04; // channel 4 start
}

// Battery Voltage on potential divider on ADC0_4B
// result depends on resistors and regulator
// 11 bits on a 2v supply would be 2/2048 => almost 1000:1
short read_adc(void)
{
        short result = ADC0_R(0);
        ADC0_SC1A = 0x04; // channel 4 restart
        return (result >> 1);
}

// Main program
int main(void)
{
    short skip = 0;
    short ax = 0;
    short ay = 0;
    short az = 4096;
    unsigned short force;
//    short pitch, roll, red, green;

    RGB_LED( 0, 50, 0 );
    // Initialize all modules
    uart_init(9600);
    lpdelay_init();
//    hal_i2c_init(I2C1_BASE_PTR);	// Setup I2C for EVK
    accel_init();
//    baro_init();
//    mag_init();
//    dac_init();
    adc_init();
    ublox_init();
    accel_init();
    rfm98_init();
    lora_init();

    //setvbuf(stdin, NULL, _IONBF, 0);        // No buffering

    // Unused here, but necessary.
    heap_end = _sbrk(0);

    //iprintf("\r\nBuilt: %s %s\r\n", __DATE__, __TIME__);
    //iprintf("Ident, Count, time, lat, lon, alt, sats, flags, G, yaw, press, temp, batt *Chksum\r\n");

    lpdelay();
    RGB_LED( 0, 0, 0 );

    for(;;) {
	//accel_read();	
	ax = accel_x();
	ay = accel_y();
	az = accel_z();
#if 0
	pitch = findArctan( ax, ay, az );
	roll  = findArctan( az, ay, 0 );
	red = pitch;
        if (red < 0) red = -red;
        if (red > 80) red = 80;
        green = roll;
        if (green < 0) green = -green;
        if (green > 80) green = 80;
	RGB_LED(red >> 2, green >> 2, 0);
#endif
	force = magnitude( ax, ay, az );
	force += force >> 5;
	force -= force >> 4;
	force += force >> 8;
	sensor.force = force >> 2;
	sensor.compass = 0;  //mag_compass(ax, ay, az);

//	sensor.pressure = get_pressure();
	sensor.temperature = rfm98_temp();
	sensor.battery = read_adc();

	lpdelay();
	if (!(skip++ & 7))
		gps_output(&sensor);
	lpdelay();
    }
}
