// (c) 2013 John Greb, MIT License.
#include "common.h"
const char domino[96][3] = {
/* 32 */{ 0, 0, 0}, { 7,11, 0}, { 0, 8,14}, { 0,10,11}, { 0, 9,10}, { 0, 9, 9}, { 0, 8,15}, { 7,10, 0},
        { 0, 8,12}, { 0, 8,11}, { 0, 9,13}, { 0, 8, 8}, { 2,11, 0}, { 7,14, 0}, { 7,13, 0}, { 0, 8, 9},
	{ 3,15, 0}, { 4,10, 0}, { 4,15, 0}, { 5, 9, 0}, { 6, 8, 0}, { 5,12, 0}, { 5,14, 0}, { 6,12, 0},
	{ 6,11, 0}, { 6,14, 0}, { 0, 8,10}, { 0, 8,13}, { 0,10, 8}, { 7,15, 0}, { 0, 9,15}, { 7,12, 0},
/* 64 */{ 0, 9, 8}, { 3, 9, 0}, { 4,14, 0}, { 3,12, 0}, { 3,14, 0}, { 3, 8, 0}, { 4,12, 0}, { 5, 8, 0},
	{ 5,10, 0}, { 3,10, 0}, { 7, 8, 0}, { 6,10, 0}, { 4,11, 0}, { 4, 8, 0}, { 4,13, 0}, { 3,11, 0},
	{ 4, 9, 0}, { 6,15, 0}, { 3,13, 0}, { 2,15, 0}, { 2,14, 0}, { 5,11, 0}, { 6,13, 0}, { 5,13, 0},
	{ 5,15, 0}, { 6, 9, 0}, { 7, 9, 0}, { 0,10,14}, { 0,10, 9}, { 0,10,15}, { 0,10,10}, { 0, 9,12},
/* 96 */{ 0, 9,11}, { 4, 0, 0}, { 1,11, 0}, { 0,12, 0}, { 0,11, 0}, { 1, 0, 0}, { 0,15, 0}, { 1, 9, 0},
	{ 0,10, 0}, { 5, 0, 0}, { 2,10, 0}, { 1,14, 0}, { 0, 9, 0}, { 0,14, 0}, { 6, 0, 0}, { 3, 0, 0}, 
	{ 1, 8, 0}, { 2, 8, 0}, { 7, 0, 0}, { 0, 8, 0}, { 2, 0, 0}, { 0,13, 0}, { 1,13, 0}, { 1,12, 0}, 
	{ 1,15, 0}, { 1,10, 0}, { 2, 9, 0}, { 0,10,12}, { 0, 9,14}, { 0,10,13}, { 0,11, 8}, { 2, 8,10}};

/* Analogue Section */
void dac_init(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	SIM_SCGC6 |= SIM_SCGC6_DAC0_MASK;
	DAC0_C0   |= 0x08; // Low power
	DAC0_C0   |= 0x80; // On
}

// ADC and DAC on pins E29,E30
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
	result -= result >> 4; // scale down 6%
	return (result >> 1);
}
// dummy IRQ handler
void DAC0_IRQHandler(void){
	asm("nop");
};

// NTX2B Base Voltage is 1<<11 for 2KHz ((2KHZ/V)*(2V/4))
#define HIGH_VOLTS (8)

// DominoEx: Very sensitive to supply/reference voltage
// Maximum voltage is 4095 bits, domino 8 width is 272 bits
// DominoEx4 is half of step size; DomEx16 is the same
#define STEP_SIZE (15)
short current = 0;
void putsym(char sym)
{
	short voltage;

	// increment symbol, range centered  around zero
	current += 2 + (sym & 0xf);
	if (current > 17) current -= 18;

	voltage = (current * STEP_SIZE);

	lpdelay(); // allow previous sym to timeout
	DAC0_DAT0H = (char)(HIGH_VOLTS | (voltage>>8));
	DAC0_DAT0L = (char)(voltage & 0xff);
}

void domino_tx(char txchar)
{
	short tx = txchar & 127; // short character set
	if (10 == tx) tx = 127; // move newline
	if (tx < 32) return; // no control chars
	tx -= 32;
	putsym( domino[tx][0] );
	if (domino[tx][1]) putsym( domino[tx][1] );
	if (domino[tx][2]) putsym( domino[tx][2] );
}

// Rtty shift for 0xA0 is 170Hz
//  bit shift for 240Hz is (0xE0)
// min shift is 24Hz, (0xE)
// 85 Hz is (0x50)
void rtty_tx(char txchar)
{
	char bit;
	short i;
	txchar |= 1<< 7; // 7N1 bits
	lpdelay(); // delay comes FIRST
	DAC0_DAT0H = (char)HIGH_VOLTS;// timer may have overflowed
	DAC0_DAT0L = (char)0xE0;      // so start with 2nd stop bit
	lpdelay();
	DAC0_DAT0L = (char)0x00;      // start bit
	for (i=0; i<8; i++) {
		bit = 1 & txchar;
		txchar >>= 1;
		lpdelay();

		DAC0_DAT0L =  (char)((bit<<5)|(bit<<6)|(bit<<7));
	}
//	lpdelay(); // extra stop bit
}

uint16_t checksum = 0xdead;
void radio_tx(char x)
{
	short j;
	uint16_t crc;
	crc = checksum ^ ( (uint16_t)x << 8);
	for (j = 0; j < 8; j++)
	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}
	checksum = crc;
	rtty_tx(x);
}

void sendChecksum(short flag)
{
	char x,y;
	if (!flag) {
		checksum = 0xffff;
		return;
	}
	y = 16;
	rtty_tx( 0x2a );
	while ( y > 0 ) {
		y -=4 ;
		x = (char)( (checksum >> y) & 0x000f );
		if (x > 9) x+= (65 - 48 - 10);
		x += 48;
		rtty_tx( x );
	}
}

