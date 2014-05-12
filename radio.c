// (c) 2013 John Greb, MIT License.

#include "common.h"

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

// Rtty shift for 0xE0 is 240Hz
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

		// 240 shift, 0xE0 or 0x00
		DAC0_DAT0L =  (char)((bit << 5)+(bit << 6)+(bit << 7));
	}
//	lpdelay(); // extra stop bit
}

uint16_t checksum = 0xdead;
uint16_t hamptr = 0;
char 	hamchars[128];
char	hamrtty_lut[] = {0x01,0x0e,0x14,0x1b,0x27,0x28,0x32,0x3d,0x42,0x4d,0x57,0x58,0x64,0x6b,0x71,0x7e};

void hamming_tx(void)
{
#ifdef HAMMING
	short j;
	char c;

	rtty_tx(0x7e);
	for (j = 0; j< hamptr; j++) {
		c = hamchars[j];
		if (c == 44 || c == 47 )
                         c = 47 + 44 - c;
		if ( c>=46 && c<=(48 + 9) ){
			rtty_tx( hamrtty_lut[c - 42] );
		} else {
			if (c==44) c = 47;
			if (c==10) c = 44;
			if (c==13) c = 46;
			c -= 32;
			if (c>=64) c -= 32;
			rtty_tx( hamrtty_lut[0x3 & c] );
			c >>= 2;
			rtty_tx( hamrtty_lut[0xf & c] );
		}
	}
	hamptr = 0;
	hamchars[hamptr++] = 0x0a;
	rtty_tx(0x0a);
#endif
}

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
#ifdef HAMMING
	hamchars[hamptr++] = x;
	if (x == 0x0a)	hamming_tx();
#endif
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
#ifdef HAMMING
	hamchars[hamptr++] = 0x2a;
#endif
	while ( y > 0 ) {
		y -=4 ;
		x = (char)( (checksum >> y) & 0x000f );
		if (x > 9) x+= (65 - 48 - 10);
		x += 48;
		rtty_tx( x );
#ifdef HAMMING
		hamchars[hamptr++] = x;
#endif
	}
}

