//
// uart.c -- Serial I/O
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
//  Copyright (c) 2013 John Greb
//

#include "common.h"

// Circular buffers for transmit and receive
#define BUFLEN 128

static uint8_t _tx_buffer[sizeof(RingBuffer) + BUFLEN] __attribute__ ((aligned(4)));
static uint8_t _rx_buffer[sizeof(RingBuffer) + BUFLEN] __attribute__ ((aligned(4)));

static RingBuffer *const tx_buffer = (RingBuffer *) &_tx_buffer;
static RingBuffer *const rx_buffer = (RingBuffer *) &_rx_buffer;

// Buffers for Ublox replies
char NAV_PVT[92];  //everything you need in one function.
	//    0xB5, 0x62, 0x01, 0x07, 0x54, 0x00,             // 6 header,
	//    ....        0x00, 0x00 };          //  + 84 bytes + 2 checksum
// Ack/Nak are mostly useful for debugging, we don`t need to check those.
// We do need to know that we are in flight mode, and NMEA strings are off
// - but it is as easy to re-send the command as it is to check current status.

/* CRITICAL Section, do not break */
// 9600 baud is quite slow: data processing needs to be done in the background
// so that all data can be captured without waiting, discarding or overwriting.
// BUT interrupts need to be short and simple :
// Reads are filtered into specific buffers, with minimum processing.
// 
/* Interrupt States:
 *	0: waiting for header
 *	1,2: got 0xB5,0x62
 *	3,4: checked headerID
 *	5: checked size byte
 *	6: read that many bytes + 2 for checksum
 */
short pos, type, count;
void UART0_IRQHandler()
{
    int status;
    char inbyte;
    status = UART0_S1;
    
    // If transmit data register empty, and data in the transmit buffer,
    // send it.  If it leaves the buffer empty, disable the transmit interrupt.
    if ((status & UART_S1_TDRE_MASK) && !buf_isempty(tx_buffer)) {
        UART0_D = buf_get_byte(tx_buffer);
        if(buf_isempty(tx_buffer))
            UART0_C2 &= ~UART_C2_TIE_MASK;
    }

    // If there is received data, process it. Interrupts are NOT disabled, be quick 
    if ( status & UART_S1_RDRF_MASK ) {
        inbyte = UART0_D;
        //UART0_C2 &= ~UART_C2_RIE_MASK;
	switch (pos) {
		case 0: if (0xB5 == inbyte) pos++;
			break;
		case 1: if (0x62 == inbyte) pos++;
			  else pos = 0;
			break;
		case 2: if (0x01 == inbyte) pos++;
			  else pos = 0;
			break;
		case 3: if (0x07 == inbyte) pos++;
			  else pos = 0;
			break;
		case 4: if (0x54 == inbyte) {
				pos++;
				type = 0;
			} else pos = 0;
			break;
		case 5: if (0x00 == inbyte) {
					pos++;
					count = 0;
			} else pos = 0;
			break;
		case 6: NAV_PVT[count++] = inbyte;
			if (count >= 86) pos = 0;
			break;
		default:pos = 0;
			break;
	}
    }
}

// parse the checksum, and copy the data
// checksum will pass if data is all zeros at boot
short ublox_update(gps_struct *gpsdata)
{
	short i;
	char  chk0, chk1, chka, chkb;
	// copy checksum first in case buffer is updating while we are reading
	chka =  NAV_PVT[84];
	chkb =  NAV_PVT[85];
	// then copy all wanted data
	gpsdata->utc = (0 | NAV_PVT[8]<<16 | NAV_PVT[9]<<8 | NAV_PVT[10]<<0);
	gpsdata->lon = (0 | NAV_PVT[25]<< 8 | NAV_PVT[26] << 16 | NAV_PVT[27] << 24);
	gpsdata->lat = (0 | NAV_PVT[29]<< 8 | NAV_PVT[30] << 16 | NAV_PVT[31] << 24);
	gpsdata->alt = (0 | NAV_PVT[33]<< 8 | NAV_PVT[34] << 16 | NAV_PVT[35] << 24);
	gpsdata->sats = (0 | NAV_PVT[23]);

	gpsdata->power = NAV_PVT[21];
	gpsdata->fix  =  NAV_PVT[20];

	// 0x01075400 header +84 data bytes
	chk0 = 0x08;
	chk1 = 0x09;
	chk0 += 0x54;
	chk1 += chk0;
	chk1 += chk0;
	for (i = 0; i < 84; i++)
	{
		chk0 += NAV_PVT[i];
		chk1 += chk0;
	}
	if (chk0 != chka)return 99;
	if (chk1 != chkb)return 99;
	return 0;
}

int uart_write(char *p, int len)
{
    int i;
    
    for(i=0; i<len; i++) {
        while(buf_isfull(tx_buffer))        // Spin wait while full
            ;
        buf_put_byte(tx_buffer, *p++);
        UART0_C2 |= UART_C2_TIE_MASK;           // Turn on Tx interrupts
    }
    return len;
}

// A blocking write, useful for error/crash/debug reporting
int uart_write_err(char *p, int len)
{
    int i;
    
    __disable_irq();
    for(i=0; i<len; i++) {
        while((UART0_S1 & UART_S1_TDRE_MASK) == 0)  // Wait until transmit buffer empty
            ;
        
        UART0_D = *p++;                     // Send char
    }
    __enable_irq();
    return len;
}

int uart_read(char *p, int len)
{
    int i = len;

    while(i > 0) {
        while(buf_isempty(rx_buffer))           // Spin wait
            ;

        *p++ = buf_get_byte(rx_buffer);
        UART0_C2 |= UART_C2_RIE_MASK;           // Turn on Rx interrupt
        i--;
    }
    return len - i;
}

//
// uart_init() -- Initialize debug / OpenSDA UART0
//
//      The OpenSDA UART RX/TX is connected to pins 27/28, PTA1/PTA2 (ALT2)
//
void uart_init(int baud_rate)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
        
    // Turn on clock to UART0 module and select 48Mhz clock (FLL/PLL source)
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
    SIM_SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
    SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1);                 // FLL/PLL source

    // Select "Alt 2" usage to enable UART0 on pins
    // Select "Alt 1" to disable OpenSDA from uart
    PORTA_PCR1 = PORT_PCR_MUX(1); //rx
    PORTA_PCR2 = PORT_PCR_MUX(2); //tx

    UART0_C2 = 0;
    UART0_C1 = 0;
    UART0_C3 = 0;
    UART0_S2 = 0;     

    // Set the baud rate divisor
    #define OVER_SAMPLE 16
    uint16_t divisor = (CORE_CLOCK / OVER_SAMPLE) / baud_rate;
    UART0_C4 = UARTLP_C4_OSR(OVER_SAMPLE - 1);
    UART0_BDH = (divisor >> 8) & UARTLP_BDH_SBR_MASK;
    UART0_BDL = (divisor & UARTLP_BDL_SBR_MASK);

    // Initialize transmit and receive circular buffers
    buf_reset(tx_buffer, BUFLEN);
    buf_reset(rx_buffer, BUFLEN);

    // Enable the transmitter, receiver, and receive interrupts
    UART0_C2 = UARTLP_C2_RE_MASK | UARTLP_C2_TE_MASK | UART_C2_RIE_MASK;
    enable_irq(INT_UART0);
}
