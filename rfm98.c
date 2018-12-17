/* (C) 2015 John Greb, MIT Lience */
#include "common.h"
#include "contestia.h"

// to write data set Address MSB High
#define SET_MODE (0x81)
#define REG_FIFO 0
#define REG_OPMODE 0x1
#define REG_BITRATE_MSB 0x2
#define REG_BITRATE_LSB 0x3
#define REG_FDEV_MSB 0x4
#define REG_FDEV_LSB 0x5
#define REG_FRF_MSB 0x6
#define REG_FRF_MID 0x7
#define REG_FRF_LSB 0x8
#define REG_PA_CONFIG 0x9
#define REG_PA_RAMP 0xA
#define REG_OCP 0xB
#define REG_LNA 0xC
#define REG_RX_CONFIG 0xD
#define REG_RSSI_CONFIG 0xE
#define REG_RSSI_COLLISION 0xF
#define REG_RSSI_THRESH 0x10
#define REG_RSSI_VALUE_FSK 0x11
#define REG_RX_BW 0x12
#define REG_AFC_BW 0x13
#define REG_OOK_PEAK 0x14
#define REG_OOK_FIX 0x15
#define REG_AVG 0x16
#define RES17 0x17
#define RES18 0x18
#define RES19 0x19
#define REG_AFC_FEI 0x1A
#define REG_AFC_MSB 0x1B
#define REG_AFC_LSB_FSK 0x1C
#define REG_FEI_MSB_FSK 0x1D
#define REG_FEI_LSB_FSK 0x1E
#define REG_PREAMBLE_DETECT 0x1F
#define REG_RX_TIMEOUT1 0x20
#define REG_RX_TIMEOUT2 0x21
#define REG_RX_TIMEOUT3 0x22
#define REG_RX_DELAY 0x23
#define REG_OSC 0x24
#define REG_PREAMBLE_MSB_FSK 0x25
#define REG_PREAMBLE_LSB_FSK 0x26
#define REG_SYNC_CONFIG 0x27
#define REG_SYNC_VALUE1 0x28
#define REG_SYNC_VALUE2 0x29
#define REG_SYNC_VALUE3 0x2A
#define REG_SYNC_VALUE4 0x2B
#define REG_SYNC_VALUE5 0x2C
#define REG_SYNC_VALUE6 0x2D
#define REG_SYNC_VALUE7 0x2E
#define REG_SYNC_VALUE8 0x2F
#define REG_PACKET_CONFIG1 0x30
#define REG_PACKET_CONFIG2 0x31
#define REG_PAYLOAD_LENGTH_FSK 0x32
#define REG_NODE_ADRS 0x33
#define REG_BROADCAST_ADRS 0x34
#define REG_FIFO_THRESH 0x35
#define REG_SEQ_CONFIG1 0x36
#define REG_SEQ_CONFIG2 0x37
#define REG_TIMER_RESOL 0x38
#define REG_TIMER1_COEF 0x39
#define REG_TIMER2_COEF 0x3A
#define REG_IMAGE_CAL 0x3B
#define REG_TEMP 0x3C
#define REG_LOW_BAT 0x3D
#define REG_IRQ_FLAGS1 0x3E
#define REG_IRQ_FLAGS2 0x3F
#define REG_DIO_MAPPING1 0x40
#define REG_DIO_MAPPING2 0x41
#define REG_VERSION 0x42
#define REG_PLL_HOP 0x44
#define REG_TCXO 0x4B
#define REG_PA_DAC 0x4D
#define REG_FORMER_TEMP 0x5B
#define REG_BITRATE_FRAC 0x5D
#define REG_AGC_REF 0x61
#define REG_AGC_THRESH1 0x62
#define REG_AGC_THRESH2 0x63
#define REG_AGC_THRESH3 0x64
#define REG_PLL 0x70

// SPI
#define xSLAVE (1<<0)
#define xCLOCK (1<<1)
#define xWRITE (1<<2)
#define xREAD  (1<<3)

void pause(void) 
{
	short i;
	for (i = 0; i < 50; i++ )
		__asm("nop");
}
short spi_rw(uint8_t address, uint8_t data)
{
	short i, j, result = 0;
	uint16_t send = ((uint16_t)address << 8) + data;

	GPIOD_PCOR = xCLOCK; // clock low
	pause();
	GPIOD_PCOR = xSLAVE; // slave select
	
	for (i = 15; i >= 0; i--) {  
		pause();
		j = send >> i;
		if (j & 1)
			GPIOD_PSOR = xWRITE; // MOSI
		else
			GPIOD_PCOR = xWRITE; // !MOSI
		pause();
		GPIOD_PSOR = xCLOCK; // clock high
		j = (GPIOD_PDIR >> 3 ) & 1;
		result |= j << i;
		pause();
		GPIOD_PCOR = xCLOCK; // clock low
	}
	pause();
	GPIOD_PSOR = xSLAVE; // slave unselect
	pause();
	GPIOD_PSOR = xCLOCK; // clock high
	return result;
}

short rfm_temp = 0;
// temp is not available in standby
short rfm98_temp(void)
{
	return 15 - (int8_t)(uint8_t)rfm_temp;
}

void fsk_config(void)
{
	lpdelay();
	spi_rw(SET_MODE, 0);	 // sleep
	lpdelay();
	spi_rw(SET_MODE, 1); // standby
	lpdelay();

	//bitrate = 500, divided to 125
	spi_rw(0x80|REG_BITRATE_LSB,0x00);
	spi_rw(0x80|REG_BITRATE_MSB,0xFA);
	spi_rw(0x80|REG_FDEV_LSB, 1); // half shift == deviation, 61Hz steps 
	spi_rw(0x80|REG_FDEV_MSB, 0);
	spi_rw(0x80|REG_PREAMBLE_LSB_FSK, 8);
	// spi_rw(0x80|REG_PREAMBLE_MSB_FSK, 0;
	spi_rw(0x80|REG_PAYLOAD_LENGTH_FSK, 0); // unlimited payload length

	//spi_rw(0x80|REG_SYNC_CONFIG, 0x13); // default 32 bits
	spi_rw(0x80|REG_SYNC_VALUE1, 0x0f); // preamble
	spi_rw(0x80|REG_SYNC_VALUE2, 0xf0); //
	spi_rw(0x80|REG_SYNC_VALUE3, 0x0f); //
	spi_rw(0x80|REG_SYNC_VALUE4, 0xf0); //

	//spi_rw(0x80|REG_PACKET_CONFIG1, 80);
	//spi_rw(0x80|REG_PACKET_CONFIG2, 40);

	// 434.400 MHz
	spi_rw(0x80|REG_FRF_MSB, 0x6c);
	spi_rw(0x80|REG_FRF_MID, 0x99);
	spi_rw(0x80|REG_FRF_LSB, 0x79);

	spi_rw(0x80|REG_PA_RAMP, 0x23); // 500us shaping
	spi_rw(0x80|REG_PA_CONFIG, 0x82); // low power
}

void char2radio(char c)
{
	short i, b;

	for (i = 0; i < 4; i++) {
		b = 0xf0 & (256 - (c & 1)); // LSB in, 4 bits (MSB) out
		c >>= 1;
		b += 0x0f & (256 - (c & 1));
		c >>= 1;	
		spi_rw(0x80, b & 0xff); // push to buffer
	}
}

char tones[8]; // 64bits for Oliva
void fsk_tx(char* message)
{
	int i, t;

	spi_rw(SET_MODE, 0); // sleep
        lpdelay();
        spi_rw(SET_MODE, 0); // standby
        lpdelay();

        // 434.400 MHz
        spi_rw(0x80|REG_FRF_MSB, 0x6c);
        spi_rw(0x80|REG_FRF_MID, 0x99);
        spi_rw(0x80|REG_FRF_LSB, 0x79);

	spi_rw(SET_MODE, 0x42); // synthesis

	// Infinite while loop avoidance :
	if ( spi_rw(REG_IRQ_FLAGS2, 0) & (1<<5) )
		return; // check buffer empty flag works
	if (!( spi_rw(REG_IRQ_FLAGS2, 0) & (1<<6) ))
		return; // check buffer full flag works

	// preamble (max 64 bytes)
	for (i = 0; i < 12; i++)
		spi_rw(0x80, 0x03); // start bits
	for (i = 0; i < 12; i++)
		spi_rw(0x80, 0x3f); // stop bits
	spi_rw(SET_MODE, 0x43); // start transmitting

	for (i = 0; i < 120; i++) { // set MAXIMUM chars to transmit
		if (0 == message[i]) break; // dont send trailing zero after message
		contestia_block(&message[i], tones);

		// fifo threshold is 16 bytes, 250ms at 500 baud
		for (t = 0; t < 4; t++) { // 4 contestia or 8 olivia
			while ( spi_rw(REG_IRQ_FLAGS2, 0) & (1<<5) )
				lpdelay();	// buffer full enough
			char2radio( tones[t] );	
		}
	}
	rfm_temp = spi_rw(0x3c, 0x00);

	while (!( spi_rw(REG_IRQ_FLAGS2, 0) & (1<<6) ))
		lpdelay();	// buffer still not empty
	lpdelay();
	spi_rw(SET_MODE, 0x0); // fsk sleep
}

#define BANDWIDTH_7K8	(0x00)
#define BANDWIDTH_10K4	(0x10)
#define BANDWIDTH_15K6	(0x20)
#define BANDWIDTH_20K8	(0x30)
#define BANDWIDTH_31K2	(0x40)
#define BANDWIDTH_41K7	(0x50)
#define BANDWIDTH_62K5	(0x60)

#define NO_HEADER	(0x01)
#define CRC_ON		(0x04)
#define LOW_DATA_RATE	(0x08)
#define LNA_AGC_ON	(0x04)

#define ERROR_CODING_5	(0x02)
#define ERROR_CODING_6	(0x04)
#define ERROR_CODING_7	(0x06)
#define ERROR_CODING_8	(0x08)

#define SPREADING_7	(0x70)
#define SPREADING_8	(0x80)
#define SPREADING_9	(0x90)
#define SPREADING_10	(0xA0)
#define SPREADING_11    (0xB0)
#define SPREADING_12	(0xC0)

#define MODE_SLEEP_LORA	(0x80)
#define MODE_LORA_SBY	(0x81)
#define MODE_LORA_TX	(0x8b)
#define MODE_LORA_RX1	(0x86)
#define MODE_LORA_RX	(0x85)
#define MODE_LORA_FS	(0x8a)

// 10mW, 5mW, 2mW
#define PA_MAX_UK	(0x88)
#define PA_5mW		(0x85)
#define PA_MIN		(0x82)
#define PA_OFF		(0x00)

#define LNA_MAX_GAIN	(0x23)
#define LNA_OFF_GAIN	(0x00)
#define LNA_LOW_GAIN	(0xC0)

// Registers: MSB SET for write
#define READ_IRQ_FLAGS	(0x12)
#define READ_BYTES_IN	(0x13)
#define READ_RFM_TEMP	(0x3C)
#define FREQ_MSB	(0x86)
#define SET_PA_CONFIG   (0x89)
#define SET_LNA         (0x8C)
#define SET_ADDR_PTR	(0x8d) 
#define SET_TX_BASE     (0x8e)
#define SET_RX_BASE     (0x8f)
#define SET_IRQ_FLAGS	(0x92)
#define MODEM_CONFIG1	(0x9D)
#define MODEM_CONFIG2   (0x9E)
#define SET_PAYLOAD_LEN	(0xA2)
#define MODEM_CONFIG3   (0xA6)

// d5
#define xLORA  (1<<6)

void rfm98_init(void)
{
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	// bitbang spio pins
	GPIOD_PSOR  = 7;	//set 0-2 high
	GPIOD_PDDR |= 7;	//set 0-2 as output
	GPIOD_PDDR &=~xREAD;	//set  3  as input
	GPIOD_PDDR &=~xLORA;	//set  6  as input
	PORTD_PCR0 = PORT_PCR_MUX(1);  // 2=PCS
	PORTD_PCR1 = PORT_PCR_MUX(1);  // 2=SCK - conflicts with blue LED
	PORTD_PCR2 = PORT_PCR_MUX(1);  // 2=MOSI, 4=MISO
	PORTD_PCR3 = PORT_PCR_MUX(1);  // 2=MISO, 4=MOSI
	//PORTD_PCR6 = PORT_PCR_MUX(1);  // Lora Data Ready

	GPIOD_PCOR = xCLOCK; // clock low

	fsk_config();
}

const char callmess[] = {"^MAGNU,434.4,0,6,64,144,8\n"};
void lora_call()
{
	short i;
	char m;

	spi_rw(SET_MODE, MODE_SLEEP_LORA); // sleep
	lpdelay();
	spi_rw(FREQ_MSB    , 0x6C);
	spi_rw(FREQ_MSB + 1, 0x9E);
	spi_rw(FREQ_MSB + 2, 0x20);
	spi_rw(MODEM_CONFIG1, ERROR_CODING_8 | BANDWIDTH_41K7);
	spi_rw(MODEM_CONFIG2, SPREADING_11 | CRC_ON);
	spi_rw(MODEM_CONFIG3, 0); // LDRO Off +  auto LNA gain 

	spi_rw(SET_MODE, MODE_LORA_FS);

	spi_rw(SET_ADDR_PTR, 0x00);
        spi_rw(SET_TX_BASE, 0x00);

        for (i=0; (i<32)&&(callmess[i]>0); i++) {
                m = callmess[i];
                spi_rw(0x80, (uint8_t)m);
        }
        spi_rw(0x80, (uint8_t)0);
        spi_rw(SET_MODE, MODE_LORA_TX | 0x40);
        lpdelay();
}

void lora_init()
{
	spi_rw(SET_MODE, MODE_SLEEP_LORA); // sleep
	lpdelay();
	spi_rw(SET_MODE, MODE_SLEEP_LORA); // lora
	lpdelay();
	spi_rw(SET_MODE, MODE_LORA_SBY);
	lpdelay();
	spi_rw(FREQ_MSB    , 0x6C);
	spi_rw(FREQ_MSB + 1, 0x99);
	spi_rw(FREQ_MSB + 2, 0x79); // 5 ppm correction
	//  ERROR_CODING_8 | BANDWIDTH_10K4
	//  ERROR_CODING_8 | BANDWIDTH_7K8
	//  ERROR_CODING_5 | BANDWIDTH_41K7

	spi_rw(MODEM_CONFIG1, ERROR_CODING_8 | BANDWIDTH_62K5 );// | NO_HEADER);
	spi_rw(MODEM_CONFIG2, SPREADING_8 | CRC_ON);
	spi_rw(MODEM_CONFIG3, 0); // | LOW_DATA_RATE);
	spi_rw(SET_TX_BASE, 0x00);
	spi_rw(SET_PAYLOAD_LEN, 80);
	//spi_rw(SET_PA_CONFIG, PA_OFF);
	//spi_rw(SET_PA_CONFIG, PA_MAX_UK);
	spi_rw(SET_PA_CONFIG, PA_MIN); // !!!!!!!!! <<<<<<<<<<<<<<< 

	spi_rw(SET_IRQ_FLAGS, 0xff); // clear all
}

uint16_t checksum = 0xdead;
void lora_tx(char* message)
{
	short i, j;
	char m;

        spi_rw(SET_MODE, MODE_SLEEP_LORA); // sleep
        lpdelay();
        spi_rw(SET_MODE, MODE_SLEEP_LORA); // lora
        lpdelay();
	spi_rw(MODEM_CONFIG1, ERROR_CODING_8 | BANDWIDTH_62K5 ); //| NO_HEADER);
	spi_rw(MODEM_CONFIG2, SPREADING_8 | CRC_ON);
	spi_rw(MODEM_CONFIG3, 0); // | LOW_DATA_RATE);
	spi_rw(SET_PAYLOAD_LEN, 80);

	spi_rw(SET_MODE, MODE_LORA_FS); // Warm up transmit

	spi_rw(SET_ADDR_PTR, 0x00);
	spi_rw(SET_TX_BASE, 0x00);

	uint16_t loracheck = 0xffff;
	for (i=0; i<18; i++) {
		m = message[i];
		uint16_t crc = loracheck ^ ( (uint16_t)m << 8);
		spi_rw(0x80, (uint8_t)m);
		for (j = 0; j < 8; j++) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc <<= 1;
		}
		loracheck = crc;
	}
	spi_rw(0x80, (uint8_t)(loracheck&0xff));
	spi_rw(0x80, (uint8_t)(loracheck >> 8));
	spi_rw(SET_MODE, MODE_LORA_TX | 0x40);
	lpdelay();
}

char tx_msg[128];
short tx_ptr = 0;
void radio_tx(char x)
{
        short j;
        uint16_t crc;

	tx_msg[tx_ptr++] = x;

        crc = checksum ^ ( (uint16_t)x << 8);
        for (j = 0; j < 8; j++)
        {
                if (crc & 0x8000)
                        crc = (crc << 1) ^ 0x1021;
                else
                        crc <<= 1;
        }
        checksum = crc;
}

short skip = 0;
void sendChecksum(short flag)
{
        char x,y;
	uint16_t c = checksum;

        if (!flag) {
                checksum = 0xffff;
                return;
        }
        y = 16;
        radio_tx( 0x2a );
        while ( y > 0 ) {
                y -=4 ;
                x = (char)( (c >> y) & 0x000f );
                if (x > 9) x+= (65 - 48 - 10);
                x += 48;
                radio_tx( x );
        }
	radio_tx( 10 );
	radio_tx( 0 );
	if (1 & skip++)
		fsk_tx(tx_msg);
	//else if (!(skip & 7))
	//	lora_call();
	else
		lora_tx(tx_msg);
	tx_ptr = 0;
}

