typedef struct radio_lora_settings_s
{
	uint8_t spreading_factor;
	uint8_t bandwidth;
 	uint8_t coding_rate;
 	uint8_t implicit_mode;
 	uint8_t crc_en;
 	uint8_t low_datarate;
} radio_lora_settings_t;


typedef struct radio_fsk_settings_s
{
	uint16_t freq_dev;
 	uint16_t bitrate;
 	uint8_t enable_sync;
 	uint8_t enable_crc;
 	uint16_t preamble_size;
} radio_fsk_settings_t;

typedef enum {BAUD_50, BAUD_300, BAUD_600} rtty_baud_t;


void radio_init(void);
uint8_t radio_read_version(void);
uint8_t radio_read_single_reg(uint8_t reg);
void radio_write_single_reg(uint8_t reg, uint8_t data);
void radio_write_burst_reg(uint8_t reg, uint8_t *data, uint16_t len);
void radio_tx_packet(uint8_t *data, uint16_t len);
void radio_write_lora_config(radio_lora_settings_t *s);
void radio_high_power(void);
void radio_set_frequency(uint32_t);
void radio_read_burst_reg(uint8_t reg, uint8_t *buff, uint16_t len);
void radio_set_continuous_rx(void);
int16_t radio_check_read_rx_packet(uint16_t max_len, uint8_t *buff, uint8_t check_crc);
void radio_lna_max(void);
void radio_pa_off(void);
uint8_t radio_fsk_poll_fifo_level(void);
uint8_t radio_rtty_poll_buffer_refill();
void radio_start_tx_rtty(char *data, rtty_baud_t baud, uint8_t deviation);
uint8_t rtty_in_progress(void);
uint8_t lora_in_progress(void);
void radio_sleep(void);



#define CODING_4_5 (0x1 << 1)
#define CODING_4_6 (0x2 << 1)
#define CODING_4_7 (0x3 << 1)
#define CODING_4_8 (0x4 << 1)

#define BANDWIDTH_7_8K (0<<4)
#define BANDWIDTH_10_4K (1<<4)
#define BANDWIDTH_15_6K (2<<4)
#define BANDWIDTH_20_8K (3<<4)
#define BANDWIDTH_31_25K (4<<4)
#define BANDWIDTH_41_7K (5<<4)
#define BANDWIDTH_62_5K (6<<4)
#define BANDWIDTH_125K (7<<4)
#define BANDWIDTH_250K (8<<4)
#define BANDWIDTH_500K (9<<4)


#define MODE_SLEEP 0
#define MODE_STNDBY 1
#define MODE_FSTX 2
#define MODE_TX 3
#define MODE_FSRX 4
#define MODE_RX 5
#define MODE_MASK 7

#define POWER_HIGH 0x88

#define FREQ_434_000 7110656
#define FREQ_434_025 7111066
#define FREQ_434_050 7111475
#define FREQ_434_075 7111885
#define FREQ_434_100 7112294

#define FDEV_600 1
#define BITRATE_500 64000

#define IRQ_RX_DONE (1<<6)
#define IRQ_CRC_ERROR (1<<5)



//FSK MODE
#define REG_FIFO 0
#define REG_OP_MODE 0x1
#define REG_BITRATE_MSB 0x2
#define REG_BITRATE_LSB 0x3
#define REG_FDEV_MSB 0x4
#define REG_FDEV_LSB 0x5
#define REG_FRF_MSB 0x6
#define REG_FRF_MID 0x7
#define REG_FRF_LSB 0x8
#define REG_PA_CONFIG 0x9
#define REG_PA_RAMP 0xA
#define REG_OCP 0xb
#define REG_LNA 0xC
#define REG_RX_CONFIG 0xd
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


//LORA MODE

#define REG_FIFO_ADDR_PTR 0xD
#define REG_FIFO_TX_BASE_ADDR 0xE
#define REG_FIFO_RX_BASE_ADDR 0xF
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS_MASK 0x11
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_RX_HEADER_CNT_VALUE_MSB 0x14
#define REG_RX_HEADER_CNT_VALUE_LSB 0x15
#define REG_RX_PACKET_CNT_VALUE_MSB 0x16
#define REG_RX_PACKET_CNT_VALUE_LSB 0x17
#define REG_MODEM_STAT 0x18
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1A
#define REG_RSSI_VALUE_LORA 0x1B
#define REG_HOP_CHANNEL 0x1C
#define REG_MODEM_CONFIG1 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_SYMB_TIMEOUT_LSB 0x1F
#define REG_PREAMBLE_MSB_LORA 0x20
#define REG_PREAMBLE_LSB_LORA 0x21
#define REG_PAYLOAD_LENGTH_LORA 0x22
#define REG_MAX_PAYLOAD_LENGTH 0x23
#define REG_HOP_PERIOD 0x24
#define REG_FIFO_RX_BYTE_ADDR 0x25
#define REG_MODEM_CONFIG3 0x26

#define REG_FEI_MSB_LORA 0x28
#define REG_FEI_MID_LORA 0x29
#define REG_FEI_LSB_LORA 0x2A
#define REG_RES 0x2B
#define REG_RSSI_WIDEBAND 0x2C

#define REG_DETECT_OPTIMIZE 0x31

#define REG_INVERT_IQ 0x33

#define REG_DETECTION_THRESHOLD 0x37


