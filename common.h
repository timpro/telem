
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
//

#include "MKL25Z4.h"

// core clock for Uart is half cpu speed
#define CORE_CLOCK 21000000

// Memory locations defined by the linker
extern uint32_t __heap_start[];
extern uint32_t __StackTop[];
extern uint32_t __data_start__[], __data_end__[];
extern uint32_t __bss_start__[], __bss_end__[];
extern uint32_t __etext[];                // End of code/flash

// From hal_i2c.c
void hal_i2c_init(I2C_MemMapPtr p);
void hal_i2c_deinit(I2C_MemMapPtr p);

// From math.c
unsigned short magnitude(short x, short y, short z);
short sine(short angle);
short cosine(short angle);
short findArctan(short x, short y, short z);
short upness(short x, short y, short z);
short findArcsin( short scalar, unsigned short mag );

// From uart.c
void UART0_IRQHandler() __attribute__((interrupt("IRQ")));
int uart_write(char *p, int len);
int uart_write_err(char *p, int len);
int uart_read(char *p, int len);
void uart_init(int baud_rate);

// From delay.c
void delay(unsigned int ms);

// From accel.c
void accel_init(void);
void accel_read(void);
int16_t accel_x(void);
int16_t accel_y(void);
int16_t accel_z(void);

// From baro3115.c
void  baro_init(void);
short baro_alt(void);
unsigned short get_pressure(void);
short baro_temp(void);

// From mag3110.c
void  mag_init(void);
short mag_compass(short ax, short ay, short az);

// From ublox.c
void ublox_init(void);
short ublox_update(void);
void gps_output(short force, short compass, short pressure,
		short temperature, short battery);

// Passing data from uart.c
short ublox_sats(void);
long ublox_time(void);
long ublox_lon(void);
long ublox_lat(void);
long ublox_alt(void);

// From domino.c
void domino_tx(char);

// From _startup.c
void fault(uint32_t pattern);
#define FAULT_FAST_BLINK 	(0b10101010101010101010101010101010)
#define FAULT_MEDIUM_BLINK 	(0b11110000111100001111000011110000)
#define FAULT_SLOW_BLINK 	(0b11111111000000001111111100000000)

static inline void enable_irq(int n) {
    NVIC_ICPR |= 1 << (n - 16);
    NVIC_ISER |= 1 << (n - 16);			
}
// TODO:  IRQ disable

static inline void __enable_irq(void)	{ asm volatile ("cpsie i"); }
static inline void __disable_irq(void)  { asm volatile ("cpsid i"); }

// ring.c
typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t size;
    volatile uint8_t data[];
} RingBuffer;

void buf_reset(RingBuffer *buf, int size);
int buf_len(const RingBuffer *buf);
int buf_isfull(const RingBuffer *buf);
int buf_isempty(const RingBuffer *buf);
uint8_t buf_get_byte(RingBuffer *buf);
void buf_put_byte(RingBuffer *buf, uint8_t val);

// tests.c
void tests(void);
