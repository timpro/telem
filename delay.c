// (c) John Greb, MIT License.

#include "common.h"

// 1 second delay between data fetches
// internal clock is 9% adrift 
#define BAUD_MS 100

// The basic delay(ms) is used to blink the LED for System Errors
// and for short i2c delays. It is not time critical

// inaccurate busy wait
void delay(short length_ms)
{
    short i;
    while (length_ms-- > 0){
	for (i=0; i<1000; i++)
		asm("nop");
    }
}

int lpt_count = 0;
int get_count(void)
{ return lpt_count; }

// Timer interrupt handler
volatile int lpt_flag = 0;
void LPTimer_IRQHandler()
{
    LPTMR0_CSR |=  LPTMR_CSR_TCF_MASK;   // clear the LPT timer compare flag
    LPTMR0_CMR = BAUD_MS;
    LPTMR0_CSR = ( LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK);
    lpt_flag++;
}

// Halt CPU between timer interrupts for maximum powersave
void lpdelay_init(void)
{
    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;  // Make sure clock is enabled
    LPTMR0_CSR = 0;                     // Reset LPTMR settings
    LPTMR0_CMR = BAUD_MS;             // Set compare value
    enable_irq(INT_LPTimer);
    LPTMR0_PSR = LPTMR_PSR_PCS(1) | LPTMR_PSR_PBYP_MASK;
    LPTMR0_CSR = LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK;
}

// Halt CPU between timer interrupts for maximum powersave
void lpdelay(void)
{
    // timer is already started and may already have looped.
    while( !lpt_flag ) {
     __asm volatile("dsb");
     __asm volatile("wfi"); /* wait for interrupt */
     __asm volatile("isb");
    }
    lpt_count += lpt_flag;
    lpt_flag = 0; // clear flag until next interrupt
} 
