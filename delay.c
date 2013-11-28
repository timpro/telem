// (c) John Greb, MIT License.

#include <freedom.h>
#include "common.h"

// The basic delay(ms) is used to blink the LED for System Errors
// and for short i2c delays. It is not time critical
// DominoEx needs regular interrupts from the low power timer

// inaccurate busy wait
void delay(unsigned int length_ms)
{
    short i;
    while (--length_ms){
	for (i=0; i<1000; i++)
		asm("nop");
    }
}

// Timer interrupt handler
volatile short lpt_flag = 0;
void LPTimer_IRQHandler()
{
    LPTMR0_CSR |=  LPTMR_CSR_TCF_MASK;   // clear the LPT timer compare flag
    LPTMR0_CMR = 64;
    LPTMR0_CSR = ( LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK);
    lpt_flag = 1;
}

// Halt CPU between timer interrupts for maximum powersave
void lpdelay_init(void)
{
    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;  // Make sure clock is enabled
    LPTMR0_CSR = 0;                     // Reset LPTMR settings
    LPTMR0_CMR = 64;             // Set compare value (64 ms for Dom16)
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
    lpt_flag = 0; // clear flag until next interrupt
}
