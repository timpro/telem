//
// uart.c -- Delay functions
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
//

#include <freedom.h>
#include "common.h"

// delay(ms) -- Spin wait delay (in ms)
//              Note:  uses low power timer (LPTMR)
void delay(unsigned int length_ms)
{
    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;  // Make sure clock is enabled
    LPTMR0_CSR = 0;                     // Reset LPTMR settings         
    LPTMR0_CMR = length_ms;             // Set compare value (in ms)

    // Use 1kHz LPO with no prescaler
    LPTMR0_PSR = LPTMR_PSR_PCS(1) | LPTMR_PSR_PBYP_MASK;
    
    // Start the timer
    LPTMR0_CSR = LPTMR_CSR_TEN_MASK;
    while (!(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)) { ;
    }
    LPTMR0_CSR = 0;                     // Turn off timer
}


// Timer interrupt handler
volatile lpt_flag = 0;
void LPTimer_IRQHandler()
{
    LPTMR0_CSR |=  LPTMR_CSR_TCF_MASK;   // write 1 to TCF to clear the LPT timer compare flag
    LPTMR0_CSR = ( LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK | LPTMR_CSR_TCF_MASK  );
    lpt_flag = 1;
}

// Halt CPU until timer interrupt for maximum powersave
void lpdelay(unsigned int length_ms)
{
    SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;  // Make sure clock is enabled
    LPTMR0_CSR = 0;                     // Reset LPTMR settings
    LPTMR0_CMR = length_ms;             // Set compare value (in ms)
    enable_irq(INT_LPTimer);

    LPTMR0_PSR = LPTMR_PSR_PCS(1) | LPTMR_PSR_PBYP_MASK;

    lpt_flag = 0;
    // Start the timer
    LPTMR0_CSR = LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK;
    while( !lpt_flag ) {
     __asm volatile("dsb");
     __asm volatile("wfi"); /* wait for interrupt */
     __asm volatile("isb");
    }

    LPTMR0_CSR = 0;                     // Turn off timer
}
