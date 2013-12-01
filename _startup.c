//
// _startup.c -- Reset initialization
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
//

#include "freedom.h"
#include "common.h"
#include <stdio.h>
#include <string.h>

void _reset_init(void)    __attribute__((naked, aligned(8)));
extern void _start(void);                   // newlib C lib initialization

// ----------------------------------------------------------------------------------
// Flash configuration field (loaded into flash memory at 0x400)
//
//      Note:  RESET_PIN_CFG is set to enable external RESET signal
//
__attribute__ ((section (".cfmconfig"))) const uint8_t _cfm[0x10] = {
    0xFF,  /* NV_BACKKEY3: KEY=0xFF */
    0xFF,  /* NV_BACKKEY2: KEY=0xFF */
    0xFF,  /* NV_BACKKEY1: KEY=0xFF */
    0xFF,  /* NV_BACKKEY0: KEY=0xFF */
    0xFF,  /* NV_BACKKEY7: KEY=0xFF */
    0xFF,  /* NV_BACKKEY6: KEY=0xFF */
    0xFF,  /* NV_BACKKEY5: KEY=0xFF */
    0xFF,  /* NV_BACKKEY4: KEY=0xFF */
    0xFF,  /* NV_FPROT3: PROT=0xFF */
    0xFF,  /* NV_FPROT2: PROT=0xFF */
    0xFF,  /* NV_FPROT1: PROT=0xFF */
    0xFF,  /* NV_FPROT0: PROT=0xFF */
    0x7E,  /* NV_FSEC: KEYEN=1,MEEN=3,FSLACC=3,SEC=2 */
    0xFF,  /* NV_FOPT: ??=1,??=1,FAST_INIT=1,LPBOOT1=1,RESET_PIN_CFG=1,
                        NMI_DIS=1,EZPORT_DIS=1,LPBOOT0=1 */
    0xFF,
    0xFF
  };

// ----------------------------------------------------------------------------------
//
// Initialize the system clocks with default 21Mhz core clock speed
//  with divide-by-four for cpu = 5.25 MHz
//  extra divide-by-four for the bus = 1.3 MHz
//  Uart0 runs from the undivided FLL clock -- #define CORE_CLOCK 21000000
//  RTC runs from external crystal, which is disabled to save power. 

static void init_clocks(void)
{   
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV4(3);
    /* MCG_C1: CLKS=0,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = (uint8_t)6; // default is 4, turns on 1KHz clock
    MCG_C2 = (uint8_t)0; // 1 or 0 => 4MHz clock or 32K clock
	// C3 adjusts clock speed: preset at factory (21MHz)
    MCG_C4 = (uint8_t)(MCG_C4 & ~(0xE0)); // default, 20-24Mhz clock
    // OSC0_CR =(uint8_t)0x80; //turn ON external oscillator.

    while(!(MCG_S & MCG_S_IREFST_MASK)) {
    }
    while(MCG_S & 0x0C) {
    }
}

// Blink an LED based on a pattern bitmask
void fault(uint32_t pattern)
{
    for(;;) {
        RGB_LED(pattern & 1 ? 100 : 0, 0, 0);           // Set RED led based on LSB
        pattern = (pattern >> 1) | (pattern << 31);     // Rotate
        delay(100);
    }
}

// ----------------------------------------------------------------------------------
//  Default interrupt handler
void __attribute__((interrupt("IRQ"))) Default_Handler()
{
    fault(0b11111110);          // Blink LED and halt
}

// The register frame pushed onto the stack during exceptions
typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    void *pc;
    uint32_t psr;
} hw_stackframe_t; 

// Handle hard faults:  print debugging information and halt
static void __attribute__((naked)) HardFault_Handler(void) 
{
    // Set up arguments and invoke _HardFault_Handler()
    asm("mov    r0, lr\n"                   // Arg 0
        "mrs  r1, psp\n"                    // Arg 1
        "mrs  r2, msp\n"                    // Arg 2
        "b  _HardFault_Handler\n");
}

void __attribute__((naked)) _HardFault_Handler(uint32_t lr, void *psp, void *msp)
{
    hw_stackframe_t *frame;
    
    // Find the active stack pointer (MSP or PSP)
    if(lr & 0x4)
        frame = psp;
    else
        frame = msp;
        
    fiprintf(stderr, "** HARD FAULT **\r\n   pc=%p\r\n  msp=%p\r\n  psp=%p\r\n", 
                    frame->pc, msp, psp);
                    
    fault(0b1111111000);            // Blink LED and halt
}


/* Weak definitions of handlers point to Default_Handler if not implemented */
void NMI_Handler()          __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler()          __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler()       __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA0_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA1_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA2_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void DMA3_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void MCM_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void FTFL_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void PMC_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void LLW_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void SPI0_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void UART2_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void ADC0_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void CMP0_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void FTM0_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void FTM1_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void FTM2_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void RTC_Seconds_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
void PIT_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void USBOTG_IRQHandler()    __attribute__ ((weak, alias("Default_Handler")));
void DAC0_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void TSI0_IRQHandler()      __attribute__ ((weak, alias("Default_Handler")));
void MCG_IRQHandler()       __attribute__ ((weak, alias("Default_Handler")));
void LPTimer_IRQHandler()   __attribute__ ((weak, alias("Default_Handler")));
void PORTA_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));
void PORTD_IRQHandler()     __attribute__ ((weak, alias("Default_Handler")));

// ----------------------------------------------------------------------------------
// Interrupt vector table (loaded into flash memory at 0x000)
//
void (* const InterruptVector[])() __attribute__ ((section(".isr_vector"))) = {
    (void(*)(void)) __StackTop,                     // Initial stack pointer
    _reset_init,                                    // Reset handler
    NMI_Handler,
    HardFault_Handler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SVC_Handler,
    0,
    0,
    PendSV_Handler,
    SysTick_Handler,

    /* Interrupts */
    DMA0_IRQHandler, /* DMA Channel 0 Transfer Complete and Error */
    DMA1_IRQHandler, /* DMA Channel 1 Transfer Complete and Error */
    DMA2_IRQHandler, /* DMA Channel 2 Transfer Complete and Error */
    DMA3_IRQHandler, /* DMA Channel 3 Transfer Complete and Error */
    MCM_IRQHandler, /* Normal Interrupt */
    FTFL_IRQHandler, /* FTFL Interrupt */
    PMC_IRQHandler, /* PMC Interrupt */
    LLW_IRQHandler, /* Low Leakage Wake-up */
    I2C0_IRQHandler, /* I2C0 interrupt */
    I2C1_IRQHandler, /* I2C1 interrupt */
    SPI0_IRQHandler, /* SPI0 Interrupt */
    SPI1_IRQHandler, /* SPI1 Interrupt */
    UART0_IRQHandler, /* UART0 Status and Error interrupt */
    UART1_IRQHandler, /* UART1 Status and Error interrupt */
    UART2_IRQHandler, /* UART2 Status and Error interrupt */
    ADC0_IRQHandler, /* ADC0 interrupt */
    CMP0_IRQHandler, /* CMP0 interrupt */
    FTM0_IRQHandler, /* FTM0 fault, overflow and channels interrupt */
    FTM1_IRQHandler, /* FTM1 fault, overflow and channels interrupt */
    FTM2_IRQHandler, /* FTM2 fault, overflow and channels interrupt */
    RTC_Alarm_IRQHandler, /* RTC Alarm interrupt */
    RTC_Seconds_IRQHandler, /* RTC Seconds interrupt */
    PIT_IRQHandler, /* PIT timer all channels interrupt */
    Default_Handler, /* Reserved interrupt 39/23 */
    USBOTG_IRQHandler, /* USB interrupt */
    DAC0_IRQHandler, /* DAC0 interrupt */
    TSI0_IRQHandler, /* TSI0 Interrupt */
    MCG_IRQHandler, /* MCG Interrupt */
    LPTimer_IRQHandler, /* LPTimer interrupt */
    Default_Handler, /* Reserved interrupt 45/29 */
    PORTA_IRQHandler, /* Port A interrupt */
    PORTD_IRQHandler /* Port D interrupt */
};

// ----------------------------------------------------------------------------------
//
// init_led_io() -- Initialize I/O pins for on-board RGB LED (PWM)
//
static void init_led_io(void)
{
    // Turn on clock gating to PortB module (red and green LEDs) and 
    // PortD module (blue LED)  
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

    SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM2_MASK;
    SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1);

    PORTB_PCR18 = PORT_PCR_MUX(3);  // TPM2_CH0 enable on PTB18 (red)
    PORTB_PCR19 = PORT_PCR_MUX(3);  // TPM2_CH1 enable on PTB19 (green)
    PORTD_PCR1  = PORT_PCR_MUX(4);  // TPM0_CH1 enable on PTD1  (blue)

    RGB_LED(0,0,0);                 // Off
    
    TPM0_MOD  = 99;
    TPM0_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
    TPM2_MOD  = 99;
    TPM2_C0SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
    TPM2_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;

    TPM2_SC   = TPM_SC_CMOD(1) | TPM_SC_PS(0);     /* Edge Aligned PWM running from BUSCLK / 1 */
    TPM0_SC   = TPM_SC_CMOD(1) | TPM_SC_PS(0);     /* Edge Aligned PWM running from BUSCLK / 1 */
}

// ----------------------------------------------------------------------------------
//
// _reset_init() -- Reset entry point.  
//
//      The CPU reset vector points here.  Initialize the CPU, and jump
//      to the C runtime start, which will eventually invoke main()
//
void _reset_init(void)
{
    SIM_COPC = 0;                       // Disable the watchdog timer   
    SCB_VTOR = (uint32_t)InterruptVector;

    // Copy values to initialize data segment
    uint32_t *fr = __etext;
    uint32_t *to = __data_start__;
    unsigned int len = __data_end__ - __data_start__;
    while(len--)
        *to++ = *fr++;

    init_clocks();
    init_led_io();
    _start();                           // Goto C lib startup
    fault(FAULT_FAST_BLINK);            // ...should never get here.
}
