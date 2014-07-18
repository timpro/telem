HAB Branch
==========

Long distance telemetry, transmitted by FSK 434MHz USB FM:

1) Ublox GPS on Uart 0 at 9600 baud.
2) NTX2B radio driven with 12 bit DAC
3) Battery Volts read with 12 bit ADC

The onboard regulator can be replaced with a 2.0V version for reduced power consumption. The external Crystal is disabled and clock speeds are divided down to 5.2MHz CPU, 1.3MHz bus. With DominoEx8 transmission the CPU only needs to be awake for a few cycles every second, waking on interrupts from the Uart or 8Hz timer.

Designed for 3 AAA batteries giving 24mA for 24Hrs at 24C: NTX2B needs 10 mA at over 3V; a 2V LDO regulator needs over 3V and wastes 4mA; the Ublox MAX-7C needs over 1.8V and 7mA  in full powersaving mode with a good signal, much more current when hunting satellites.


EVK Branch
==========

Adding support for Xtrinsic EVK sensors Pack:

1) MMA8491Q Accelerometer.
2) MPL3115A Barometer.
3) MAG3110  Magnetometer.

Bare Metal Arm
==============

This is a "bare metal" runtime for the 
[Freescale Freedom FRDM-KL25Z](http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=FRDM-KL25Z) 
ARM development board ($13). It builds with the GCC ARM toolchain, with no other external dependencies. 

Quick start on Linux or Mac OS X:
* Clone the repo: `git clone https://github.com/payne92/bare-metal-arm.git`
* Grab and unpack GCC ARM toolchain: `cd bare-metal-arm; make gcc-arm`
* `make`

This will create a `demo.srec` image file to flash onto the development board.  (If you're using
the standard bootloader, plug the SDA USB port to a host computer.  On Linux, type `make deploy`.  On other systems,
copy the .SREC file to the FRDM-KL25Z volume.)  

If everything is working, the RGB LEB will flash a few times and then be steady green.  You can access the USB 
SDA serial port (at 115,200 baud) and see the accelerometer and touch input status.

References:
* [Freescale Freedom FRDM-KL25Z](http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=FRDM-KL25Z)
  * Where to buy: [Newark](http://www.newark.com/jsp/search/productdetail.jsp?SKU=28W5033&CMP=KNC-GPLA&mckv=|pcrid|27090073701|plid|),
[Digi-Key](http://www.digikey.com/product-detail/en/FRDM-KL25Z/FRDM-KL25Z-ND/3529594?WT.mc_id=PLA_3529594)
* [KL25 Sub-Family Reference Manual](http://cache.freescale.com/files/32bit/doc/ref_manual/KL25P80M48SF0RM.pdf)
* [GCC ARM toolchain](https://launchpad.net/gcc-arm-embedded)
* [Newlib C library](http://sourceware.org/newlib/)

Why do this?
------------

Most vendor provided toolchains are daunting. They're large and complicated because they support a wide
range of processors, libraries, and commerical C compilers.  These tools are usually laden with 
complex configuration tools, lots of source macros and #ifdef statements, and code that's been ported
down several generations of processors.

In contrast, this project is a small (<1,000 lines without USB) simple, and clean bare metal framework that builds 
from the command line.  It has no external library or tool dependencies, only supports GCC, and has minimal use of
assembly.

Walkthrough
-----------

The interrupt vectors and reset code are in `_startup.c`.  The CPU comes out of reset in `_reset_init()` which:
* Copies initialized constant values from flash ROM to RAM
* Configures the main clock (48Mhz)
* Jumps to `_start()` in the Newlib C library

After the C library is done initializing, it invokes `main()` (implemented in `demo.c`).
