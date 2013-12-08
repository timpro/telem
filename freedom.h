//
// freedom.h -- Definitions for Freescale Freedom development board
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
//

#include "MKL25Z4.h"                    // CPU definitions

static inline void RED_LED(int red) {
    if (1&red) {
	GPIOB_PCOR |= 1 << 18; // low is on
    } else {
	GPIOB_PSOR |= 1 << 18; // high is off
    }
}
