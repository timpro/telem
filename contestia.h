/*
 * Outputs contestia to the si_trx
 * Copyright (C) 2015  Richard Meadows <richardeoin>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef CONTESTIA_H
#define CONTESTIA_H

#include <stdint.h>

/**
 * Contestia 2/250
 */
#define CONTESTIA_NUMBER_OF_TONES 32
#define CONTESTIA_CHARACTERS_PER_BLOCK 1
#define CONTESTIA_CHANNEL_SPACING 1 /* Corresponds to 122 Hz */
#define CONTESTIA_SYMBOL_RATE 125
#define OLIVIA_NUMBER_OF_TONES 64

uint8_t contestia_block(char* data, char* outstring);
void contestia_encode_block(char* block, int8_t* tones);
void contestiaize(char* string, uint16_t length);
uint8_t olivia_block(char* data, char* outstring);
void olivia_encode_block(char* block, int8_t* tones);

#endif /* CONTESTIA_H */
