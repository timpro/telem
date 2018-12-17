/*
 * Functions for MFSK encoding / decoding
 * Copyright (C) 2014  Richard Meadows <richardeoin>
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

#include <string.h>

#include "fwht.h"
#include "contestia.h"

static const uint64_t scrambler_olivia		= 0xE257E6D0291574ECLL;
static const uint64_t scrambler_contestia	= 0xEDB88320LL;

/**
 * USEFUL RESOURES =============================================================
 *
 * http://en.wikipedia.org/wiki/Olivia_MFSK
 * http://www.arrl.org/olivia
 *
 * Also, although without images:
 * http://web.archive.org/web/20070927210543/http://homepage.sunrise.ch/mysunrise/jalocha/fht_coding.htm
 *
 * Useful guide to the differences between olivia and contestia
 * (ultimately we'd like to support both):
 * http://f1ult.free.fr/DIGIMODES/MULTIPSK/contestia_rttym_en.htm
 *
 * Also dl-fldigi's source might be helpful:
 * https://github.com/jamescoxon/dl-fldigi/blob/master/src/include/jalocha/pj_mfsk.h
 */

void mfsk_encode_block(char* block, int8_t* tones,
                       uint8_t symbols_per_block,	/* The number of on-the-air symbols to transmit for each block */
                       uint8_t bits_per_symbol,		/* The number of bits encoded in each on-the-air symbol */
                       uint64_t scrambler,		/* Scrambler sequence */
                       uint8_t scrambling_shift)	/* Scrambler shift */
{
  uint8_t character;
  uint32_t symbol, mask_index;
  int8_t fwht_vector[symbols_per_block];
  memset(tones, 0, symbols_per_block * sizeof(int8_t));

  /**
   * There is one bit in the symbol for each character in the
   * block. Iterate over each character.
   */
  for (character = 0; character < bits_per_symbol; character++) {

    /* Mask off unuseds bits in the character */
    block[character] &= ((symbols_per_block * 2) - 1);

    /* Set a deviation in the input vector */
    memset(fwht_vector, 0, symbols_per_block * sizeof(int8_t));
    if (block[character] < symbols_per_block) {
      fwht_vector[block[character] - 0]                  = 1;	/* +ve */
    } else {
      fwht_vector[block[character] - symbols_per_block] = -1;	/* -ve */
    }

    /* Perform an in-place Inverse Fast Walsh-Hadamard Transform */
    ifwht(fwht_vector, symbols_per_block);

    mask_index = character * scrambling_shift;
    /* Iterate over each symbol in the output block */
    for (symbol = 0; symbol < symbols_per_block; symbol++, mask_index++) {
      mask_index %= symbols_per_block;

      /* If this bit in the FWHT is significant */
      if ((scrambler & (1LL << mask_index)) ?
          (fwht_vector[symbol] > 0) :	/* Scrambled:     +ve is significant */
          (fwht_vector[symbol] < 0)) {	/* Not Scrambled: -ve is significant */

        /* Find the bit index to set */
        uint8_t bit_index = (character + symbol) % bits_per_symbol;

        /* Set this bit */
        tones[symbol] |= (1 << bit_index);
      }
    }
  }
}


/**
 * This function encodes a single block of Olivia MFSK
 *
 * It takes a buffer of ASCII-encoded text and returns an array of
 * tones to transmit.
 */
void olivia_encode_block(char* block, int8_t* tones)
{
  size_t bits_per_symbol = 1; /* That is, there are 2^1=2 tones */

  mfsk_encode_block(block, tones, 64, bits_per_symbol, scrambler_olivia, 13);
}
/**
 * This function encodes a single block of Contestia MFSK
 *
 * It takes a single character of ASCII-encoded text and returns an array of
 * tones to transmit.
 */
void contestia_encode_block(char* block, int8_t* tones)
{
  size_t bits_per_symbol = 1; /* That is, there are 2^1=2 tones */

  {
    char character = *block;

    /* lowercase => UPPERCASE */
    if (character >= 'a' && character <= 'z') {
      character += 'A' - 'a';
    }

    /* Convert to contestia character set */
    if (character >= '!' && character <= 'Z') {	/* Printables...   */
      character -= 32;
    } else if (character == ' ') {		/* Space           */
      character = 59;
    } else if (character == '\r') {		/* Carriage Return */
      character = 60;
    } else if (character == '\n') {		/* Line Feed       */
      character = 60;				/*   no \n, use \r */
    } else if (character == 8) {		/* Backspace       */
      character = 61;
    } else if (character == 0) {		/* Null            */
      character = 0;
    } else {					/* ????????????    */
      character = '?' - 32;
    }

    *block = character;
  }


  mfsk_encode_block(block, tones, 32, bits_per_symbol, scrambler_contestia, 5);
}


/*
 *  Code to pack bits into bytes for Tx
 */

/* Olivia is twice as long as Contestia */
int8_t contestia_tones[OLIVIA_NUMBER_OF_TONES];

/* pack a block of 32 tones into 4 chars for transmission */
uint8_t contestia_convert(char* returnstring, uint16_t length) {
	uint8_t index, outdex, shift;
	int8_t	binarycode, greycode;

	for (index = 0; index < length; index++) {
		binarycode = contestia_tones[index];
		greycode = binarycode & 0x01; // (binarycode >> 1) ^ binarycode;

		// pack 1 bit into 8bit output string
		// Output is little endian
		outdex = index >> 3;
		shift = index & 0x07;
		returnstring[outdex] &= ~(1 << shift);	
		returnstring[outdex] |= greycode << shift;
	}

	return (length + 7) >> 3;
}

/* translate 1 char into 32 tones */
uint8_t contestia_block(char* block, char* outstring) {
	contestia_encode_block(block, contestia_tones);
	return contestia_convert( outstring, 32);
}

/* translate 1 char into 64 tones */
uint8_t olivia_block(char* block, char* outstring) {
	olivia_encode_block(block, contestia_tones);
	return  contestia_convert( outstring, 64);
}

/* This function parses a string and converts to contestia characters */
void contestiaize(char* string, uint16_t length) {
  uint16_t i;
  for (i = 0; i < length; i++) {

    /* lowercase => UPPERCASE */
    if (string[i] >= 'a' && string[i] <= 'z') {
      string[i] += 'A' - 'a';
    }

    if ((string[i] < '!' || string[i] > 'Z') && /* Not Printable */
        (string[i] != ' ')  && (string[i] != '\r') &&
        (string[i] != '\n') && (string[i] != 8) &&
        (string[i] != 0)) {
      string[i] = '?'; /* Comes out as question mark */
    }
  }
}
