/*
 Linear Technology DC2100A Demonstration Board.
 DC2100A Application File for Controlling the LTC3300-1 Battery Balancers through the LTC6804-2 Battery Monitor on the DC2100A PCB.

 @verbatim
 todo
 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 750 $
 $Date: 2014-09-17 19:27:28 -0400 (Wed, 17 Sep 2014) $

 Copyright (c) 2013, Linear Technology Corp.(LTC)
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


Library Adopted and Modified for use by Joseph Ojo (March 2020)

*/

#ifndef __TYPEDEFS_H__
#define __TYPEDEFS_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// datatypes defined by CCS
#define int8                char
#define int16               short // Keep an eye out for this. The size might change based on the system it's running on
#define int32               int
#define float32             float
#define int1                char
#define FALSE               0
#define TRUE                1
#define BYTE                int8
#define BOOLEAN             int1

// macros for retrieving parts of variables in an endian safe way
#define UPPER_NIBBLE(x)     ((x >> 4) & 0x0f)
#define LOWER_NIBBLE(x)     (x & 0x0f)
#define UPPER_BYTE(x)       ((x >> 8) & 0xff)
#define LOWER_BYTE(x)       (x & 0xff)
#define UPPER_WORD(x)       ((x >> 16) & 0xffff)
#define LOWER_WORD(x)       (x & 0xffff)
#define LAST_BYTE(x)        (&((int8*)x)[sizeof(x) - 1])

// macros for common operations
#define MIN(x, y)           (x < y ? x : y)
// Note - CCS compiler can not use MAX macro as it conflicts with max keyword for #task directive.
//        Case sensitivity is turned off in CCS by default, but turning it on causes code not to compile for other reasons.
#define MAX2(x, y)          (x > y ? x : y)
#define TOGGLE(x)           (x = (x ? 0 : 1))
#define MASK(size, shift)   (((1LL << (size)) - 1) << (shift))
#define BITVAL(value, bit)  ((value & MASK(1,bit)) >> bit)
#define STRINGIZE2(X)       #X
#define STRINGIZE(X)        STRINGIZE2(X)

// macros to return the number of bits needed to store a number
#define NBITS2(n)           ((n&2)?1:0)
#define NBITS4(n)           ((n&(0xC))?(2+NBITS2(n>>2)):(NBITS2(n)))
#define NBITS8(n)           ((n&0xF0)?(4+NBITS4(n>>4)):(NBITS4(n)))
#define NBITS16(n)          ((n&0xFF00)?(8+NBITS8(n>>8)):(NBITS8(n)))
#define NBITS32(n)          ((n&0xFFFF0000)?(16+NBITS16(n>>16)):(NBITS16(n)))
#define NBITS(n)            (n==0?0:NBITS32(n)+1)

// This macro divides a unsigned number by an unsigned number with rounding.
#define UNSIGNED_DIVIDE_BY_UNSIGNED_WITH_ROUND(numerator, denominator)      \
(                                                                           \
    ((numerator) + ((denominator + 1) >> 1)) / (denominator)                \
)                                                                           \

// This macro divides a unsigned number by an unsigned number through a shift with rounding.
// Note - this macro seems to work for most situations, but for some reason it produces garbage if your numerator variable is smaller than
//        the variable where you assign the result.  You would expect the macro to finish working with the small numerator variable and then
//        sign extend when it casts it to a larger variable, but for some reason it does not.  Since there is little reason to assign the result
//        to a variable larger than the numerator (given that the right shift can only make the result smaller than the numerator), this is not
//        much of a limitation.
#define UNSIGNED_RIGHT_SHIFT_WITH_ROUND(numerator, shift)                   \
(                                                                           \
    ((numerator) + (1LL << ((shift) - 1))) >> (shift)                       \
)                                                                           \


// This macro divides a signed number by an unsigned number with rounding.
#define SIGNED_DIVIDE_BY_UNSIGNED_WITH_ROUND(numerator, denominator)        \
(                                                                           \
    (                                                                       \
        (numerator) + ((denominator + 1) >> 1) * ((numerator) < 0 ? -1 : 1) \
    ) / (denominator)                                                       \
)                                                                           \

// This macro divides a signed number by an unsigned number through a shift with rounding. (i.e. round( numerator / 2^(shift) )  )
// Note - this macro seems to work for most situations, but for some reason it produces garbage if your numerator variable is smaller than
//        the variable where you assign the result.  You would expect the macro to finish working with the small numerator variable and then
//        sign extend when it casts it to a larger variable, but for some reason it does not.  Since there is little reason to assign the result
//        to a variable larger than the numerator (given that the right shift can only make the result smaller than the numerator), this is not
//        much of a limitation.
#define SIGNED_RIGHT_SHIFT_WITH_ROUND(numerator, shift)                                                     \
(                                                                                                           \
    (                                                                                                       \
        ((numerator) + (1LL << ((shift) - 1)) + ((numerator) < 0 ? - 1 : 0) ) >> (shift)                    \
    ) |                                                                                                     \
    (                                                                                                       \
        ((numerator) < 0 ? MASK(shift, sizeof((numerator))*BITS_PER_BYTE - shift) : 0 )                     \
    )                                                                                                       \
)                                                                                                           \

// constants that improve code readability
#define BITS_PER_NIBBLE     4
#define BITS_PER_BYTE       8
#define ASCII_PER_BYTE      2
#define UNICODE_PER_ASCII   2
#define MA_PER_A            1000
#define MS_PER_S            1000
#define US_PER_MS           1000
#define US_PER_S            1000000
#define MV_PER_V            1000
#define UV_PER_V            1000000
#define PERCENT_MAX         100

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#endif
