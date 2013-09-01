#if !defined(FEC_C_H_INCLUDED)
#define FEC_C_H_INCLUDED 1

/*
    Copyright (C) 2010, Michael P. Thompson

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 2 as 
    specified in the README.txt file or as published by the Free Software 
    Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

    $Id: rvFec.h 28 2010-03-09 23:49:39Z mike $
*/

// Basic 8, 16, and 32 bit integer types.
typedef char                rvInt8;
typedef short               rvInt16;
typedef unsigned char       rvUint8;
typedef unsigned short      rvUint16;

// A boolean type to match the C++ definition of "bool".
#ifndef __cplusplus
typedef char bool;

#define false ((bool)0)
#define true  ((bool)1)
#endif

#ifdef __cplusplus
extern "C" {
#endif

// This defines the type used to store an element in the various
// Galois Field tables.  Make sure this is something larger than
// a rvUint8 if anything larger than GF(256) is used.
typedef rvUint8 uintGF;

// Forward error correcting types.
typedef struct _rvFec rvFec;

typedef struct _rvFec *FEC;

#include "Logical.h"
#include "Unsigned.h"

// Forward error correcting structures.
struct _rvFec
{
    // Buffer properties.
    rvInt16 zeroSize;               // Zero padding portion of buffer.
    rvInt16 dataSize;               // Data portion of buffer.
    rvInt16 paritySize;         // Parity portion of buffer.
    rvInt16 blockSize;          // Data and parity portion of buffer.

    // Reed Solomon properties.
    rvInt16 mm;                 // The code symbol size in bits.
    rvInt16 nn;                 // The block size in symbols, which is always (2**mm - 1).
    rvInt16 kk;                 // The number of data symbols per block, kk < nn.

    // Reed Solomon tables.
    uintGF *gg;                 // Generator polynomial g(x)
    uintGF *alphaTo;            // Index to polynomial form conversion table.
    uintGF *indexOf;            // Polynomial to index form conversion table.
};

// Forward error correcting methods.
rvFec* rvFec_New(rvInt16 symbolSize, rvInt16 dataSize, rvInt16 paritySize);
rvInt16 rvFec_Parity(rvFec* self, rvUint8* dataBuffer, rvUint8* parityBuffer);
rvInt16 rvFec_Correct(rvFec* self, rvUint8* blockBuffer);

#ifdef __cplusplus
} // "C"
#endif

// External declarations:

Logical FEC__correct(FEC fec, Unsigned *data, Unsigned size);
void FEC__parity(FEC fec, Unsigned *data, Unsigned size);
FEC FEC__create(Unsigned symbol_size, Unsigned data_size, Unsigned parity_size);

#endif // !defined(FEC_C_H_INCLUDED)
