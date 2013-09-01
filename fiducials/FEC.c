/*
 * This code appears to be mostly copied from with some additional minor
 * code provided by Michael Thompson.
 *
 *  http://www.piclist.com/techref/method/error/rs-gp-pk-uoh-199609/rs_c.htm
 *
 * This code claims to be written by Phil Karns in 1996.  The code at that
 * site unfortunately has no license in it.
 *
 * The most current version of Phil's library (fec-3.0.1) is at:
 *
 *   http://www.ka9q.net/code/fec/
 *
 * The fec-3.0.1 library (August 7, 2007) is distributed under LPGL.
 * 
 * By the time I got the the code, it appears that Michael Thompson had
 * attached a GNU Public License to it all.  Given that Phil Karn is
 * distributing his latest code under the GNU LGPL, my assumption is
 * that his older code is actually distributed under LGPL.  So I have
 * removed the GPL and I have reattached the LGPL back onto this code
 * (see end of this file.)
 *
 * Wayne C. Gramlich
 * September 17, 2010
 */

/*
    This code implements general purpose Reed-Solomon forward error correction
    algorithm for buffers of 3-bit to 8-bit symbols.  It is a based on code by
    Phil Karn, which is in turn a rewrite of code by Robert Morelos-Zaragoza
    and Hari Thirumoorthy, which was in turn based on an earlier program by
    Simon Rockliff.  Further information on Phil Karn's implementation can be
    found at:

    http://www.piclist.com/techref/method/error/rs-gp-pk-uoh-199609/index.htm

    This code is woefully undocumented and uncommented.  Hopefully someone
    who can understand the math within the Reed Solomon algorithm can comment
    and optimize this code further.
*/

#include <stdlib.h>
#include <alloca.h>

#include "FEC.h"
#include "Logical.h"
#include "Unsigned.h"

// No legal value in index form represents zero, so
// we use nn as a special value for this purpose.
#define ALPHA_ZERO  ((uintGF) self->nn)

// Alpha exponent for the first root of the generator polynomial.
#define ALPHA_ROOT  1

// Helper macro to find minimum value.
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

// Primitive polynomial tables.  See Lin & Costello, Error Control Coding
// Appendix A  and  Lee & Messerschmitt, Digital Communication p. 453.

// 1 + x + x^3
rvUint8 pp_3[4] = { 1, 1, 0, 1 };

// 1 + x + x^4
rvUint8 pp_4[5] = { 1, 1, 0, 0, 1 };

// 1 + x^2 + x^5
rvUint8 pp_5[6] = { 1, 0, 1, 0, 0, 1 };

// 1 + x + x^6
rvUint8 pp_6[7] = { 1, 1, 0, 0, 0, 0, 1 };

// 1 + x^3 + x^7
rvUint8 pp_7[8] = { 1, 0, 0, 1, 0, 0, 0, 1 };

// 1+x^2+x^3+x^4+x^8
rvUint8 pp_8[9] = { 1, 0, 1, 1, 1, 0, 0, 0, 1 };

#if 0
// The following primitive polynomial tables are currently unused, but
// copied here for future reference.

// 1+x^4+x^9
rvUint8 pp_9[10] = { 1, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

// 1+x^3+x^10
rvUint8 pp_10[11] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

// 1+x^2+x^11
rvUint8 pp_11[12] = { 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

// 1+x+x^4+x^6+x^12
rvUint8 pp_12[13] = { 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1 };

// 1+x+x^3+x^4+x^13
rvUint8 pp_13[14] = { 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

// 1+x+x^6+x^10+x^14
rvUint8 pp_14[15] = { 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

// 1+x+x^15
rvUint8 pp_15[16] = { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

// 1+x+x^3+x^12+x^16
rvUint8 pp_16[17] = { 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1 };
#endif

// Helper macro to compute x mod nn.
#define modnn(x) ((x) % self->nn)

static void rvFec_InitGaloisField(rvFec* self, rvUint8 *pp)
// Initialize GF(2**mm) table from the irreducible polynomial p(X) in pp[0]..pp[MM - 1].
//
// The following tables are created:
//
//   index form to polynomial form alphaTo[] contains j=alpha**i;
//   polynomial form to index form indexOf[j=alpha**i] = i
//
// The representation of the elements of GF(2**mm) is either in index form,
// where the number is the power of the primitive element alpha, which is
// convenient for multiplication (add the powers modulo 2**mm-1) or in
// polynomial form, where the bits represent the coefficients of the
// polynomial representation of the number, which is the most convenient form
// for addition.  The two forms are swapped between via lookup tables.
// This leads to fairly messy looking expressions, but unfortunately, there
// is no easy alternative when working with Galois arithmetic.
{
    rvUint16 i;
    rvUint16 mask;

    // Local references so code is easier on the eyes.
    uintGF mm = (uintGF) self->mm;
    uintGF nn = (uintGF) self->nn;
    uintGF *alphaTo = self->alphaTo;
    uintGF *indexOf = self->indexOf;

    mask = 1;
    alphaTo[mm] = 0;
    for (i = 0; i < mm; i++) {
        alphaTo[i] = (uintGF) mask;
        indexOf[alphaTo[i]] = (uintGF) i;
        if (pp[i] != 0) {
	    alphaTo[mm] ^= (uintGF) mask;
	}
        mask <<= 1;
    }
    indexOf[alphaTo[mm]] = mm;
    mask >>= 1;
    for (i = mm + 1; i < nn; i++) {
 	if (alphaTo[i - 1] >= mask) {
            alphaTo[i] = alphaTo[mm] ^ ((alphaTo[i - 1] ^ mask) << 1);
        } else {
            alphaTo[i] = alphaTo[i - 1] << 1;
	}
        indexOf[alphaTo[i]] = (uintGF) i;
    }
    indexOf[0] = ALPHA_ZERO;
    alphaTo[nn] = 0;
}


static void
rvFec_InitPolynomial(
  rvFec* self)
{
    // Initialize the generator polynomial table.
    rvInt16 i;
    rvInt16 j;

    // Local references so code is easier on the eyes.
    uintGF *gg = self->gg;
    uintGF *alphaTo = self->alphaTo;
    uintGF *indexOf = self->indexOf;

    gg[0] = alphaTo[ALPHA_ROOT];
    gg[1] = 1;
    for (i = 2; i <= self->paritySize; i++) {
        gg[i] = 1;
        for (j = i - 1; j > 0; --j) {
	    if (gg[j] != 0) {
                gg[j] = gg[j - 1] ^
		  alphaTo[modnn((indexOf[gg[j]]) + ALPHA_ROOT + i - 1)];
            } else {
                gg[j] = gg[j - 1];
	    }
        }
        gg[0] = alphaTo[modnn((indexOf[gg[0]]) + ALPHA_ROOT + i - 1)];
    }

    // Convert gg[] to index form for quicker encoding.
    for (i = 0; i <= self->paritySize; i++) {
	gg[i] = indexOf[gg[i]];
    }
}


rvFec* rvFec_New(rvInt16 symbolSize, rvInt16 dataSize, rvInt16 paritySize)
// Initialize the encoder object.
{
    rvInt16 mm;
    rvInt16 nn;
    rvInt16 kk;
    rvInt16 zeroSize;
    rvInt16 blockSize;
    rvUint8* pp = NULL;
    uintGF* gg = NULL;
    uintGF* alphaTo = NULL;
    uintGF* indexOf = NULL;
    rvFec* self = NULL;

    // Set the Reed Solomon properties.
    mm = symbolSize;
    nn = ((1 << mm) - 1);
    kk = nn - paritySize;

    // The block size is the combined data and parity size.
    blockSize = dataSize + paritySize;

    // The zero size is the size of the zero padded data.
    zeroSize = nn - blockSize;

    // Sanity check block and data size.
    if ((blockSize < 1) || (blockSize > nn)) return NULL;
    if ((dataSize < 1) || (dataSize >= blockSize)) return NULL;

    // Point to primative polynomial table.
    if (symbolSize == 3) pp = pp_3;
    else if (symbolSize == 4) pp = pp_4;
    else if (symbolSize == 5) pp = pp_5;
    else if (symbolSize == 6) pp = pp_6;
    else if (symbolSize == 7) pp = pp_7;
    else if (symbolSize == 8) pp = pp_8;
    else return NULL;

    // Allocate a new ecc object and tables.
    self = (rvFec *) malloc(sizeof(rvFec));
    gg = (uintGF*) malloc(sizeof(uintGF) * (paritySize + 1));
    alphaTo = (uintGF*) malloc(sizeof(uintGF) * (nn + 1));
    indexOf = (uintGF*) malloc(sizeof(uintGF) * (nn + 1));

    // Did we allocate the object.
    if ((self != NULL) && (gg != NULL) &&
      (alphaTo != NULL) && (indexOf != NULL)) {
        // Set the Reed Solomon properties.
        self->mm = mm;
        self->nn = nn;
        self->kk = kk;

        // Set the buffer properties.
        self->zeroSize = zeroSize;
        self->dataSize = dataSize;
        self->paritySize = paritySize;
        self->blockSize = blockSize;

        // Set the allocated tables.
        self->gg = gg;
        self->alphaTo = alphaTo;
        self->indexOf = indexOf;

        // Initialize Galois Field table
        rvFec_InitGaloisField(self, pp);

        // Initialize generator polynomial.
        rvFec_InitPolynomial(self);
    } else {
        // Clean up.
        if (self) free(self);
        if (gg) free(gg);
        if (alphaTo) free(alphaTo);
        if (indexOf) free(indexOf);
    }

    return self;
}


rvInt16 rvFec_Parity(rvFec* self, rvUint8* dataBuffer, rvUint8* parityBuffer)
// Encode parity for the data buffer.  Returns 1 if success or 0 if
// unable to encode.  Encoding is done by using a feedback shift register
// with appropriate connections specified by the elements of gg[], which
// was generated by the ecc_generate_galois_field() function.
{
    rvInt16 i;
    rvInt16 j;
    uintGF feedback;

    // Local references so code is easier on the eyes.
    uintGF *gg = self->gg;
    uintGF *alphaTo = self->alphaTo;
    uintGF *indexOf = self->indexOf;

    // Zero the parity data.
    for (i = 0; i < self->paritySize; ++i) parityBuffer[i] = 0;

    // Loop over the data portion of the encode buffer.
    for (i = self->zeroSize + self->dataSize; i >= 0; --i) {
        // Pad data with zero to account for shortened buffer when
	// determining feedback.
        feedback =
	  indexOf[((i >= self->zeroSize) ? dataBuffer[i - self->zeroSize] : 0) ^
	  parityBuffer[self->paritySize - 1]];

        // Is the feedback term non-zero?
        if (feedback != ALPHA_ZERO) {
            for (j = self->paritySize - 1; j > 0; --j) {
		if (gg[j] != ALPHA_ZERO) {
                    parityBuffer[j] =
		      parityBuffer[j - 1] ^ alphaTo[modnn(gg[j] + feedback)];
                } else {
                    parityBuffer[j] = parityBuffer[j - 1];
		}
            }
            parityBuffer[0] = alphaTo[modnn(gg[0] + feedback)];
        } else {
            for (j = self->paritySize - 1; j > 0; --j) {
                parityBuffer[j] = parityBuffer[j - 1];
            }
            parityBuffer[0] = 0;
        }
    }

    return 1;
}


rvInt16 rvFec_Correct(rvFec* self, rvUint8* blockBuffer)
// Correct the block buffer.  Returns 1 if success or 0 if unable to correct.
{
    rvInt16 i;
    rvInt16 j;
    rvInt16 r;
    rvInt16 el;
    rvInt16 count;
    rvInt16 syn_error;
    rvInt16 deg_omega;
    rvInt16 deg_lambda;
    rvInt16 *loc;
    uintGF q;
    uintGF *b;
    uintGF *t;
    uintGF *reg;
    uintGF *recd;
    uintGF *root;
    uintGF *omega;
    uintGF *lambda;
    uintGF *syndromes;

    // Local references so code is easier on the eyes.
    rvInt16 nn = self->nn;
    uintGF *gg = self->gg;
    uintGF *alphaTo = self->alphaTo;
    uintGF *indexOf = self->indexOf;

    // Allocate the recieve buffer off the stack.
    recd = (uintGF*) alloca(sizeof(uintGF) * nn);

    // Copy and convert the encoded buffer from polynomial form to index form.
    for (i = nn - 1; i >= 0; --i) {
        // Pad data with zero to account for shortened buffer.
        recd[i] =
	  indexOf[(i >= self->zeroSize) ? blockBuffer[i - self->zeroSize] : 0];
    }

    // Allocate the syndromes buffer off the stack.
    syndromes = (uintGF*) alloca(sizeof(uintGF) * (self->paritySize + 1));

    // Initialize the syndrome error flag.
    syn_error = 0;

    // Form and store each syndrome by evaluating recd(x) at roots of g(x)
    // namely @**(ALPHA_ROOT+i), i = 0, ... ,(nn-KK-1)
    for (i = 1; i <= self->paritySize; i++) {
        uintGF syndrome = 0;

        // Loop over each data byte.
        for (j = 0; j < nn; j++) {
            // All non-zero receive values added to the syndrome.
            if (recd[j] != ALPHA_ZERO) {
                syndrome ^= alphaTo[modnn(recd[j] + (ALPHA_ROOT + i - 1) * j)];
            }
        }

        // Set flag if non-zero syndrome.
        syn_error |= syndrome;

        // Store syndrome in index form.
        syndromes[i] = indexOf[syndrome];
    }

    if (!syn_error) {
        // If all syndromes are zero there are no errors to correct.
        return 1;
    }

    // Allocate the lambda buffer off the stack.
    lambda = (uintGF*) alloca(sizeof(uintGF) * (self->paritySize + 1));

    // Clear the lampda buffer.
    for (i = 1; i < self->paritySize + 1; ++i) lambda[i] = 0;

    lambda[0] = 1;

    b = (uintGF*) alloca(sizeof(uintGF) * (self->paritySize + 1));
    t = (uintGF*) alloca(sizeof(uintGF) * (self->paritySize + 1));

    for (i = 0; i < self->paritySize + 1; i++) {
	b[i] = indexOf[lambda[i]];
    }

    // Begin Berlekamp-Massey algorithm to determine error locator polynomial.
    el = 0;

    for (r = 1; r <= self->paritySize; ++r) {
        uintGF discrepancy = 0;

        // Compute discrepancy at the r-th step in poly-form.
        for (i = 0; i < r; i++) {
            if ((lambda[i] != 0) && (syndromes[r - i] != ALPHA_ZERO)) {
                discrepancy ^=
		  alphaTo[modnn(indexOf[lambda[i]] + syndromes[r - i])];
            }
        }

        // Convert the discrepancy to index form.
        discrepancy = indexOf[discrepancy];

        // Is the discrepancy zero?
        if (discrepancy == ALPHA_ZERO) {
            // Move all array elements up.
	    for (i = self->paritySize - 1; i >= 0; --i) {
		b[i + 1] = b[i];
	    }

            // The first array element is zero.
            b[0] = ALPHA_ZERO;
        } else {
            // 7 lines below: T(x) <-- lambda(x) - discrepancy*x*b(x)
            t[0] = lambda[0];

            for (i = 0 ; i < self->paritySize; i++) {
		if (b[i] != ALPHA_ZERO) {
                    t[i+1] = lambda[i+1] ^ alphaTo[modnn(discrepancy + b[i])];
                } else {
                    t[i+1] = lambda[i+1];
		}
            }

            if ((2 * el) <= (r - 1)) {
                el = r - el;

                // 2 lines below: B(x) <-- inv(discrepancy) * lambda(x)
                for (i = 0; i <= self->paritySize; i++) {
                    b[i] = (lambda[i] == 0) ? ALPHA_ZERO :
		      modnn(indexOf[lambda[i]] - discrepancy + nn);
                }
            } else {
                // Move all array elements up.
		for (i = self->paritySize - 1; i >= 0; --i) {
		    b[i + 1] = b[i];
		}

                // The first array element is zero.
                b[0] = ALPHA_ZERO;
            }

            for (i = 0; i < self->paritySize + 1; ++i) lambda[i] = t[i];
        }
    }

    deg_lambda = 0;

    // Convert lambda to index form and compute deg(lambda(x)).
    for (i = 0; i < self->paritySize + 1; ++i) {
        lambda[i] = indexOf[lambda[i]];

        if (lambda[i] != ALPHA_ZERO) {
	    deg_lambda = i;
	}
    }

    loc = (rvInt16*) alloca(sizeof(rvInt16) * self->paritySize);
    reg = (uintGF*) alloca(sizeof(uintGF) * (self->paritySize + 1));
    root = (uintGF*) alloca(sizeof(uintGF) * self->paritySize);

    // Find roots of the error locator polynomial by Chien search.
    for (i = 1; i < self->paritySize + 1; ++i) {
	reg[i] = lambda[i];
    }

    // Number of roots of lambda(x).
    count = 0;

    for (i = 1; i <= nn; i++) {
        q = 1;

        for (j = deg_lambda; j > 0; j--) {
            if (reg[j] != ALPHA_ZERO) {
                reg[j] = modnn(reg[j] + j);
                q ^= alphaTo[reg[j]];
            }
        }

        if (!q) {
            // Store root (index-form) and error location number.
            root[count] = (uintGF) i;
            loc[count] = nn - i;
            count++;
        }
    }

#ifdef DEBUG
    printf("\n Final error positions:\t");
    for (i = 0; i < count; i++) {
        printf("%d ", loc[i]);
    }
    printf("\n");
#endif

    // If deg(lambda) unequal to number of roots then uncorrectable error
    // detected:
    if (deg_lambda != count) {
        return 0;
    }

    deg_omega = 0;

    omega = (uintGF*) alloca(sizeof(uintGF) * (self->paritySize + 1));

    // Compute error evaluator poly omega(x) =
    // s(x) * lambda(x) (modulo x**(NN-KK))
    // in index form. Also find deg(omega).
    for (i = 0; i < self->paritySize; ++i) {
        uintGF tmp = 0;

        for (j = (deg_lambda < i) ? deg_lambda : i; j >= 0; --j) {
            if ((syndromes[i + 1 - j] != ALPHA_ZERO) &&
	      (lambda[j] != ALPHA_ZERO)) {
                tmp ^= alphaTo[modnn(syndromes[i + 1 - j] + lambda[j])];
	    }
        }

        if (tmp != 0) {
	    deg_omega = i;
	}

        omega[i] = indexOf[tmp];
    }

    omega[self->paritySize] = ALPHA_ZERO;

    // Compute error values in poly-form.
    for (j = count - 1; j >= 0; j--) {
        uintGF den;
        uintGF num1;
        uintGF num2;

        num1 = 0;

        for (i = deg_omega; i >= 0; i--) {
	    if (omega[i] != ALPHA_ZERO) {
		num1 ^= alphaTo[modnn(omega[i] + i * root[j])];
	    }
        }

        num2 = alphaTo[modnn(root[j] * (ALPHA_ROOT - 1) + nn)];

        den = 0;

        // lambda[i+1] for i even is the formal derivative lambda_pr
	// of lambda[i]:
        for (i = MIN(deg_lambda, self->paritySize - 1) & ~1; i >= 0; i -=2) {
	    if (lambda[i + 1] != ALPHA_ZERO) {
		den ^= alphaTo[modnn(lambda[i+1] + i * root[j])];
	    }
        }

        if (den == 0) {
#ifdef DEBUG
            printf("\n ERROR: denominator = 0\n");
#endif
            return 0;
        }

        // Apply error to data.
        if (num1 != 0) {
            // We should never need to correct in the zero padding.
            if (loc[j] < self->zeroSize) {
                return 0;
            }

            // Account for shortened data.
            if ((loc[j] >= self->zeroSize) &&
	      (loc[j] < self->zeroSize + self->dataSize)) {
                blockBuffer[loc[j] - self->zeroSize] ^=
		  alphaTo[modnn(indexOf[num1] + indexOf[num2] +
		  nn - indexOf[den])];
            }
        }
    }

    return 1;
}

Logical
FEC__correct(
    FEC fec,
    Unsigned *data,
    Unsigned size)
{
    unsigned char data_bytes[8];
    Unsigned index;
    Logical result;

    /* Load {data} into {data_bytes}: */
    for (index = 0; index < size; index++) {
	data_bytes[index] = data[index] & 0xff;
    }    

    /* Correct any errors: */
    result = rvFec_Correct(fec, data_bytes);

    /* Load {data_bytes} into {data}: */
    for (index = 0; index < size; index++) {
	data[index] = data_bytes[index];
    }    

    return result;
}

void
FEC__parity(
  FEC fec,
  Unsigned *data,
  Unsigned size)
{
    unsigned char data_bytes[8];
    Unsigned index;

    /* Load {data} into {data_bytes}: */
    for (index = 0; index < 8; index++) {
	data_bytes[index] = data[index] & 0xff;
    }    

    /* Compute the parity information: */
    rvFec_Parity(fec, data_bytes, data_bytes + 4);

    /* Load {data_bytes} into {data}: */
    for (index = 0; index < 8; index++) {
	data[index] = data_bytes[index];
    }    
}

FEC
FEC__create(
  Unsigned symbol_size,
  Unsigned data_size,
  Unsigned parity_size)
{
    return rvFec_New(symbol_size, data_size, parity_size);
}

/*

		  GNU LESSER GENERAL PUBLIC LICENSE
		       Version 2.1, February 1999

 Copyright (C) 1991, 1999 Free Software Foundation, Inc.
     59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.

[This is the first released version of the Lesser GPL.  It also counts
 as the successor of the GNU Library Public License, version 2, hence
 the version number 2.1.]

			    Preamble

  The licenses for most software are designed to take away your
freedom to share and change it.  By contrast, the GNU General Public
Licenses are intended to guarantee your freedom to share and change
free software--to make sure the software is free for all its users.

  This license, the Lesser General Public License, applies to some
specially designated software packages--typically libraries--of the
Free Software Foundation and other authors who decide to use it.  You
can use it too, but we suggest you first think carefully about whether
this license or the ordinary General Public License is the better
strategy to use in any particular case, based on the explanations below.

  When we speak of free software, we are referring to freedom of use,
not price.  Our General Public Licenses are designed to make sure that
you have the freedom to distribute copies of free software (and charge
for this service if you wish); that you receive source code or can get
it if you want it; that you can change the software and use pieces of
it in new free programs; and that you are informed that you can do
these things.

  To protect your rights, we need to make restrictions that forbid
distributors to deny you these rights or to ask you to surrender these
rights.  These restrictions translate to certain responsibilities for
you if you distribute copies of the library or if you modify it.

  For example, if you distribute copies of the library, whether gratis
or for a fee, you must give the recipients all the rights that we gave
you.  You must make sure that they, too, receive or can get the source
code.  If you link other code with the library, you must provide
complete object files to the recipients, so that they can relink them
with the library after making changes to the library and recompiling
it.  And you must show them these terms so they know their rights.

  We protect your rights with a two-step method: (1) we copyright the
library, and (2) we offer you this license, which gives you legal
permission to copy, distribute and/or modify the library.

  To protect each distributor, we want to make it very clear that
there is no warranty for the free library.  Also, if the library is
modified by someone else and passed on, the recipients should know
that what they have is not the original version, so that the original
author's reputation will not be affected by problems that might be
introduced by others.

  Finally, software patents pose a constant threat to the existence of
any free program.  We wish to make sure that a company cannot
effectively restrict the users of a free program by obtaining a
restrictive license from a patent holder.  Therefore, we insist that
any patent license obtained for a version of the library must be
consistent with the full freedom of use specified in this license.

  Most GNU software, including some libraries, is covered by the
ordinary GNU General Public License.  This license, the GNU Lesser
General Public License, applies to certain designated libraries, and
is quite different from the ordinary General Public License.  We use
this license for certain libraries in order to permit linking those
libraries into non-free programs.

  When a program is linked with a library, whether statically or using
a shared library, the combination of the two is legally speaking a
combined work, a derivative of the original library.  The ordinary
General Public License therefore permits such linking only if the
entire combination fits its criteria of freedom.  The Lesser General
Public License permits more lax criteria for linking other code with
the library.

  We call this license the "Lesser" General Public License because it
does Less to protect the user's freedom than the ordinary General
Public License.  It also provides other free software developers Less
of an advantage over competing non-free programs.  These disadvantages
are the reason we use the ordinary General Public License for many
libraries.  However, the Lesser license provides advantages in certain
special circumstances.

  For example, on rare occasions, there may be a special need to
encourage the widest possible use of a certain library, so that it becomes
a de-facto standard.  To achieve this, non-free programs must be
allowed to use the library.  A more frequent case is that a free
library does the same job as widely used non-free libraries.  In this
case, there is little to gain by limiting the free library to free
software only, so we use the Lesser General Public License.

  In other cases, permission to use a particular library in non-free
programs enables a greater number of people to use a large body of
free software.  For example, permission to use the GNU C Library in
non-free programs enables many more people to use the whole GNU
operating system, as well as its variant, the GNU/Linux operating
system.

  Although the Lesser General Public License is Less protective of the
users' freedom, it does ensure that the user of a program that is
linked with the Library has the freedom and the wherewithal to run
that program using a modified version of the Library.

  The precise terms and conditions for copying, distribution and
modification follow.  Pay close attention to the difference between a
"work based on the library" and a "work that uses the library".  The
former contains code derived from the library, whereas the latter must
be combined with the library in order to run.

		  GNU LESSER GENERAL PUBLIC LICENSE
   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

  0. This License Agreement applies to any software library or other
program which contains a notice placed by the copyright holder or
other authorized party saying it may be distributed under the terms of
this Lesser General Public License (also called "this License").
Each licensee is addressed as "you".

  A "library" means a collection of software functions and/or data
prepared so as to be conveniently linked with application programs
(which use some of those functions and data) to form executables.

  The "Library", below, refers to any such software library or work
which has been distributed under these terms.  A "work based on the
Library" means either the Library or any derivative work under
copyright law: that is to say, a work containing the Library or a
portion of it, either verbatim or with modifications and/or translated
straightforwardly into another language.  (Hereinafter, translation is
included without limitation in the term "modification".)

  "Source code" for a work means the preferred form of the work for
making modifications to it.  For a library, complete source code means
all the source code for all modules it contains, plus any associated
interface definition files, plus the scripts used to control compilation
and installation of the library.

  Activities other than copying, distribution and modification are not
covered by this License; they are outside its scope.  The act of
running a program using the Library is not restricted, and output from
such a program is covered only if its contents constitute a work based
on the Library (independent of the use of the Library in a tool for
writing it).  Whether that is true depends on what the Library does
and what the program that uses the Library does.
  
  1. You may copy and distribute verbatim copies of the Library's
complete source code as you receive it, in any medium, provided that
you conspicuously and appropriately publish on each copy an
appropriate copyright notice and disclaimer of warranty; keep intact
all the notices that refer to this License and to the absence of any
warranty; and distribute a copy of this License along with the
Library.

  You may charge a fee for the physical act of transferring a copy,
and you may at your option offer warranty protection in exchange for a
fee.

  2. You may modify your copy or copies of the Library or any portion
of it, thus forming a work based on the Library, and copy and
distribute such modifications or work under the terms of Section 1
above, provided that you also meet all of these conditions:

    a) The modified work must itself be a software library.

    b) You must cause the files modified to carry prominent notices
    stating that you changed the files and the date of any change.

    c) You must cause the whole of the work to be licensed at no
    charge to all third parties under the terms of this License.

    d) If a facility in the modified Library refers to a function or a
    table of data to be supplied by an application program that uses
    the facility, other than as an argument passed when the facility
    is invoked, then you must make a good faith effort to ensure that,
    in the event an application does not supply such function or
    table, the facility still operates, and performs whatever part of
    its purpose remains meaningful.

    (For example, a function in a library to compute square roots has
    a purpose that is entirely well-defined independent of the
    application.  Therefore, Subsection 2d requires that any
    application-supplied function or table used by this function must
    be optional: if the application does not supply it, the square
    root function must still compute square roots.)

These requirements apply to the modified work as a whole.  If
identifiable sections of that work are not derived from the Library,
and can be reasonably considered independent and separate works in
themselves, then this License, and its terms, do not apply to those
sections when you distribute them as separate works.  But when you
distribute the same sections as part of a whole which is a work based
on the Library, the distribution of the whole must be on the terms of
this License, whose permissions for other licensees extend to the
entire whole, and thus to each and every part regardless of who wrote
it.

Thus, it is not the intent of this section to claim rights or contest
your rights to work written entirely by you; rather, the intent is to
exercise the right to control the distribution of derivative or
collective works based on the Library.

In addition, mere aggregation of another work not based on the Library
with the Library (or with a work based on the Library) on a volume of
a storage or distribution medium does not bring the other work under
the scope of this License.

  3. You may opt to apply the terms of the ordinary GNU General Public
License instead of this License to a given copy of the Library.  To do
this, you must alter all the notices that refer to this License, so
that they refer to the ordinary GNU General Public License, version 2,
instead of to this License.  (If a newer version than version 2 of the
ordinary GNU General Public License has appeared, then you can specify
that version instead if you wish.)  Do not make any other change in
these notices.

  Once this change is made in a given copy, it is irreversible for
that copy, so the ordinary GNU General Public License applies to all
subsequent copies and derivative works made from that copy.

  This option is useful when you wish to copy part of the code of
the Library into a program that is not a library.

  4. You may copy and distribute the Library (or a portion or
derivative of it, under Section 2) in object code or executable form
under the terms of Sections 1 and 2 above provided that you accompany
it with the complete corresponding machine-readable source code, which
must be distributed under the terms of Sections 1 and 2 above on a
medium customarily used for software interchange.

  If distribution of object code is made by offering access to copy
from a designated place, then offering equivalent access to copy the
source code from the same place satisfies the requirement to
distribute the source code, even though third parties are not
compelled to copy the source along with the object code.

  5. A program that contains no derivative of any portion of the
Library, but is designed to work with the Library by being compiled or
linked with it, is called a "work that uses the Library".  Such a
work, in isolation, is not a derivative work of the Library, and
therefore falls outside the scope of this License.

  However, linking a "work that uses the Library" with the Library
creates an executable that is a derivative of the Library (because it
contains portions of the Library), rather than a "work that uses the
library".  The executable is therefore covered by this License.
Section 6 states terms for distribution of such executables.

  When a "work that uses the Library" uses material from a header file
that is part of the Library, the object code for the work may be a
derivative work of the Library even though the source code is not.
Whether this is true is especially significant if the work can be
linked without the Library, or if the work is itself a library.  The
threshold for this to be true is not precisely defined by law.

  If such an object file uses only numerical parameters, data
structure layouts and accessors, and small macros and small inline
functions (ten lines or less in length), then the use of the object
file is unrestricted, regardless of whether it is legally a derivative
work.  (Executables containing this object code plus portions of the
Library will still fall under Section 6.)

  Otherwise, if the work is a derivative of the Library, you may
distribute the object code for the work under the terms of Section 6.
Any executables containing that work also fall under Section 6,
whether or not they are linked directly with the Library itself.

  6. As an exception to the Sections above, you may also combine or
link a "work that uses the Library" with the Library to produce a
work containing portions of the Library, and distribute that work
under terms of your choice, provided that the terms permit
modification of the work for the customer's own use and reverse
engineering for debugging such modifications.

  You must give prominent notice with each copy of the work that the
Library is used in it and that the Library and its use are covered by
this License.  You must supply a copy of this License.  If the work
during execution displays copyright notices, you must include the
copyright notice for the Library among them, as well as a reference
directing the user to the copy of this License.  Also, you must do one
of these things:

    a) Accompany the work with the complete corresponding
    machine-readable source code for the Library including whatever
    changes were used in the work (which must be distributed under
    Sections 1 and 2 above); and, if the work is an executable linked
    with the Library, with the complete machine-readable "work that
    uses the Library", as object code and/or source code, so that the
    user can modify the Library and then relink to produce a modified
    executable containing the modified Library.  (It is understood
    that the user who changes the contents of definitions files in the
    Library will not necessarily be able to recompile the application
    to use the modified definitions.)

    b) Use a suitable shared library mechanism for linking with the
    Library.  A suitable mechanism is one that (1) uses at run time a
    copy of the library already present on the user's computer system,
    rather than copying library functions into the executable, and (2)
    will operate properly with a modified version of the library, if
    the user installs one, as long as the modified version is
    interface-compatible with the version that the work was made with.

    c) Accompany the work with a written offer, valid for at
    least three years, to give the same user the materials
    specified in Subsection 6a, above, for a charge no more
    than the cost of performing this distribution.

    d) If distribution of the work is made by offering access to copy
    from a designated place, offer equivalent access to copy the above
    specified materials from the same place.

    e) Verify that the user has already received a copy of these
    materials or that you have already sent this user a copy.

  For an executable, the required form of the "work that uses the
Library" must include any data and utility programs needed for
reproducing the executable from it.  However, as a special exception,
the materials to be distributed need not include anything that is
normally distributed (in either source or binary form) with the major
components (compiler, kernel, and so on) of the operating system on
which the executable runs, unless that component itself accompanies
the executable.

  It may happen that this requirement contradicts the license
restrictions of other proprietary libraries that do not normally
accompany the operating system.  Such a contradiction means you cannot
use both them and the Library together in an executable that you
distribute.

  7. You may place library facilities that are a work based on the
Library side-by-side in a single library together with other library
facilities not covered by this License, and distribute such a combined
library, provided that the separate distribution of the work based on
the Library and of the other library facilities is otherwise
permitted, and provided that you do these two things:

    a) Accompany the combined library with a copy of the same work
    based on the Library, uncombined with any other library
    facilities.  This must be distributed under the terms of the
    Sections above.

    b) Give prominent notice with the combined library of the fact
    that part of it is a work based on the Library, and explaining
    where to find the accompanying uncombined form of the same work.

  8. You may not copy, modify, sublicense, link with, or distribute
the Library except as expressly provided under this License.  Any
attempt otherwise to copy, modify, sublicense, link with, or
distribute the Library is void, and will automatically terminate your
rights under this License.  However, parties who have received copies,
or rights, from you under this License will not have their licenses
terminated so long as such parties remain in full compliance.

  9. You are not required to accept this License, since you have not
signed it.  However, nothing else grants you permission to modify or
distribute the Library or its derivative works.  These actions are
prohibited by law if you do not accept this License.  Therefore, by
modifying or distributing the Library (or any work based on the
Library), you indicate your acceptance of this License to do so, and
all its terms and conditions for copying, distributing or modifying
the Library or works based on it.

  10. Each time you redistribute the Library (or any work based on the
Library), the recipient automatically receives a license from the
original licensor to copy, distribute, link with or modify the Library
subject to these terms and conditions.  You may not impose any further
restrictions on the recipients' exercise of the rights granted herein.
You are not responsible for enforcing compliance by third parties with
this License.

  11. If, as a consequence of a court judgment or allegation of patent
infringement or for any other reason (not limited to patent issues),
conditions are imposed on you (whether by court order, agreement or
otherwise) that contradict the conditions of this License, they do not
excuse you from the conditions of this License.  If you cannot
distribute so as to satisfy simultaneously your obligations under this
License and any other pertinent obligations, then as a consequence you
may not distribute the Library at all.  For example, if a patent
license would not permit royalty-free redistribution of the Library by
all those who receive copies directly or indirectly through you, then
the only way you could satisfy both it and this License would be to
refrain entirely from distribution of the Library.

If any portion of this section is held invalid or unenforceable under any
particular circumstance, the balance of the section is intended to apply,
and the section as a whole is intended to apply in other circumstances.

It is not the purpose of this section to induce you to infringe any
patents or other property right claims or to contest validity of any
such claims; this section has the sole purpose of protecting the
integrity of the free software distribution system which is
implemented by public license practices.  Many people have made
generous contributions to the wide range of software distributed
through that system in reliance on consistent application of that
system; it is up to the author/donor to decide if he or she is willing
to distribute software through any other system and a licensee cannot
impose that choice.

This section is intended to make thoroughly clear what is believed to
be a consequence of the rest of this License.

  12. If the distribution and/or use of the Library is restricted in
certain countries either by patents or by copyrighted interfaces, the
original copyright holder who places the Library under this License may add
an explicit geographical distribution limitation excluding those countries,
so that distribution is permitted only in or among countries not thus
excluded.  In such case, this License incorporates the limitation as if
written in the body of this License.

  13. The Free Software Foundation may publish revised and/or new
versions of the Lesser General Public License from time to time.
Such new versions will be similar in spirit to the present version,
but may differ in detail to address new problems or concerns.

Each version is given a distinguishing version number.  If the Library
specifies a version number of this License which applies to it and
"any later version", you have the option of following the terms and
conditions either of that version or of any later version published by
the Free Software Foundation.  If the Library does not specify a
license version number, you may choose any version ever published by
the Free Software Foundation.

  14. If you wish to incorporate parts of the Library into other free
programs whose distribution conditions are incompatible with these,
write to the author to ask for permission.  For software which is
copyrighted by the Free Software Foundation, write to the Free
Software Foundation; we sometimes make exceptions for this.  Our
decision will be guided by the two goals of preserving the free status
of all derivatives of our free software and of promoting the sharing
and reuse of software generally.

			    NO WARRANTY

  15. BECAUSE THE LIBRARY IS LICENSED FREE OF CHARGE, THERE IS NO
WARRANTY FOR THE LIBRARY, TO THE EXTENT PERMITTED BY APPLICABLE LAW.
EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR
OTHER PARTIES PROVIDE THE LIBRARY "AS IS" WITHOUT WARRANTY OF ANY
KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE
LIBRARY IS WITH YOU.  SHOULD THE LIBRARY PROVE DEFECTIVE, YOU ASSUME
THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.

  16. IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY MODIFY
AND/OR REDISTRIBUTE THE LIBRARY AS PERMITTED ABOVE, BE LIABLE TO YOU
FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL, INCIDENTAL OR
CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR INABILITY TO USE THE
LIBRARY (INCLUDING BUT NOT LIMITED TO LOSS OF DATA OR DATA BEING
RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU OR THIRD PARTIES OR A
FAILURE OF THE LIBRARY TO OPERATE WITH ANY OTHER SOFTWARE), EVEN IF
SUCH HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
DAMAGES.

		     END OF TERMS AND CONDITIONS

           How to Apply These Terms to Your New Libraries

  If you develop a new library, and you want it to be of the greatest
possible use to the public, we recommend making it free software that
everyone can redistribute and change.  You can do so by permitting
redistribution under these terms (or, alternatively, under the terms of the
ordinary General Public License).

  To apply these terms, attach the following notices to the library.  It is
safest to attach them to the start of each source file to most effectively
convey the exclusion of warranty; and each file should have at least the
"copyright" line and a pointer to where the full notice is found.

    <one line to give the library's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Also add information on how to contact you by electronic and paper mail.

You should also get your employer (if you work as a programmer) or your
school, if any, to sign a "copyright disclaimer" for the library, if
necessary.  Here is a sample; alter the names:

  Yoyodyne, Inc., hereby disclaims all copyright interest in the
  library `Frob' (a library for tweaking knobs) written by James Random Hacker.

  <signature of Ty Coon>, 1 April 1990
  Ty Coon, President of Vice

That's all there is to it!

*/
