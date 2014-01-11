// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(CRC_H_INCLUDED)
#define CRC_H_INCLUDED 1

#include "Unsigned.h"

#ifdef __cplusplus
extern "C" {
#endif
extern Unsigned CRC__compute(Unsigned *buffer, Unsigned size);
#ifdef __cplusplus
}
#endif

#endif // !defined(CRC_H_INCLUDED)

