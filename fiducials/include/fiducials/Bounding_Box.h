// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(BOUNDING_BOX_H_INCLUDED)
#define BOUNDING_BOX_H_INCLUDED 1

#include "Double.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef struct Bounding_Box__Struct *Bounding_Box;

struct Bounding_Box__Struct {
  Double maximum_x;
  Double minimum_x;
  Double maximum_y;
  Double minimum_y;
};

extern Bounding_Box Bounding_Box__new(void);
extern void Bounding_Box__reset(Bounding_Box bounding_box);
extern void Bounding_Box__free(Bounding_Box bounding_box);
extern void Bounding_Box__update(Bounding_Box bounding_box, Double x, Double y);

#ifdef __cplusplus
}
#endif
#endif // !defined(BOUNDING_BOX_H_INCLUDED)
