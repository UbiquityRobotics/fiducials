// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(BOUNDING_BOX_H_INCLUDED)
#define BOUNDING_BOX_H_INCLUDED 1

typedef struct Bounding_Box__Struct *Bounding_Box;

struct Bounding_Box__Struct {
  double maximum_x;
  double minimum_x;
  double maximum_y;
  double minimum_y;
};

extern void Bounding_Box__free(Bounding_Box bounding_box);
extern Bounding_Box Bounding_Box__new(void);
extern void Bounding_Box__reset(Bounding_Box bounding_box);
extern void Bounding_Box__free(Bounding_Box bounding_box);
extern void Bounding_Box__update(Bounding_Box bounding_box, double x, double y);

#endif // !defined(BOUNDING_BOX_H_INCLUDED)
