#include <stdio.h>
#include <limits.h>
#include <float.h>
#include <math.h>
#include "stag/ED/NFA.h"

/// nfa function prototype
double nfa(int n, int k, double p, double logNT);

///----------------------------------------------
/// Look Up Table (LUT) for NFA Computation
///
NFALUT::NFALUT(int size, double _prob, double _logNT) {
  LUTSize = size;
  LUT = new int[LUTSize];

  prob = _prob;
  logNT = _logNT;

  LUT[0] = 1;
  int j = 1;
  for (int i = 1; i < LUTSize; i++) {
    LUT[i] = LUTSize + 1;
    double ret = nfa(i, j, prob, logNT);
    if (ret < 0) {
      while (j < i) {
        j++;
        ret = nfa(i, j, prob, logNT);
        if (ret >= 0) break;
      }  // end-while

      if (ret < 0) continue;
    }  // end-if

    LUT[i] = j;
  }  // end-for

#if 0
  fprintf(stderr, "================== ENTIRE TABLE ====================\n");
  for (int i=0; i<LUTSize; i++){
    fprintf(stderr, "n: %4d, k: %4d\n", i, LUT[i]);
  } //end-for
#endif
}  // end-LUT

///-------------------------------------------
/// Validation check without the help of a LUT
///
bool checkValidationByNFA(int n, int k, double prob, double logNT) {
  return nfa(n, k, prob, logNT) >= 0.0;
}  // end-checkValidationByNFA

///-------------------------------------------
/// Validation check with the help of a LUT
///
bool checkValidationByNFA(int n, int k, NFALUT *lut) {
  if (n >= lut->LUTSize)
    return nfa(n, k, lut->prob, lut->logNT) >= 0.0;
  else
    return k >= lut->LUT[n];
}  // end-checkValidationByNFA

///------------------------------------------------------
/// The rest of this code used for NFA computation is directly taken from
/// the LSD code distribution. Hope they do not mind :-)
///
#define error(str) \
  {                \
    printf(str);   \
    exit(1);       \
  }

/** ln(10) */
#ifndef M_LN10
#define M_LN10 2.30258509299404568402
#endif /* !M_LN10 */

/** PI */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif /* !M_PI */

#ifndef FALSE
#define FALSE 0
#endif /* !FALSE */

#ifndef TRUE
#define TRUE 1
#endif /* !TRUE */

/** Label for pixels with undefined gradient. */
#define NOTDEF -1024.0

/** 3/2 pi */
#define M_3_2_PI 4.71238898038

/** 2 pi */
#define M_2__PI 6.28318530718

/** Label for pixels not used in yet. */
#define NOTUSED 0

/** Label for pixels already used in detection. */
#define USED 1

#define RELATIVE_ERROR_FACTOR 100.0

static int double_equal(double a, double b) {
  double abs_diff, aa, bb, abs_max;

  /* trivial case */
  if (a == b) return TRUE;

  abs_diff = fabs(a - b);
  aa = fabs(a);
  bb = fabs(b);
  abs_max = aa > bb ? aa : bb;

  /* DBL_MIN is the smallest normalized number, thus, the smallest
     number whose relative error is bounded by DBL_EPSILON. For
     smaller numbers, the same quantization steps as for DBL_MIN
     are used. Then, for smaller numbers, a meaningful "relative"
     error should be computed by dividing the difference by DBL_MIN. */
  if (abs_max < DBL_MIN) abs_max = DBL_MIN;

  /* equal if relative error <= factor x eps */
  return (abs_diff / abs_max) <= (RELATIVE_ERROR_FACTOR * DBL_EPSILON);
}

#define TABSIZE 100000

static double log_gamma_windschitl(double x) {
  return 0.918938533204673 + (x - 0.5) * log(x) - x +
         0.5 * x * log(x * sinh(1 / x) + 1 / (810.0 * pow(x, 6.0)));
}

static double log_gamma_lanczos(double x) {
  static double q[7] = {75122.6331530, 80916.6278952, 36308.2951477,
                        8687.24529705, 1168.92649479, 83.8676043424,
                        2.50662827511};
  double a = (x + 0.5) * log(x + 5.5) - (x + 5.5);
  double b = 0.0;
  int n;

  for (n = 0; n < 7; n++) {
    a -= log(x + (double)n);
    b += q[n] * pow(x, (double)n);
  }
  return a + log(b);
}

#define log_gamma(x) \
  ((x) > 15.0 ? log_gamma_windschitl(x) : log_gamma_lanczos(x))

double nfa(int n, int k, double p, double logNT) {
  static double inv[TABSIZE]; /* table to keep computed inverse values */
  double tolerance = 0.1;     /* an error of 10% in the result is accepted */
  double log1term, term, bin_term, mult_term, bin_tail, err, p_term;
  int i;

  /* check parameters */
  if (n < 0 || k < 0 || k > n || p <= 0.0 || p >= 1.0) return -1.0;

  /* trivial cases */
  if (n == 0 || k == 0) return -logNT;
  if (n == k) return -logNT - (double)n * log10(p);

  /* probability term */
  p_term = p / (1.0 - p);

  /* compute the first term of the series */
  /*
     binomial_tail(n,k,p) = sum_{i=k}^n bincoef(n,i) * p^i * (1-p)^{n-i}
     where bincoef(n,i) are the binomial coefficients.
     But
       bincoef(n,k) = gamma(n+1) / ( gamma(k+1) * gamma(n-k+1) ).
     We use this to compute the first term. Actually the log of it.
   */
  log1term = log_gamma((double)n + 1.0) - log_gamma((double)k + 1.0) -
             log_gamma((double)(n - k) + 1.0) + (double)k * log(p) +
             (double)(n - k) * log(1.0 - p);
  term = exp(log1term);

  /* in some cases no more computations are needed */
  if (double_equal(term, 0.0)) {         /* the first term is almost zero */
    if ((double)k > (double)n * p)       /* at begin or end of the tail?  */
      return -log1term / M_LN10 - logNT; /* end: use just the first term  */
    else
      return -logNT; /* begin: the tail is roughly 1  */
  }                  // end-if

  /* compute more terms if needed */
  bin_tail = term;
  for (i = k + 1; i <= n; i++) {
    /*
       As
         term_i = bincoef(n,i) * p^i * (1-p)^(n-i)
       and
         bincoef(n,i)/bincoef(n,i-1) = n-1+1 / i,
       then,
         term_i / term_i-1 = (n-i+1)/i * p/(1-p)
       and
         term_i = term_i-1 * (n-i+1)/i * p/(1-p).
       1/i is stored in a table as they are computed,
       because divisions are expensive.
       p/(1-p) is computed only once and stored in 'p_term'.
     */
    bin_term =
        (double)(n - i + 1) *
        (i < TABSIZE ? (inv[i] != 0.0 ? inv[i] : (inv[i] = 1.0 / (double)i))
                     : 1.0 / (double)i);

    mult_term = bin_term * p_term;
    term *= mult_term;
    bin_tail += term;

    if (bin_term < 1.0) {
      /* When bin_term<1 then mult_term_j<mult_term_i for j>i.
         Then, the error on the binomial tail when truncated at
         the i term can be bounded by a geometric series of form
         term_i * sum mult_term_i^j.                            */
      err = term *
            ((1.0 - pow(mult_term, (double)(n - i + 1))) / (1.0 - mult_term) -
             1.0);

      /* One wants an error at most of tolerance*final_result, or:
         tolerance * abs(-log10(bin_tail)-logNT).
         Now, the error that can be accepted on bin_tail is
         given by tolerance*final_result divided by the derivative
         of -log10(x) when x=bin_tail. that is:
         tolerance * abs(-log10(bin_tail)-logNT) / (1/bin_tail)
         Finally, we truncate the tail if the error is less than:
         tolerance * abs(-log10(bin_tail)-logNT) * bin_tail        */
      if (err < tolerance * fabs(-log10(bin_tail) - logNT) * bin_tail) break;
    }  // end-if
  }    // end-for

  return -log10(bin_tail) - logNT;
}  // end-nfa

#if 0
///-----------------------------------------------------------------------------------------
/// Check is a line is valid
///
bool IsLineValid(double b, int invert, double *x, double *y, int noPixels, EDImage *ed){
  // Compute Line's angle
  double lineAngle;

  if (invert == 0){
    // y = a + bx
    lineAngle = atan(b);

  } else {
    // x = a + by
    lineAngle = atan(1.0/b);
  } //end-else

  if (lineAngle < 0) lineAngle += PI;

  unsigned char *image = ed->image;
  int width = ed->width;
  int height = ed->height;

  double logNT = 2.0*(log10((double)width) + log10((double)height));

#define PRECISON_ANGLE 22.5
  double prec = (PRECISON_ANGLE/180)*PI;
  double prob = 0.125;
#undef PRECISON_ANGLE

  static short segmentNo = 1000;
  if (++segmentNo >= 32000) segmentNo = 1000;

  int aligned = 0;
  int count = 0;
  for(int i=1; i<noPixels; i++){
    Pixel validPixels[4];
    int noValidPixels = 1;

    int pr = (int)y[i-1];
    int pc = (int)x[i-1];

    int r = (int)y[i];
    int c = (int)x[i];

    validPixels[0].r = r;
    validPixels[0].c = c;

    if (r == pr){
      validPixels[1].r = r-1;
      validPixels[1].c = c;

      validPixels[2].r = r+1;
      validPixels[2].c = c;

      noValidPixels = 3;

    } else if (c == pc){
      validPixels[1].r = r;
      validPixels[1].c = c-1;

      validPixels[2].r = r;
      validPixels[2].c = c+1;

      noValidPixels = 3;

    } else {
      if (r == pr-1){
        if (c == pc-1){
          validPixels[1].r = r;
          validPixels[1].c = c+1;

          validPixels[2].r = r+1;
          validPixels[2].c = c;

          noValidPixels = 3;

        } else if (c == pc+1){
          validPixels[1].r = r;
          validPixels[1].c = c-1;

          validPixels[2].r = r+1;
          validPixels[2].c = c;

          noValidPixels = 3;
        } // end-else

      } else if (r == pr+1){
        if (c == pc-1){
          validPixels[1].r = r;
          validPixels[1].c = c+1;

          validPixels[2].r = r-1;
          validPixels[2].c = c;

          noValidPixels = 3;

        } else if (c == pc+1){
          validPixels[1].r = r;
          validPixels[1].c = c-1;

          validPixels[2].r = r-1;
          validPixels[2].c = c;

          noValidPixels = 3;
        } // end-else
      } // end-else
    } //end-else

    for (int k=0; k<noValidPixels; k++){
      r = validPixels[k].r;
      c = validPixels[k].c;
      if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) continue;
      if (ed->gradImg[r*width+c] == segmentNo) continue;
      ed->gradImg[r*width+c] = segmentNo;

      count++;

#if 0
      // compute gx & gy using the LSD filter
      // LSD gradient computation
      // A B
      // C D
      // gx = (B-A) + (D-C)
      // gy = (C-A) + (D-B)
      //
      // To make this faster: 
      // com1 = (D-A)
      // com2 = (B-C)
      // Then: gx = com1 + com2 = (D-A) + (B-C) = (B-A) + (D-C)
      //       gy = com1 - com2 = (D-A) - (B-C) = (C-A) + (D-B)
      // 
      int com1 = image[(r+1)*width+c+1] - image[r*width+c];
      int com2 = image[r*width+c+1] - image[(r+1)*width+c];

      int gx = (com1 + com2);
      int gy = (com1 - com2);
#else
      // compute gx & gy using the Sobel Filter
      // Faster method below
      // A B C
      // D x E
      // F G H
      // gx = (C-A) + 2*(E-D) + (H-F)
      // gy = (F-A) + 2*(G-B) + (H-C)
      //
      // To make this faster: 
      // com1 = (H-A)
      // com2 = (C-F)
      // Then: gx = com1 + com2 + 2*(E-D) = (H-A) + (C-F) + 2*(E-D) = (C-A) + 2*(E-D) + (H-F)
      //       gy = com2 - com1 + 2*(G-B) = (H-A) - (C-F) + 2*(G-B) = (F-A) + 2*(G-B) + (H-C)
      // 
      int com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
      int com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

      int gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
      int gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));
#endif

//        double pixelAngle = atan2((double)gx, (double)-gy);
//        if (pixelAngle < 0) pixelAngle += PI;
      double pixelAngle = myAtan2((double)gx, (double)-gy);
      double diff = fabs(lineAngle - pixelAngle);

      if (diff <= prec || diff >= PI-prec) aligned++;
    } //end-for
  } //end-for

  return nfa(count, aligned, prob, logNT) >= 0.0;
} //end-IsLineValid

///-----------------------------------------------------------------------------------------
/// Check is a line is valid
///
bool IsLineValid(LineSegment *ls, EDImage *ed){
  // Compute Line's angle
  double lineAngle;

  if (ls->invert == 0){
    // y = a + bx
    lineAngle = atan(ls->b);

  } else {
    // x = a + by
    lineAngle = atan(1.0/ls->b);
  } //end-else

  if (lineAngle < 0) lineAngle += PI;

  unsigned char *image = ed->image;
  int width = ed->width;
  int height = ed->height;

  double logNT = 2.0*(log10((double)width) + log10((double)height));

#define PRECISON_ANGLE 22.5
  double prec = (PRECISON_ANGLE/180)*PI;
  double prob = 0.125;
#undef PRECISON_ANGLE

  Pixel *pixels = &ed->map->segments[ls->segmentNo].pixels[ls->firstPixelIndex];
  int noPixels = ls->len;

  static short segmentNo = 1000;
  if (++segmentNo >= 32000) segmentNo = 1000;

  int aligned = 0;
  int count = 0;
  for(int i=1; i<noPixels; i++){
    Pixel validPixels[4];
    int noValidPixels = 1;

    int pr = pixels[i-1].r;
    int pc = pixels[i-1].c;

    int r = pixels[i].r;
    int c = pixels[i].c;

    validPixels[0].r = r;
    validPixels[0].c = c;

    if (r == pr){
      validPixels[1].r = r-1;
      validPixels[1].c = c;

      validPixels[2].r = r+1;
      validPixels[2].c = c;

      noValidPixels = 3;

    } else if (c == pc){
      validPixels[1].r = r;
      validPixels[1].c = c-1;

      validPixels[2].r = r;
      validPixels[2].c = c+1;

      noValidPixels = 3;

    } else {
      if (r == pr-1){
        if (c == pc-1){
          validPixels[1].r = r;
          validPixels[1].c = c+1;

          validPixels[2].r = r+1;
          validPixels[2].c = c;

          noValidPixels = 3;

        } else if (c == pc+1){
          validPixels[1].r = r;
          validPixels[1].c = c-1;

          validPixels[2].r = r+1;
          validPixels[2].c = c;

          noValidPixels = 3;
        } // end-else

      } else if (r == pr+1){
        if (c == pc-1){
          validPixels[1].r = r;
          validPixels[1].c = c+1;

          validPixels[2].r = r-1;
          validPixels[2].c = c;

          noValidPixels = 3;

        } else if (c == pc+1){
          validPixels[1].r = r;
          validPixels[1].c = c-1;

          validPixels[2].r = r-1;
          validPixels[2].c = c;

          noValidPixels = 3;
        } // end-else
      } // end-else
    } //end-else

    for (int k=0; k<noValidPixels; k++){
      r = validPixels[k].r;
      c = validPixels[k].c;
      if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) continue;
      if (ed->gradImg[r*width+c] == segmentNo) continue;
      ed->gradImg[r*width+c] = segmentNo;

      count++;

#if 0
      // compute gx & gy using the LSD filter
      // LSD gradient computation
      // A B
      // C D
      // gx = (B-A) + (D-C)
      // gy = (C-A) + (D-B)
      //
      // To make this faster: 
      // com1 = (D-A)
      // com2 = (B-C)
      // Then: gx = com1 + com2 = (D-A) + (B-C) = (B-A) + (D-C)
      //       gy = com1 - com2 = (D-A) - (B-C) = (C-A) + (D-B)
      // 
      int com1 = image[(r+1)*width+c+1] - image[r*width+c];
      int com2 = image[r*width+c+1] - image[(r+1)*width+c];

      int gx = (com1 + com2);
      int gy = (com1 - com2);
#else
      // compute gx & gy using the Sobel Filter
      // Faster method below
      // A B C
      // D x E
      // F G H
      // gx = (C-A) + 2*(E-D) + (H-F)
      // gy = (F-A) + 2*(G-B) + (H-C)
      //
      // To make this faster: 
      // com1 = (H-A)
      // com2 = (C-F)
      // Then: gx = com1 + com2 + 2*(E-D) = (H-A) + (C-F) + 2*(E-D) = (C-A) + 2*(E-D) + (H-F)
      //       gy = com2 - com1 + 2*(G-B) = (H-A) - (C-F) + 2*(G-B) = (F-A) + 2*(G-B) + (H-C)
      // 
      int com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
      int com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

      int gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
      int gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));
#endif

//        double pixelAngle = atan2((double)gx, (double)-gy);
//        if (pixelAngle < 0) pixelAngle += PI;
      double pixelAngle = myAtan2((double)gx, (double)-gy);
      double diff = fabs(lineAngle - pixelAngle);

      if (diff <= prec || diff >= PI-prec) aligned++;
    } //end-for
  } //end-for

  return nfa(count, aligned, prob, logNT) >= 0.0;
} //end-IsLineValid

///-----------------------------------------------------------
/// Check is a line is valid -- Faster implementation of the Fast method
///
bool IsLineValidFaster(LineSegment *ls, EDImage *ed){
  // Compute Line's angle
  double lineAngle;

  if (ls->invert == 0){
    // y = a + bx
    lineAngle = atan(ls->b);

  } else {
    // x = a + by
    lineAngle = atan(1.0/ls->b);
  } //end-else

  if (lineAngle < 0) lineAngle += PI;

  unsigned char *image = ed->image;
  int width = ed->width;
  int height = ed->height;

  double logNT = 2.0*(log10((double)width) + log10((double)height));

#define PRECISON_ANGLE 22.5
  double prec = (PRECISON_ANGLE/180)*PI;
  double prob = 0.125;
#undef PRECISON_ANGLE

  Pixel *pixels = &ed->map->segments[ls->segmentNo].pixels[ls->firstPixelIndex];
  int noPixels = ls->len;

  static short segmentNo = 1000;
  if (++segmentNo >= 32000) segmentNo = 1000;

  int aligned = 0;
  int count = 0;
  for(int i=1; i<noPixels; i++){
    int pr = pixels[i-1].r;
    int pc = pixels[i-1].c;

    int r = pixels[i].r;
    int c = pixels[i].c;

    if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) goto nextPixel1;
    if (ed->gradImg[r*width+c] == segmentNo) goto nextPixel1;
    ed->gradImg[r*width+c] = segmentNo;

    count++;
    
    int com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
    int com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

    int gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
    int gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

    double pixelAngle = myAtan2((double)gx, (double)-gy);
    double diff = fabs(lineAngle - pixelAngle);
    if (diff <= prec || diff >= PI-prec) aligned++;

nextPixel1:

    if (r == pr){
      r = pixels[i].r-1;
      c = pixels[i].c;

      if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) goto nextPixel2;
      if (ed->gradImg[r*width+c] == segmentNo) goto nextPixel2;
      ed->gradImg[r*width+c] = segmentNo;

      count++;
      
      com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
      com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

      gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
      gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

      pixelAngle = myAtan2((double)gx, (double)-gy);
      diff = fabs(lineAngle - pixelAngle);
      if (diff <= prec || diff >= PI-prec) aligned++;

nextPixel2:
      r = pixels[i].r+1;
      c = pixels[i].c;

      if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) continue;
      if (ed->gradImg[r*width+c] == segmentNo) continue;
      ed->gradImg[r*width+c] = segmentNo;

      count++;
      
      com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
      com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

      gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
      gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

      pixelAngle = myAtan2((double)gx, (double)-gy);
      diff = fabs(lineAngle - pixelAngle);
      if (diff <= prec || diff >= PI-prec) aligned++;

    } else if (c == pc){
      r = pixels[i].r;
      c = pixels[i].c-1;

      if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) goto nextPixel3;
      if (ed->gradImg[r*width+c] == segmentNo) goto nextPixel3;
      ed->gradImg[r*width+c] = segmentNo;

      count++;
      
      com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
      com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

      gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
      gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

      pixelAngle = myAtan2((double)gx, (double)-gy);
      diff = fabs(lineAngle - pixelAngle);
      if (diff <= prec || diff >= PI-prec) aligned++;

nextPixel3:
      r = pixels[i].r;
      c = pixels[i].c+1;

      if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) continue;
      if (ed->gradImg[r*width+c] == segmentNo) continue;
      ed->gradImg[r*width+c] = segmentNo;

      count++;
      
      com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
      com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

      gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
      gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

      pixelAngle = myAtan2((double)gx, (double)-gy);
      diff = fabs(lineAngle - pixelAngle);
      if (diff <= prec || diff >= PI-prec) aligned++;

    } else {
      if (r == pr-1){
        if (c == pc-1){
          r = pixels[i].r;
          c = pixels[i].c+1;

          if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) goto nextPixel4;
          if (ed->gradImg[r*width+c] == segmentNo) goto nextPixel4;
          ed->gradImg[r*width+c] = segmentNo;

          count++;
          
          com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
          com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

          gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
          gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

          pixelAngle = myAtan2((double)gx, (double)-gy);
          diff = fabs(lineAngle - pixelAngle);
          if (diff <= prec || diff >= PI-prec) aligned++;

  nextPixel4:
          r = pixels[i].r+1;
          c = pixels[i].c;

          if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) continue;
          if (ed->gradImg[r*width+c] == segmentNo) continue;
          ed->gradImg[r*width+c] = segmentNo;

          count++;
          
          com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
          com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

          gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
          gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

          pixelAngle = myAtan2((double)gx, (double)-gy);
          diff = fabs(lineAngle - pixelAngle);
          if (diff <= prec || diff >= PI-prec) aligned++;

        } else if (c == pc+1){
          r = pixels[i].r;
          c = pixels[i].c-1;

          if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) goto nextPixel5;
          if (ed->gradImg[r*width+c] == segmentNo) goto nextPixel5;
          ed->gradImg[r*width+c] = segmentNo;

          count++;
          
          com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
          com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

          gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
          gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

          pixelAngle = myAtan2((double)gx, (double)-gy);
          diff = fabs(lineAngle - pixelAngle);
          if (diff <= prec || diff >= PI-prec) aligned++;

  nextPixel5:
          r = pixels[i].r+1;
          c = pixels[i].c;

          if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) continue;
          if (ed->gradImg[r*width+c] == segmentNo) continue;
          ed->gradImg[r*width+c] = segmentNo;

          count++;
          
          com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
          com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

          gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
          gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

          pixelAngle = myAtan2((double)gx, (double)-gy);
          diff = fabs(lineAngle - pixelAngle);
          if (diff <= prec || diff >= PI-prec) aligned++;
        } // end-else

      } else if (r == pr+1){
        if (c == pc-1){
          r = pixels[i].r;
          c = pixels[i].c+1;

          if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) goto nextPixel6;
          if (ed->gradImg[r*width+c] == segmentNo) goto nextPixel6;
          ed->gradImg[r*width+c] = segmentNo;

          count++;
          
          com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
          com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

          gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
          gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

          pixelAngle = myAtan2((double)gx, (double)-gy);
          diff = fabs(lineAngle - pixelAngle);
          if (diff <= prec || diff >= PI-prec) aligned++;

  nextPixel6:
          r = pixels[i].r-1;
          c = pixels[i].c;

          if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) continue;
          if (ed->gradImg[r*width+c] == segmentNo) continue;
          ed->gradImg[r*width+c] = segmentNo;

          count++;
          
          com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
          com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

          gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
          gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

          pixelAngle = myAtan2((double)gx, (double)-gy);
          diff = fabs(lineAngle - pixelAngle);
          if (diff <= prec || diff >= PI-prec) aligned++;

        } else if (c == pc+1){
          r = pixels[i].r;
          c = pixels[i].c-1;

          if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) goto nextPixel8;
          if (ed->gradImg[r*width+c] == segmentNo) goto nextPixel8;
          ed->gradImg[r*width+c] = segmentNo;

          count++;
          
          com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
          com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

          gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
          gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

          pixelAngle = myAtan2((double)gx, (double)-gy);
          diff = fabs(lineAngle - pixelAngle);
          if (diff <= prec || diff >= PI-prec) aligned++;

  nextPixel8:
          r = pixels[i].r-1;
          c = pixels[i].c;

          if (r<=0|| r>=height-1 || c<=0 ||c>=width-1) continue;
          if (ed->gradImg[r*width+c] == segmentNo) continue;
          ed->gradImg[r*width+c] = segmentNo;

          count++;
          
          com1 = image[(r+1)*width+c+1] - image[(r-1)*width+c-1];
          com2 = image[(r-1)*width+c+1] - image[(r+1)*width+c-1];

          gx = (com1 + com2 + 2*(image[r*width+c+1] - image[r*width+c-1]));
          gy = (com1 - com2 + 2*(image[(r+1)*width+c] - image[(r-1)*width+c]));

          pixelAngle = myAtan2((double)gx, (double)-gy);
          diff = fabs(lineAngle - pixelAngle);
          if (diff <= prec || diff >= PI-prec) aligned++;
        } // end-else
      } // end-else
    } // end-else
  } //end-for

  return nfa(count, aligned, prob, logNT) >= 0.0;
} //end-IsLineValidFaster
#endif
