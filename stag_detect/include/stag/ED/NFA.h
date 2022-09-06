#ifndef NFA_H
#define NFA_H

///-----------------------------------------------
/// Lookup table (LUT) for NFA computation
///
struct NFALUT {
 public:
  int *LUT;
  int LUTSize;

  double prob;
  double logNT;

 public:
  /// Constructor
  NFALUT(int size, double prob, double logNT);

  // Destructor
  ~NFALUT() { delete LUT; }  // end-~NFALUT
};

///-------------------------------------------
/// Validation check width/without LUT
///
bool checkValidationByNFA(int n, int k, double prob, double logNT);
bool checkValidationByNFA(int n, int k, NFALUT *lut);

#endif
