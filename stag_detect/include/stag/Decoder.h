#ifndef DECODER_H
#define DECODER_H

#include <vector>
#include <bitset>
#include "stag/MarkerIDs.h"

using std::vector;
using std::bitset;

typedef bitset<48> Codeword;

class Decoder {
  int wordSize = 48;
  int noOfCodewords;

  vector<Codeword> codewords;

 public:
  Decoder() {}
  Decoder(int hd);
  bool decode(const Codeword& c, int errCorr, int& id, int& shift);
};

#endif